//
// Created by manuelli on 10/12/15.
//

#include "ContactFilter.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <math.h>
#include "gurobiUtil.hpp"
#include "lcmtypes/drc/contact_filter_estimate_t.hpp"
#include "lcmtypes/drc/contact_filter_body_wrench_estimate_t.hpp"
#include <tuple>
#define MU 0.6
#define DETECTION_THRESHOLD 0.2
#define PUBLISH_CHANNEL "CONTACT_FILTER_POINT_ESTIMATE"
#define BODY_WRENCH_PUBLISH_CHANNEL "CONTACT_FILTER_BODY_WRENCH_ESTIMATE"
#include <utility>


const double pi = 3.14159265358;
using namespace Eigen;

ContactFilter::ContactFilter(void):grbModel(grbEnv),grbModelInitialized(false){
  this->mu = MU;
  this->detectionThreshold = DETECTION_THRESHOLD;
  this->initializeGurobiModel();
  this->initializeFrictionCone();
  this->publishChannel = PUBLISH_CHANNEL;
  this->bodyWrenchPublishChannel = BODY_WRENCH_PUBLISH_CHANNEL;

  //check that our lcm publisher is good
  if(!lcm.good()){
    std::cerr << "Error: lcm is not good()" << std::endl;
  }
}
ContactFilter::~ContactFilter(void){
}

//ContactFilter::ContactFilter(std::string URDFString):grbModel(grbEnv), grbModelInitialized(false){
//  this->mu = MU;
//  this->addRobotFromURDFString(URDFString);
//
//}

void ContactFilter::addRobotFromURDFString(std::string URDFString){
  this->drake_model.addRobotFromURDFString(URDFString);
  this->drake_model.compile();
  this->initializeDrakeModelDetails();
}

void ContactFilter::addRobotFromURDF(std::string urdfFilename){
  if (urdfFilename.empty()){
    std::cout << "no urdf received, building atlas_v5 minimal contact model" << std::endl;
    std::string drcBase = std::string(std::getenv("DRC_BASE"));
    urdfFilename = drcBase + "/software/models/atlas_v5/model_LR_RR.urdf";
  }

  this->drake_model.addRobotFromURDF(urdfFilename);
  this->drake_model.compile();
  this->initializeDrakeModelDetails();
}

void ContactFilter::initializeDrakeModelDetails(){
  this->nq = drake_model.num_positions;
  this->nv = drake_model.num_velocities;
  this->W = MatrixXd::Identity(this->nv, this->nv);
  this->determinantW = this->W.determinant();
  this->detectionThresholdVec = this->detectionThreshold*VectorXd::Ones(this->nv);

  for(int i = 0; i < this->nv; i++){
    this->velocity_names.push_back(this->drake_model.getVelocityName(i));
  }

  for (const auto& rigidBodyPointer: this->drake_model.bodies){
    std::pair<std::shared_ptr<RigidBody>, std::set<std::shared_ptr<RigidBody>>> mapPair;
    mapPair.first = rigidBodyPointer;
    this->rigidBodyChildrenMap.insert(mapPair);
  }

  // populate the children map
  for (auto & p: this->rigidBodyChildrenMap){
    if (p.first->hasParent()){
      this->rigidBodyChildrenMap.at(p.first->parent).insert(p.first);
    }
  }

  //figure out which are the leaf nodes
  for (auto &p: this->rigidBodyChildrenMap){
    if (p.second.empty()){
      this->rigidBodyLeafNodes.insert(p.first);
    }
  }
}

void ContactFilter::initializeFrictionCone() {
  this->FrictionCone << this->mu, -this->mu, 0, 0,
                        0, 0, this->mu, -this->mu,
                        1,1,1,1;

}

void ContactFilter::addContactPoints(std::vector<ContactFilterPoint> contactPointsToAdd) {
  for ( auto cfp: contactPointsToAdd){
    // make sure the body_id is correct for the model we are using
    // always go by body_names, this is the safest
    cfp.body_id = this->drake_model.findLinkId(cfp.body_name);
    this->contactPoints.push_back(cfp);

    // also insert it into the contactPointMap according to the body it is on
    if (this->contactPointMap.count(cfp.body_name) > 0){
      this->contactPointMap.at(cfp.body_name).push_back(cfp);
    }
    else{
      std::vector<ContactFilterPoint> vec{cfp};
      this->contactPointMap.insert(std::make_pair(cfp.body_name, vec));
    }
  }
}

MeasurementUpdate ContactFilter::computeLikelihood(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                                        const ContactFilterPoint& contactFilterPoint, bool publish){
  bool verbose = false;
  // suppress gurobi output to terminal if we are not in verbose mode
  if (!verbose){
    this->grbModel.getEnv().set(GRB_IntParam_OutputFlag, 0);
  }

  this->drake_model.doKinematics(q, v, false, false);
//  Vector3d points << 0,0,0;
//  MatrixXd R_world_to_body = this->drake_model.forwardKin(points, )
  std::vector<int> v_indices;

  //unpack the contactFilterPoint struct
  const Vector3d & contactNormal = contactFilterPoint.contactNormal;
  const Vector3d & contactPoint = contactFilterPoint.contactPoint;
  const int & body_id = contactFilterPoint.body_id;


  MatrixXd linkJacobian = this->drake_model.geometricJacobian<double>(0, body_id, body_id, 0, false, &v_indices).value();

  // need to convert this to full sized Jacobian!
  MatrixXd linkJacobianFull = MatrixXd::Zero(6, this->nv);
  for (int i=0; i < v_indices.size(); i++){
    linkJacobianFull.col(v_indices[i]) = linkJacobian.col(i);
  }

  // want to formulate QP with objective (r - J_force^T(\sum_i alpha_i F_c,i))^T W (r - J_force^T(\sum_i alpha_i F_c,i))

  //first need to rotate the friction cone the link frame
  Vector3d z_vector;
  z_vector << 0,0,1;
  Matrix<double,3,3> rotationMatrix = rotateVectorToAlign(z_vector, contactNormal);
  Matrix<double, 3,4> rotatedFrictionCone = rotationMatrix*this->FrictionCone;
  Matrix<double, 6, 3> forceToWrench = computeForceToWrenchTransform(contactPoint);
  // still need to do force moment transformation from contactPoint to link frame


  // want to write objective in the form (r - H alpha)^T W (r - H alpha)
  MatrixXd H = linkJacobianFull.transpose()*forceToWrench*rotatedFrictionCone;

  // now construct matrices needed to pass model to gurobi, write objective in the form x^T Q x + f x
  // Q = H^T W H, f= - r^T W H
  MatrixXd Q = H.transpose()*this->W*H;
  VectorXd f = -2*residual.transpose()*this->W*H;

  if (verbose){
    std::cout << "linkName = " + this->drake_model.getBodyOrFrameName(body_id) << std::endl;
    std::cout << "rotated friction cone:\n" << rotatedFrictionCone << std::endl;
    std::cout << "force to wrench transform:\n" << forceToWrench << std::endl;
    std::cout << "linkJacobianSmall:\n" << linkJacobian << std::endl;
    std::cout << "linkJacobianSmall.transpose()*forceToWrench:\n" << linkJacobian.transpose()*forceToWrench << std::endl;
    std::cout << "linkJacobianFull:\n" << linkJacobianFull << std::endl;
    std::cout << "Q matrix:\n" << Q << std::endl;
  }

  //clear the objective value of the current model
  GRBLinExpr linExpr(0.0);
  this->grbModel.setObjective(linExpr);
  this->grbModel.update();


  GRBVar* varsArray = this->grbModel.getVars();
  // convert to std
  std::vector<GRBVar*> vars;
  int num_vars = this->grbModel.get(GRB_IntAttr_NumVars);
  int i = 0;
  for (GRBVar* var = varsArray; i<num_vars; var++, i++){
    vars.push_back(var);
  }
  gurobiAddObjective(this->grbModel, Q, f, this->grbVars);
  delete [] varsArray;


  // solve
  this->grbModel.optimize();

  //parse output and publish
  VectorXd alphaArgMin = gurobiArgMinAsVectorXd(this->grbModel);
  VectorXd forceInBodyFrame = rotatedFrictionCone*alphaArgMin;
  VectorXd estResidual = linkJacobianFull.transpose()*forceToWrench*forceInBodyFrame;
  double exponentVal = -1/2.0*(residual - estResidual).transpose()*this->W*(residual - estResidual);
  double likelihood = exp(exponentVal); //up to a constant
  MeasurementUpdate measurementUpdate = {t, likelihood, exponentVal, forceInBodyFrame, estResidual, contactFilterPoint};

  if (publish){
    this->publishPointEstimate(measurementUpdate);
  }

  if (verbose){
    this->printGurobiModel();
    std::cout << "estimated force in body frame is " << std::endl;
    std::cout << forceInBodyFrame << std::endl;
    std::cout << "likelihood is " << likelihood << std::endl;
    std::cout << "exponentVal = " << exponentVal << std::endl;
  }

  return measurementUpdate;

}

void ContactFilter::publishPointEstimate(const MeasurementUpdate &measurementUpdate, bool publishOnUniqueChannel){



//  drc::contact_filter_estimate_t msg;
//  msg.utime = static_cast<int64_t> (measurementUpdate.t*1e6);
//  msg.body_id = (int16_t) measurementUpdate.contactFilterPoint.body_id;
//  msg.body_name = this->drake_model.getBodyOrFrameName(measurementUpdate.contactFilterPoint.body_id);
//  msg.name = measurementUpdate.contactFilterPoint.name;
//  msg.velocity_names = this->velocity_names;
//  msg.likelihood = (float) measurementUpdate.likelihood;
//  msg.logLikelihood = (float) measurementUpdate.exponentVal;
//  msg.contact_force_magnitude = (float) measurementUpdate.forceInBodyFrame.norm();
//
//  for(int i=0; i < 3; i++){
//    msg.contact_position[i] = (float) measurementUpdate.contactFilterPoint.contactPoint(i);
//    msg.contact_normal[i] = (float) measurementUpdate.contactFilterPoint.contactNormal(i);
//    msg.contact_force[i] = (float) measurementUpdate.forceInBodyFrame(i);
//  }
//
//  msg.num_velocities = (int16_t) this->nv;
//  msg.implied_residual.resize(this->nv);
//  for(int j=0; j<this->nv; j++){
//    msg.implied_residual[j] = (float) measurementUpdate.estResidual(j);
//  }
//
//  std::string channel = this->publishChannel;
//
//  if (publishOnUniqueChannel){
//    if (measurementUpdate.contactFilterPoint.name.length() > 0){
//      channel = channel + "_" + measurementUpdate.contactFilterPoint.name;
//    }
//    else{
//      channel = channel + "_NO_NAME";
//    }
//  }
//
//  this->lcm.publish(channel, &msg);

}


// loop through the contact points in this->contactPoints, run the measurement update, store the pertinent
// information in this->measurementUpdateVec
void ContactFilter::computeLikelihoodFull(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &v, bool publishMostLikely, bool publishAll){
  this->measurementUpdateVec.clear();
  for (int i=0; i<this->contactPoints.size(); i++){
    this->measurementUpdateVec.push_back(this->computeLikelihood(t, residual, q, v, this->contactPoints[i], publishAll));
  }

  if (publishMostLikely){
      this->publishMostLikelyEstimate();
    }

}

MultiContactMeasurementUpdate ContactFilter::computeMultiContactLikelihood(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                                                             const Eigen::VectorXd &v,
                                                             const std::vector<ContactFilterPoint>& contactFilterPointsVec,
                                                             bool publish){
  MultiContactMeasurementUpdate mcMeasurementUpdate;

  // need to actually implement this, requires setting up a new QP type problem
  // should be able to handle different numbers of active contact points

  return mcMeasurementUpdate;
}

void ContactFilter::computeMultiContactLikelihoodFull(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                                       const Eigen::VectorXd &v, std::vector<std::string> activeLinks,
                                       bool publishMostLikely,
                                       bool publishAll){


  this->multiContactMeasurementUpdateVec.clear();

  std::vector<std::vector<ContactFilterPoint>::iterator> it;
  std::vector<std::vector<ContactFilterPoint>> vec;
  int K = activeLinks.size();

  for (auto activeLinkName: activeLinks){
    auto & contactFilterPointVec = this->contactPointMap.at(activeLinkName);
    it.push_back(contactFilterPointVec.begin());
    vec.push_back(contactFilterPointVec);
  }

  while (it[0] != vec[0].end()) {

    std::vector<ContactFilterPoint> contactPointVec;
    for (int i = 0; i < K; i++){
      contactPointVec.push_back(*it[i]);
    }

    auto mcMeasurementUpdate = this->computeMultiContactLikelihood(t, residual, q, v, contactPointVec, publishAll);
    this->multiContactMeasurementUpdateVec.push_back(mcMeasurementUpdate);
    // the following increments the "odometer" by 1
    ++it[K-1];
    for (int i = K-1; (i > 0) && (it[i] == vec[i].end()); --i) {
      it[i] = vec[i].begin();
      ++it[i-1];
    }
  }


}

// loop through this->measurementUpdateVec, record the most likely estimate and publish it
void ContactFilter::publishMostLikelyEstimate() {

  // if there is nothing in measurementUpdateVec just return, there was nothing to do
  if (this->measurementUpdateVec.size() == 0){
    return;
  }

  MeasurementUpdate* mostLikelyMeasurementUpdate = & this->measurementUpdateVec[0];
  for(auto & measurementUpdate: this->measurementUpdateVec){
    if (measurementUpdate.exponentVal > mostLikelyMeasurementUpdate->exponentVal){
      mostLikelyMeasurementUpdate = &measurementUpdate;
    }
  }

  this->publishPointEstimate(*mostLikelyMeasurementUpdate, false);
}

std::set<std::shared_ptr<RigidBody>> ContactFilter::findActiveLinks(const Eigen::VectorXd &residual) {
  std::set<std::shared_ptr<RigidBody>> activeRigidBodies;
  for (auto &p: this->rigidBodyChildrenMap) {
    auto &rb = p.first;
    if (!rb->hasJoint()) {
      continue;
    }
    int velocityIdxStart = rb->velocity_num_start;
    int numVelocities = rb->getJoint().getNumVelocities();
    VectorXd temp = residual.segment(velocityIdxStart, numVelocities).cwiseAbs() -
                    this->detectionThresholdVec.segment(velocityIdxStart, numVelocities);
    if ((temp.array() > 0).any()) {
//      std::cout << "potentially active:  " + rb->linkname << std::endl;
      activeRigidBodies.insert(rb);
    }
  }

  // start in the leaf nodes and work backwards. Remove anything from active set that has something further down the tree
  // which is in activeRigidBodies
//  std::cout << std::endl;
  for (auto &rb: this->rigidBodyLeafNodes) {
    std::shared_ptr<RigidBody> firstActiveRB = rb;
    while ((activeRigidBodies.find(firstActiveRB) == activeRigidBodies.end()) && firstActiveRB->hasParent()) {
      //this means it has a parent (i.e. is not the root) and that it is not active
      firstActiveRB = firstActiveRB->parent;
    }

//    std::cout << "leaf node = " + rb->linkname << std::endl;
//    std::cout << "firstActiveRB = " + firstActiveRB->linkname << std::endl;
    std::shared_ptr<RigidBody> current = firstActiveRB;
    while (current->hasParent()) {
      current = current->parent;
//      std::cout << "removing from activeRigidBodies: " + current->linkname << std::endl;
      activeRigidBodies.erase(current);
    }

  }

  return activeRigidBodies;
}

void ContactFilter::computeActiveLinkForceTorque(double t, const Eigen::VectorXd & residual, const Eigen::VectorXd &q,
                                                 const Eigen::VectorXd &v, bool publish, std::vector<std::string> activeLinkNames){

  std::set<std::shared_ptr<RigidBody>> activeRigidBodies;
  if (activeLinkNames.empty()){
     activeRigidBodies = this->findActiveLinks(residual);
  }
  else{
    for (auto& linkName: activeLinkNames){
      activeRigidBodies.insert(this->drake_model.findLink(linkName));
    }
  }


  if (activeRigidBodies.empty()){
    return;
  }

  this->drake_model.doKinematics(q, v, false, false);
  std::vector<MatrixXd> linkJacobianVec;
  std::vector<BodyWrenchEstimate> bodyWrenchEstimateVec;

  for (auto & rb: activeRigidBodies){
    int body_id = this->drake_model.findLinkId(rb->linkname);
    BodyWrenchEstimate bwe;
    bwe.linkName = rb->linkname;
    bwe.jointName = rb->getJoint().getName();
    bodyWrenchEstimateVec.push_back(bwe);

    MatrixXd linkJacobianFull = this->geometricJacobianFull(0, body_id, body_id, 0, false);
    linkJacobianVec.push_back(linkJacobianFull.transpose());
  }

  // construct the large matrix we are going to use to do the pseudo-inverse
  MatrixXd A(this->nv, 6*activeRigidBodies.size());
  for (int i = 0; i < linkJacobianVec.size(); i++){
    int startRow = 0;
    int startColumn = 6*i;
    int blockRows = this->nv;
    int blockCols = 6;
    A.block(startRow, startColumn, blockRows, blockCols) = linkJacobianVec[i];
  }

  VectorXd wrenchEstimateFull = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(residual);
  VectorXd estResidual = A*wrenchEstimateFull;
  double exponentVal = -1/2.0*(residual - estResidual).transpose()*this->W*(residual - estResidual);

  for (int i = 0; i < bodyWrenchEstimateVec.size(); i++){
    BodyWrenchEstimate& bwe = bodyWrenchEstimateVec[i];
    bwe.exponentVal = exponentVal;
    bwe.estResidual = estResidual;
    bwe.t = t;
    bwe.wrench = wrenchEstimateFull.segment(6*i, 6);
  }


  if (publish){
    this->publishBodyFrameWrench(bodyWrenchEstimateVec);
  }

}

void ContactFilter::publishBodyFrameWrench(const std::vector<BodyWrenchEstimate> & wrenchEstimateVec){

//  std::cout << "attempting to publish body wrench" << std::endl;
//  std::cout << std::endl;

  drc::contact_filter_body_wrench_estimate_t msg;
  msg.num_bodies = static_cast<int32_t> (wrenchEstimateVec.size());
  msg.num_velocities = static_cast<int32_t> (this->nv);
  msg.utime = static_cast<int64_t> (wrenchEstimateVec[0].t*1e6);
  msg.exponentVal = (float) (wrenchEstimateVec[0].exponentVal);
  msg.implied_residual.resize(this->nv);

  for (int i = 0; i < this->nv; i++){
    msg.implied_residual[i] = wrenchEstimateVec[0].estResidual[i];
  }

  msg.body_name.resize(msg.num_bodies);
  msg.joint_name.resize(msg.num_bodies);
  msg.tx.resize(msg.num_bodies);
  msg.ty.resize(msg.num_bodies);
  msg.tz.resize(msg.num_bodies);
  msg.fx.resize(msg.num_bodies);
  msg.fy.resize(msg.num_bodies);
  msg.fz.resize(msg.num_bodies);

  for (int i = 0; i < wrenchEstimateVec.size(); i++){
    const BodyWrenchEstimate & wrenchEstimate = wrenchEstimateVec[i];
    msg.body_name[i] = wrenchEstimate.linkName;
    msg.joint_name[i] = wrenchEstimate.jointName;
    msg.tx[i] = (float) wrenchEstimate.wrench(0);
    msg.ty[i] = (float) wrenchEstimate.wrench(1);
    msg.tz[i] = (float) wrenchEstimate.wrench(2);
    msg.fx[i] = (float) wrenchEstimate.wrench(3);
    msg.fy[i] = (float) wrenchEstimate.wrench(4);
    msg.fz[i] = (float) wrenchEstimate.wrench(5);
  }

  this->lcm.publish(this->bodyWrenchPublishChannel, &msg);

}

// this method assumes that you have already called doKinematics with the desired position, velocity
MatrixXd ContactFilter::geometricJacobianFull(int base_body_or_frame_ind, int end_effector_body_or_frame_ind,
                           int expressed_in_body_or_frame_ind, int gradient_order,
                           bool in_terms_of_qdot){

  std::vector<int> v_indices;
  MatrixXd linkJacobian = this->drake_model.geometricJacobian<double>(base_body_or_frame_ind, end_effector_body_or_frame_ind,
                                                                      expressed_in_body_or_frame_ind, gradient_order,
                                                                      in_terms_of_qdot, &v_indices).value();
  // need to convert this to full sized Jacobian!
  MatrixXd linkJacobianFull = MatrixXd::Zero(6, this->nv);
  for (int i=0; i < v_indices.size(); i++){
    linkJacobianFull.col(v_indices[i]) = linkJacobian.col(i);
  }

  return linkJacobianFull;

}





void ContactFilter::testQP(){
  VectorXd y = VectorXd::Zero(4);
  y << 1,0,0,0;
  MatrixXd H = MatrixXd::Identity(4,4);
  H(0,0) = 0;
  H(0,1) = 1;
  H(1,1) = 0;
  MatrixXd Q = H.transpose()*H;
  VectorXd f = -2*y.transpose()*H;
  std::cout << "f is " << f << std::endl;
  std::cout << "size of f is " << f.size() << std::endl;

  GRBLinExpr linExpr = 0.0;
  this->grbModel.setObjective(linExpr);
  gurobiAddObjective(this->grbModel, Q, f, this->grbVars);
  this->grbModel.optimize();
  this->printGurobiModel();
}


void ContactFilter::printGurobiModel(){

  int modelStatus = this->grbModel.get(GRB_IntAttr_Status);
  if (modelStatus==1){
    std::cout << "model loaded NOT solved\n";
  }
  if (modelStatus == 2){
    std::cout << "model solved to optimality\n";
    for (auto &var: this->grbVars){
      std::cout << var.get(GRB_StringAttr_VarName) << " = " << var.get(GRB_DoubleAttr_X) << std::endl;
    }
  }


  GRBQuadExpr objective = this->grbModel.getObjective();
  GRBLinExpr linObjective = objective.getLinExpr();
  int n = objective.size();
  std::cout << "number of terms in quadratic objective = " << objective.size() << std::endl;
  for( int j = 0; j<n; j++) {
    std::string name1 = objective.getVar1(j).get(GRB_StringAttr_VarName);
    std::string name2 = objective.getVar2(j).get(GRB_StringAttr_VarName);
    std::cout << "coefficient on " + name1 + " " + name2 + " = " << objective.getCoeff(j) << std::endl;
  }

  n = linObjective.size();
  std::cout << "number of linear terms in objective is " << n << std::endl;

  for(int j = 0; j<n; j++){
    std::string name = linObjective.getVar(j).get(GRB_StringAttr_VarName);
    std::cout << "coefficient on linear term " + name + " = " << linObjective.getCoeff(j) << std::endl;
  }
}


void ContactFilter::initializeGurobiModel() {
  if (this->grbModelInitialized){
    return;
  }

  this->grbModelInitialized = true;
  std::string var_name;

  for (int i=0; i < 4; i++){
    var_name = "alpha_" + std::to_string(i);
    this->grbVars.push_back(this->grbModel.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, var_name));
  }

  this->grbModel.update();

}

void ContactFilter::runFindLinkTest() {
  std::cout << "the RigidBody elements are" << std::endl;
  for (auto &p: this->rigidBodyChildrenMap){
    std::cout << "linkname = " << p.first->linkname << std::endl;
    if (p.first->hasJoint()){
      std::cout << "joint name = " << p.first->getJoint().getName() << std::endl;
      std::cout << "position num start = " << p.first->position_num_start << std::endl;
      std::cout << "joint num velocities = " << p.first->getJoint().getNumVelocities() << std::endl;
    }
    else{
      std::cout << "this body has no joint" << std::endl;
    }
    std::cout << std::endl;
  }

  std::cout << "the leaf nodes are" << std::endl;

  for (auto& p: this->rigidBodyLeafNodes){
    std::cout << p->linkname << std::endl;
  }

  std::cout << std::endl;

  VectorXd residual = VectorXd::Zero(this->nv);
  residual(5) = 2.0;
  residual(7) = -2.0;

  auto activeRigidBodies = this->findActiveLinks(residual);
  std::cout << "active rigid bodies are" << std::endl;
  for (auto& rb: activeRigidBodies){
    std::cout << rb->linkname << std::endl;
  }


  // SVD Test
  MatrixXf A = MatrixXf::Random(3, 2);
  VectorXf b = VectorXf::Random(3);
  VectorXf test = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

}

// returns the rotation matrix that rotates vector a to vector b
Eigen::Matrix<double, 3, 3> rotateVectorToAlign(Eigen::Vector3d a, Eigen::Vector3d b){
  Matrix<double, 3, 3> R;

  // normalize them to be unit vectors first
  a.normalize();
  b.normalize();
  Vector3d axis = a.cross(b);
  Vector4d quat = Vector4d::Zero();

  if (axis.norm() > 1e-4){
    axis.normalize();
    double angle = acos(a.dot(b));
    quat(0) = cos(angle/2.0);
    quat.tail<3>() = sin(angle/2)*axis;
  }
  else if (a.dot(b) > 0){
//    std::cout << "vectors are too closely aligned, returning identity rotation matrix" << std::endl;
    quat(0) = 1; // quat is [1,0,0,0]
  }
  else if (a.dot(b) < 0){
    Vector3d rand = Vector3d::Random();
    axis = a.cross(rand);
    axis.normalize();
    quat(0) = 0;
    quat.tail<3>() = axis;
  }

  R = quat2rotmat(quat);
  return R;
};

Matrix<double,6,3> computeForceToWrenchTransform(Vector3d p){
  Matrix<double,6,3> fM;
  fM.block(0,0,3,3) << 0, -p(2), p(1),
                       p(2), 0, -p(0),
                       -p(1), p(0), 0;
  fM.block(3,0,3,3) = MatrixXd::Identity(3,3);

  return fM;
};



