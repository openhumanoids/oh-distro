//
// Created by manuelli on 10/12/15.
//

#include "ContactFilter.hpp"
#include <Eigen/Geometry>
#include <math.h>
#include "gurobiUtil.hpp"
#include "lcmtypes/drc/contact_filter_estimate_t.hpp"
#include <tuple>
#define MU 0.6
#define PUBLISH_CHANNEL "CONTACT_FILTER_POINT_ESTIMATE"


const double pi = 3.14159265358;
using namespace Eigen;

ContactFilter::ContactFilter(void):grbModel(grbEnv),grbModelInitialized(false){
  this->mu = MU;
  this->initializeGurobiModel();
  this->initializeFrictionCone();
  this->publishChannel = PUBLISH_CHANNEL;

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
  this->nq = drake_model.num_positions;
  this->nv = drake_model.num_velocities;
  this->W = MatrixXd::Identity(this->nv, this->nv);
  this->determinantW = this->W.determinant();

  for(int i = 0; i < this->nv; i++){
    this->velocity_names.push_back(this->drake_model.getVelocityName(i));
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

  drc::contact_filter_estimate_t msg;
  msg.utime = static_cast<int64_t> (measurementUpdate.t*1e6);
  msg.body_id = (int16_t) measurementUpdate.contactFilterPoint.body_id;
  msg.body_name = this->drake_model.getBodyOrFrameName(measurementUpdate.contactFilterPoint.body_id);
  msg.name = measurementUpdate.contactFilterPoint.name;
  msg.velocity_names = this->velocity_names;
  msg.likelihood = (float) measurementUpdate.likelihood;
  msg.logLikelihood = (float) measurementUpdate.exponentVal;
  msg.contact_force_magnitude = (float) measurementUpdate.forceInBodyFrame.norm();

  for(int i=0; i < 3; i++){
    msg.contact_position[i] = (float) measurementUpdate.contactFilterPoint.contactPoint(i);
    msg.contact_normal[i] = (float) measurementUpdate.contactFilterPoint.contactNormal(i);
    msg.contact_force[i] = (float) measurementUpdate.forceInBodyFrame(i);
  }

  msg.num_velocities = (int16_t) this->nv;
  msg.implied_residual.resize(this->nv);
  for(int j=0; j<this->nv; j++){
    msg.implied_residual[j] = (float) measurementUpdate.estResidual(j);
  }

  std::string channel = this->publishChannel;

  if (publishOnUniqueChannel){
    if (measurementUpdate.contactFilterPoint.name.length() > 0){
      channel = channel + "_" + measurementUpdate.contactFilterPoint.name;
    }
    else{
      channel = channel + "_NO_NAME";
    }
  }

  this->lcm.publish(channel, &msg);
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
  fM.block(0,0,3,3) << 0, -p(0), p(1),
                       p(0), 0, -p(0),
                       -p(1), p(0), 0;
  fM.block(3,0,3,3) = MatrixXd::Identity(3,3);

  return fM;
};
