//
// Created by manuelli on 10/12/15.
//

#include "ContactFilter.hpp"
#include <Eigen/Geometry>
#include <math.h>
#include "gurobiUtil.hpp"
#define MU 1.0



using namespace Eigen;

ContactFilter::ContactFilter(void):grbModel(grbEnv),grbModelInitialized(false){
  this->mu = MU;
  this->initializeGurobiModel();
  this->initializeFrictionCone();
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
}

void ContactFilter::initializeFrictionCone() {
  this->FrictionCone << this->mu, -this->mu, 0, 0,
                        0, 0, this->mu, -this->mu,
                        1,1,1,1;

}

double ContactFilter::computeLikelihood(const VectorXd &residual, const VectorXd &q, const VectorXd &v,
                                        Vector3d contactPosition, Vector3d contactNormal, int body_id,
                                        bool publish) {
  this->drake_model.doKinematics(q, v, false, false);
//  Vector3d points << 0,0,0;
//  MatrixXd R_world_to_body = this->drake_model.forwardKin(points, )
  std::vector<int> v_indices;
  MatrixXd linkJacobian = this->drake_model.geometricJacobian<double>(0, body_id, body_id, 0, false, &v_indices).value();
  // will only be specifying forces, not full wrenches so only need the linear velocity component of the jacobian
//  MatrixXd &J_force = linkJacobian.block(3,0,3,linkJacobian.cols());

  // want to formulate QP with objective (r - J_force^T(\sum_i alpha_i F_c,i))^T W (r - J_force^T(\sum_i alpha_i F_c,i))

  //first need to rotate the friction cone the link frame
  Vector3d z_vector;
  z_vector << 0,0,1;
  Matrix<double,3,3> rotationMatrix = rotateVectorToAlign(z_vector, contactNormal);
  Matrix<double, 3,4> rotatedFrictionCone = rotationMatrix*this->FrictionCone;
  Matrix<double, 6, 3> forceMoment = forceMomentTransformation(contactPosition);
  // still need to do force moment transformation from contactPosition to link frame


  // want to write objective in the form (r - H alpha)^T W (r - H alpha)
  MatrixXd H = linkJacobian.transpose()*forceMoment*rotatedFrictionCone;

  // now construct matrices needed to pass model to gurobi, write objective in the form x^T Q x + f x
  // Q = H^T W H, f=r^T W H
  MatrixXd Q = H.transpose()*this->W*H;
  VectorXd f = residual*this->W*H;

  //clear the objective value of the current model
  GRBLinExpr linExpr = 0.0;
  this->grbModel.setObjective(linExpr);
  GRBVar* varsArray = this->grbModel.getVars(); // this is an array
  //make std::vector out of it
  std::vector<GRBVar*> vars;

  for (GRBVar* v = varsArray; ++v; v != nullptr){
    vars.push_back(v);
  }
  gurobiAddObjective(this->grbModel, Q, f, vars);

  delete [] varsArray;

  return 0;
}


void ContactFilter::initializeGurobiModel() {
  if (this->grbModelInitialized){
    return;
  }

  this->grbModelInitialized = true;
  std::string var_name;

  for (int i=0; i < 4; i++){
    var_name = "alpha_" + std::to_string(i);
    this->grbVars.push_back(this->grbModel.addVar(0.0, NULL, 0.0, GRB_CONTINUOUS, var_name));
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

Matrix<double,6,3> forceMomentTransformation(Vector3d p){
  Matrix<double,6,3> fM;
  fM.block(0,0,3,1) = MatrixXd::Identity(3,3);
  fM.block(3,0,3,1) << 0, -p(0), p(1),
                       p(0), 0, -p(0),
                       -p(1), p(0), 0;

  return fM;
};
