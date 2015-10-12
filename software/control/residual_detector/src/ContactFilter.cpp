//
// Created by manuelli on 10/12/15.
//

#include "ContactFilter.hpp"
#include <Eigen/Geometry>
#include <math.h>

using namespace Eigen;

ContactFilter::ContactFilter(void){
  this->mu
}
ContactFilter::~ContactFilter(void){
}

ContactFilter::ContactFilter(std::string URDFString){
  this->addRobotFromURDFString(URDFString);

}

void ContactFilter::addRobotFromURDFString(std::string URDFString){
  this->drake_model.addRobotFromURDFString(URDFString);
  this->drake_model.compile();
  this->nq = drake_model.num_positions;
  this->nv = drake_model.num_velocities;
}

double ContactFilter::computeLikelihood(const VectorXd &q, const VectorXd &v,
                                        Vector3d contactPosition, Vector3d contactNormal, int body_id,
                                        bool publish) {
  this->drake_model.doKinematics(q, v, false, false);
  std::vector<int> v_indices;
  MatrixXd linkJacobian = this->drake_model.geometricJacobian(0, body_id, body_id, 0, false, *v_indices)

  return 0;
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
    std::cout << "vectors are too closely aligned, returning identity rotation matrix" << std::endl;
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


