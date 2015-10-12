//
// Created by manuelli on 10/12/15.
//
#include "drake/controlUtil.h"
#include "gurobi_c++.h"

class ContactFilter{

private:
  RigidBodyManipulator drake_model;
  int nv;
  int nq;
  int mu;
//  GRBEnv gurobiEnv;

public:

  ContactFilter(std::string URDFString);
  ContactFilter(void);
  virtual ~ContactFilter(void);

  void addRobotFromURDFString(std::string URDFString);

  // contact position and contact normal are both in body frame
  double computeLikelihood(const Eigen::VectorXd &q, const Eigen::VectorXd &v, Eigen::Vector3d contactPosition, Eigen::Vector3d contactNormal, int body_id, bool publish=false);
};

Eigen::Matrix<double, 3, 3> rotateVectorToAlign(Eigen::Vector3d a, Eigen::Vector3d b);
