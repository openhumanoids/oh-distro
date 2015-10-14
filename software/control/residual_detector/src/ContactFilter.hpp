//
// Created by manuelli on 10/12/15.
//
#include "drake/controlUtil.h"
#include <gurobi_c++.h>

class ContactFilter{

private:
  RigidBodyManipulator drake_model;
  int nv;
  int nq;
  double mu;
  GRBEnv grbEnv;
  GRBModel grbModel;
  std::vector<GRBVar> grbVars;
  bool grbModelInitialized;
  MatrixXd W;
  Matrix<double,3,4> FrictionCone;
  void initializeGurobiModel();
  void initializeFrictionCone();

public:

//  ContactFilter(std::string URDFString);
  ContactFilter(void);
  virtual ~ContactFilter(void);

  void addRobotFromURDFString(std::string URDFString);

  // contact position and contact normal are both in body frame
  double computeLikelihood(const Eigen::VectorXd &residual, const Eigen::VectorXd &q, const Eigen::VectorXd &v, Eigen::Vector3d contactPosition, Eigen::Vector3d contactNormal, int body_id, bool publish=false);

  void testQP();
  void printGurobiModel();
};

Eigen::Matrix<double, 3, 3> rotateVectorToAlign(Eigen::Vector3d a, Eigen::Vector3d b);
Matrix<double,6,3> forceMomentTransformation(Vector3d p);