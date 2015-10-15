//
// Created by manuelli on 10/12/15.
//
#include "drake/controlUtil.h"
#include <gurobi_c++.h>
#include <lcm/lcm-cpp.hpp>

struct ContactFilterPoint{
  int body_id;
  std::string name;
  Eigen::Vector3d contactPoint;
  Eigen::Vector3d contactNormal;
};

class ContactFilter{

private:
  RigidBodyManipulator drake_model;
  int nv;
  int nq;
  double mu;
  std::string publishChannel;
  lcm::LCM lcm;
  std::vector<std::string> velocity_names;
  GRBEnv grbEnv;
  GRBModel grbModel;
  std::vector<GRBVar> grbVars;
  bool grbModelInitialized;
  MatrixXd W;
  double determinantW;
  Matrix<double,3,4> FrictionCone;
  void initializeGurobiModel();
  void initializeFrictionCone();
  void publishPointEstimate(double t, int body_id, const Eigen::Vector3d & contactPosition, const Eigen::Vector3d &contactNormal,
               const Eigen::Vector3d &forceInBodyFrame, const Eigen::VectorXd &estResidual, const double& likelihood);

public:

//  ContactFilter(std::string URDFString);
  ContactFilter(void);
  virtual ~ContactFilter(void);

  void addRobotFromURDFString(std::string URDFString);

  // contact position and contact normal are both in body frame
  double computeLikelihood(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q, const Eigen::VectorXd &v, Eigen::Vector3d contactPosition, Eigen::Vector3d contactNormal, int body_id, bool publish=false);

  void testQP();
  void printGurobiModel();
};

Eigen::Matrix<double, 3, 3> rotateVectorToAlign(Eigen::Vector3d a, Eigen::Vector3d b);
Matrix<double,6,3> computeForceToWrenchTransform(Vector3d p);