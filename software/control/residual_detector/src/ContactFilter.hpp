//
// Created by manuelli on 10/12/15.
//
#include "drake/controlUtil.h"
#include <Eigen/Geometry>
#include <gurobi_c++.h>
#include <lcm/lcm-cpp.hpp>

struct ContactFilterPoint{
  int body_id;
  std::string body_name;
  std::string name;
  Eigen::Vector3d contactPoint;
  Eigen::Vector3d contactNormal;
};

struct MeasurementUpdate{
  double t;
  double likelihood;
  double exponentVal;
  Eigen::VectorXd forceInBodyFrame;
  Eigen::VectorXd estResidual;
  const ContactFilterPoint &contactFilterPoint;
};

class ContactFilter{

private:
  RigidBodyManipulator drake_model;
  int nv;
  int nq;
  double mu;
  double detectionThreshold;
  Eigen::VectorXd detectionThresholdVec;
  std::vector<ContactFilterPoint> contactPoints;
  std::vector<MeasurementUpdate> measurementUpdateVec;
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
  std::map<std::shared_ptr<RigidBody>, std::set<std::shared_ptr<RigidBody>> > rigidBodyChildrenMap;
  std::set<std::shared_ptr<RigidBody>> rigidBodyLeafNodes;
//  std::set<std::shared_ptr<RigidBody>> activeRigidBodies;



  void initializeGurobiModel();
  void initializeFrictionCone();
  void publishPointEstimate(const MeasurementUpdate & measurementUpdate, bool publishOnUniqueChannel=true);
  void initializeDrakeModelDetails();
  std::set<std::shared_ptr<RigidBody>> findActiveLinks(Eigen::VectorXd &residual);


public:

//  ContactFilter(std::string URDFString);
  ContactFilter(void);
  virtual ~ContactFilter(void);

  void addRobotFromURDFString(std::string URDFString);
  void addRobotFromURDF(std::string urdfFilename="");
  void addContactPoints(std::vector<ContactFilterPoint> contactPointsToAdd);

  // contact position and contact normal are both in body frame
  MeasurementUpdate computeLikelihood(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q, const Eigen::VectorXd &v,
                           const ContactFilterPoint& contactFilterPoint, bool publish=false);

  void computeLikelihoodFull(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                             const Eigen::VectorXd &v, bool publishMostLikely=false, bool publishAll=false);

  void publishMostLikelyEstimate();


  void runFindLinkTest();

  void testQP();
  void printGurobiModel();
};

Eigen::Matrix<double, 3, 3> rotateVectorToAlign(Eigen::Vector3d a, Eigen::Vector3d b);
Matrix<double,6,3> computeForceToWrenchTransform(Vector3d p);

