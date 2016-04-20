//
// Created by manuelli on 10/12/15.
//
#include "drake/systems/controllers/controlUtil.h"
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

struct BodyWrenchEstimate{
  double t;
  double exponentVal;
  Eigen::VectorXd wrench;
  Eigen::VectorXd estResidual;
  std::string jointName;
  std::string linkName;
};

typedef std::vector<MeasurementUpdate> MultiContactMeasurementUpdate;


class ContactFilter{

private:
  RigidBodyManipulator drake_model;
  int nv;
  int nq;
  double mu;
  double detectionThreshold;
  Eigen::VectorXd detectionThresholdVec;
  std::vector<ContactFilterPoint> contactPoints;
  std::map<std::string, std::vector<ContactFilterPoint>> contactPointMap;
  std::vector<MeasurementUpdate> measurementUpdateVec;
  std::vector<MultiContactMeasurementUpdate> multiContactMeasurementUpdateVec;
  std::string publishChannel;
  std::string bodyWrenchPublishChannel;
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
  std::set<std::shared_ptr<RigidBody>> findActiveLinks(const Eigen::VectorXd &residual);

  void publishBodyFrameWrench(const std::vector<BodyWrenchEstimate> &wrenchEstimateVec);
  Eigen::MatrixXd geometricJacobianFull(int base_body_or_frame_ind, int end_effector_body_or_frame_ind,
                             int expressed_in_body_or_frame_ind, int gradient_order,
  bool in_terms_of_qdot = false);


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

  MultiContactMeasurementUpdate computeMultiContactLikelihood(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                                                               const Eigen::VectorXd &v,
                                      const std::vector<ContactFilterPoint>& contactFilterPointsVec, bool publish=false);

  void computeMultiContactLikelihoodFull(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                                                               const Eigen::VectorXd &v, std::vector<std::string> activeLinks,
                                                                   bool publishMostLikely=false,
                                                                   bool publishAll=false);

  void computeLikelihoodFull(double t, const Eigen::VectorXd &residual, const Eigen::VectorXd &q,
                             const Eigen::VectorXd &v, bool publishMostLikely=false, bool publishAll=false);

  void publishMostLikelyEstimate();
  void computeActiveLinkForceTorque(double t, const Eigen::VectorXd & residual, const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &v, bool publish=false,
                                    std::vector<std::string> activeLinkNames = std::vector<std::string>());


  void runFindLinkTest();

  void testQP();
  void printGurobiModel();
};

Eigen::Matrix<double, 3, 3> rotateVectorToAlign(Eigen::Vector3d a, Eigen::Vector3d b);
Matrix<double,6,3> computeForceToWrenchTransform(Vector3d p);

