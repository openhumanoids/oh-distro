#pragma once

#include "drake/lcmt_qp_controller_input.hpp"
#include "bot_core/robot_state_t.hpp"

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/robotInterfaces/BodyMotionData.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/yaml/yamlUtil.h"
 
#include "zmp_planner.h"

#include "drc/robot_plan_t.hpp"
#include "drc/walking_plan_request_t.hpp"

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;
};

struct RigidBodySupportStateElement {
  int body;
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  Eigen::Vector4d support_surface;
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;
 
class GenericPlan {
 public:
  GenericPlan(const std::string &urdf_name, const std::string &config_name)
      : robot_(urdf_name, DrakeJoint::ROLLPITCHYAW) {
    p_mu_ = 1.;
    p_zmp_height_ = 0.8;
    p_initial_transition_time_ = 0.5;
    LoadConfigurationFromYAML(config_name);
  }
  virtual ~GenericPlan() { ; }
  virtual void LoadConfigurationFromYAML(const std::string &name);
  inline double t0() const { return interp_t0_; }

  virtual void HandleCommittedRobotPlan(const void *msg,
                                        const Eigen::VectorXd &est_q,
                                        const Eigen::VectorXd &est_qd,
                                        const Eigen::VectorXd &last_q_d) = 0;
  virtual drake::lcmt_qp_controller_input MakeQPInput(double cur_time) = 0;

  virtual Eigen::VectorXd GetLatestKeyFrame(double time) = 0;

 protected:
  enum ContactState {
    SSL = 0,
    SSR = 1,
    DSc = 2
  }; 

  // top level yaml config file node
  YAML::Node config_;

  // some params
  double p_mu_;
  double p_zmp_height_;
  double p_initial_transition_time_;
  RobotPropertyCache rpc_;
  std::map<int, Eigen::Matrix3Xd> contact_offsets;

  // is set the first time in the publishing / interp loop
  double interp_t0_ = -1;

  // robot for doing kinematics
  RigidBodyTree robot_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  // estimated robot state
  Eigen::VectorXd est_q_;
  Eigen::VectorXd est_v_;

  // splines for joints
  PiecewisePolynomial<double> q_trajs_;

  // spline for zmp
  PiecewisePolynomial<double> zmp_traj_;
  ZMPPlanner zmp_planner_;

  // list of tracked bodies
  std::vector<BodyMotionData> body_motions_;

  // list of support
  RigidBodySupportState support_state_;
  
  
  RigidBodySupportState MakeDefaultSupportState(ContactState cs) const;
  BodyMotionData MakeDefaultBodyMotionData(size_t num_segments) const;
 
  // make lcm messages
  drake::lcmt_support_data EncodeSupportData(const RigidBodySupportStateElement &element) const;
  drake::lcmt_body_motion_data EncodeBodyMotionData(double plan_time, const BodyMotionData &body_motion) const;
};

std::string PrimaryBodyOrFrameName(const std::string &full_body_name);
Eigen::Matrix<double,7,1> Isometry3dToVector7d(const Eigen::Isometry3d &pose);

void KeyframeToState(const bot_core::robot_state_t &keyframe,
                            Eigen::VectorXd &q, Eigen::VectorXd &v);
Eigen::Vector6d getTaskSpaceVel(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    int body_or_frame_id,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
Eigen::Vector6d getTaskSpaceJacobianDotTimesV(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    int body_or_frame_id,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
Eigen::MatrixXd getTaskSpaceJacobian(
    const RigidBodyTree &r, const KinematicsCache<double> &cache, int body,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
 
