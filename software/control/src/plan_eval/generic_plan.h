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
#include "drc/foot_contact_estimate_t.hpp"

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;
};

struct RigidBodySupportStateElement {
  int body;
  double total_normal_force_upper_bound;
  double total_normal_force_lower_bound;
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  Eigen::Vector4d support_surface;
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;

enum class PlanExecutionStatus{
  UNKNOWN,
  EXECUTING,
  FINISHED,
};

enum class PlanType{
  UNKNOWN,
  STANDING,
  WALKING,
  BRACING,
  RECOVERING,
};

struct PlanStatus {
  PlanExecutionStatus executionStatus;
  PlanType planType;
};

class GenericPlan {
 public:
  /*
  enum ContactState {
    AIR = 0,
    SSL = 1,
    SSR = 2,
    DSc = 3,
  };
  */
 protected:

  // top level yaml config file node
  YAML::Node config_;

  // some params
  double p_mu_;
  double p_zmp_height_;
  double p_initial_transition_time_;
  double p_transition_trq_alpha_filter_;
  double p_min_Fz_;

  RobotPropertyCache rpc_;
  std::map<int, Eigen::Matrix3Xd> contact_offsets;

  // is set the first time in the publishing / interp loop
  double interp_t0_ = -1;

  // the plan status
  PlanStatus plan_status_;

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

  // list of constrained dof
  std::vector<int> constrained_dofs_;

  RigidBodySupportState MakeDefaultSupportState(const ContactState &cs) const;
  BodyMotionData MakeDefaultBodyMotionData(size_t num_segments) const;

  drake::lcmt_qp_controller_input MakeDefaultQPInput(double real_time, double plan_time, const std::string &param_set_name, bool apply_torque_alpha_filter) const;

  // make lcm messages
  drake::lcmt_support_data EncodeSupportData(const RigidBodySupportStateElement &element) const;
  drake::lcmt_body_motion_data EncodeBodyMotionData(double plan_time, const BodyMotionData &body_motion) const;
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
                                        const DrakeRobotState &est_rs,
                                        const Eigen::VectorXd &last_q_d) = 0;
  virtual drake::lcmt_qp_controller_input MakeQPInput(const DrakeRobotState &est_rs) = 0;

  virtual Eigen::VectorXd GetLatestKeyFrame(double time) = 0;

  PlanStatus getPlanStatus();
};

std::string PrimaryBodyOrFrameName(const std::string &full_body_name);
Eigen::Matrix<double,7,1> Isometry3dToVector7d(const Eigen::Isometry3d &pose);

void KeyframeToState(const bot_core::robot_state_t &keyframe,
                            Eigen::VectorXd &q, Eigen::VectorXd &v);
