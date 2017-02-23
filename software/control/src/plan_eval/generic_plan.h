#pragma once

#include "drake/lcmt_qp_controller_input.hpp"
#include "bot_core/robot_state_t.hpp"

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/robotInterfaces/BodyMotionData.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/yaml/yamlUtil.h"
#include "plan_eval_common.h"
#include "zmp_planner.h"

#include "drc/robot_plan_t.hpp"
#include "drc/walking_plan_request_t.hpp"
#include "drc/foot_contact_estimate_t.hpp"
#include "drc/plan_eval_debug_t.hpp"

// see plan_eval_common.h for type definitions
namespace plan_eval {

class GenericPlan {
protected:

  // top level yaml config file node
  YAML::Node config_;
  GenericPlanConfig generic_plan_config_;
  RobotPropertyCache rpc_;

  // all the state of GenericPlan is contained here
  GenericPlanState generic_plan_state_;

  // the plan status
  PlanStatus plan_status_;

  //Debugging data
  DebugData debug_data_;

  // robot for doing kinematics
  // should make this a shared ptr, not an object.
  // otherwise you can't default initialize it
  RigidBodyTree robot_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  // estimated robot state
  Eigen::VectorXd est_q_;
  Eigen::VectorXd est_v_;

  // splines for joints
  PiecewisePolynomial<double> q_trajs_;

  // spline for zmp
  ZMPPlanner zmp_planner_;

  // list of constrained dof
  std::vector<int> constrained_dofs_;

  RigidBodySupportState MakeDefaultSupportState(const ContactState &cs) const;

  BodyMotionData MakeDefaultBodyMotionData(size_t num_segments) const;

  drake::lcmt_qp_controller_input
  MakeDefaultQPInput(const double& real_time, const double& plan_time, const std::string &param_set_name,
                     bool apply_torque_alpha_filter) const;

  // make lcm messages
  drake::lcmt_support_data EncodeSupportData(const RigidBodySupportStateElement &element) const;

  drake::lcmt_body_motion_data EncodeBodyMotionData(const double& plan_start_time, const double& plan_time, const BodyMotionData &body_motion) const;

  // stores some default debug information in debug_data_ field.
  // It is recommenede to call this method from inside the plan's MakeQPInput method
  // Child classes can also overload this message to provide specific plan information
  void RecordDefaultDebugData(double &plan_time);


public:
  GenericPlan(const std::string &urdf_name, const std::string &config_name)
      : robot_(urdf_name, DrakeJoint::ROLLPITCHYAW) {
    LoadConfigurationFromYAML(config_name);
  }

  virtual ~GenericPlan() { ; }

  virtual void LoadConfigurationFromYAML(const std::string &name);

  inline double t0() const { return generic_plan_state_.plan_start_time; }

  virtual void HandleCommittedRobotPlan(const void *msg,
                                        const DrakeRobotState &est_rs,
                                        const Eigen::VectorXd &last_q_d) = 0;

  virtual drake::lcmt_qp_controller_input MakeQPInput(const DrakeRobotState &est_rs) = 0;

  virtual Eigen::VectorXd GetLatestKeyFrame(double time) = 0;

  void SimpleTest();

  PlanStatus getPlanStatus();

  // converts the DebugDataStruct into a message
  drc::plan_eval_debug_t EncodeDebugData(double &real_time);
};

std::string PrimaryBodyOrFrameName(const std::string &full_body_name);

Eigen::Matrix<double, 7, 1> Isometry3dToVector7d(const Eigen::Isometry3d &pose);

void KeyframeToState(const bot_core::robot_state_t &keyframe,
                     Eigen::VectorXd &q, Eigen::VectorXd &v);
}// plan_eval