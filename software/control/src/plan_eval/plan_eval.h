#pragma once
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "bot_core/robot_state_t.hpp"
#include "drc/robot_plan_t.hpp"
#include "drake/lcmt_qp_controller_input.hpp"

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/robotInterfaces/BodyMotionData.h"

#include "drake/systems/controllers/QPCommon.h"

#include "drake/Path.h"

#include "zmp_planner.h"

struct RigidBodySupportStateElement {
  int body;
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  Eigen::Vector4d support_surface;
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;

class GenericPlan {
 public:
  GenericPlan()
      : robot_(Drake::getDrakePath() + std::string(
                                           "/../../models/val_description/urdf/"
                                           "valkyrie_sim_drake.urdf"),
               DrakeJoint::ROLLPITCHYAW) {
    // kinematics related init
    LoadConfigurationFromYAML(Drake::getDrakePath() + "/../../config/val_mit/control_config_hardware.yaml");
  }
  virtual ~GenericPlan() { ; }
  virtual void LoadConfigurationFromYAML(const std::string &name);
  inline double t0() const { return interp_t0_; }

  virtual void HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &channel,
                                        const drc::robot_plan_t *msg) = 0;
  virtual drake::lcmt_qp_controller_input MakeQPInput(double cur_time) = 0;


 protected:
  // is set the first time in the publishing / interp loop
  double interp_t0_ = -1;

  // robot for doing kinematics
  RigidBodyTree robot_; 
  RobotPropertyCache rpc_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  // splines for joints
  PiecewisePolynomial<double> q_trajs_;

  // spline for zmp
  PiecewisePolynomial<double> zmp_traj_;
  ZMPPlanner zmp_planner_;

  // list of tracked bodies
  std::vector<BodyMotionData> body_motions_;

  // list of support
  RigidBodySupportState support_state_;
};

class ManipPlan : public GenericPlan {
 public:
  void HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                const std::string &channel,
                                const drc::robot_plan_t *msg);
  drake::lcmt_qp_controller_input MakeQPInput(double cur_time);
};

class PlanEval {
 public:
  PlanEval();
  
  void Start();
  void Stop();

  inline bool isReceiverRunning() const { return receiver_thread_.joinable(); }
  inline bool isPublisherRunning() const {
    return publisher_thread_.joinable();
  }

 private:
  // only working on manip now, need to think about how to switch between manip
  // and walking
  ManipPlan plan_;

  lcm::LCM lcm_handle_;
  double time_;  ///< from est robot state

  // input
  std::mutex plan_lock_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_stop_;
  std::atomic<bool> new_plan_;
  std::shared_ptr<GenericPlan> current_plan_;  

  // output
  std::thread publisher_thread_;
  std::atomic<bool> publisher_stop_;

  void ReceiverLoop();
  void PublisherLoop();

  void HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &channel,
                                        const drc::robot_plan_t *msg);

  // handle robot state msg
  void HandleEstRobotState(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const bot_core::robot_state_t *msg);
};
