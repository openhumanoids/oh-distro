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

#include "../RobotStateDriver.hpp"

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
    LoadConfigurationFromYAML(config_name);
  }
  virtual ~GenericPlan() { ; }
  virtual void LoadConfigurationFromYAML(const std::string &name);
  inline double t0() const { return interp_t0_; }

  virtual void HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &channel,
                                        const drc::robot_plan_t *msg,
                                        const Eigen::VectorXd est_q, 
                                        const Eigen::VectorXd est_qd) = 0;
  virtual drake::lcmt_qp_controller_input MakeQPInput(double cur_time) = 0;


 protected:
  // some params
  double default_mu_;
  double default_zmp_height_;
  std::map<std::string, Eigen::Matrix3Xd> contact_offsets;

  // is set the first time in the publishing / interp loop
  double interp_t0_ = -1;

  // robot for doing kinematics
  RigidBodyTree robot_; 
  RobotPropertyCache rpc_;
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
};

class ManipPlan : public GenericPlan {
 public:
  ManipPlan(const std::string &urdf_name, const std::string &config_name) : GenericPlan(urdf_name, config_name) {
    ;
  }

  void HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                const std::string &channel,
                                const drc::robot_plan_t *msg,
                                const Eigen::VectorXd est_q, 
                                const Eigen::VectorXd est_qd);
  drake::lcmt_qp_controller_input MakeQPInput(double cur_time);
};


// only working on manip now, need to think about how to switch between manip
// and walking
class PlanEval {
 public:
  PlanEval(const std::string &urdf_name, const std::string &config_name) {
    // names
    urdf_name_ = urdf_name;
    config_name_ = config_name;

    // state decoder stuff
    RigidBodyTree r(urdf_name);
    est_robot_state_.q.resize(r.num_positions);
    est_robot_state_.qd.resize(r.num_velocities);
    int num_states = r.num_positions + r.num_velocities;
    std::vector<std::string> state_coordinate_names(num_states);
    for (int i = 0; i < num_states; i++) {
      state_coordinate_names[i] = r.getStateName(i);
    }
    state_driver_.reset(new RobotStateDriver(state_coordinate_names));

    // threading + lcm
    receiver_stop_ = false;
    publisher_stop_ = false;
    new_plan_ = false;

    if (!lcm_handle_.good()) {
      std::cerr << "ERROR: lcm is not good()" << std::endl;
      exit(-1);
    }

    lcm::Subscription *sub;
    sub = lcm_handle_.subscribe("COMMITTED_ROBOT_PLAN",
        &PlanEval::HandleCommittedRobotPlan, this);
    sub->setQueueCapacity(1);

    sub = lcm_handle_.subscribe("EST_ROBOT_STATE", &PlanEval::HandleEstRobotState,
        this);
    sub->setQueueCapacity(1); 
  }
  
  void Start();
  void Stop();

  inline bool isReceiverRunning() const { return receiver_thread_.joinable(); }
  inline bool isPublisherRunning() const {
    return publisher_thread_.joinable();
  }

 private:
  std::string urdf_name_;
  std::string config_name_;

  lcm::LCM lcm_handle_;
  
  // est robot state
  std::mutex state_lock_;
  DrakeRobotState est_robot_state_;
  std::shared_ptr<RobotStateDriver> state_driver_;

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
