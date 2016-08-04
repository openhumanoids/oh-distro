#pragma once
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "bot_core/robot_state_t.hpp"

#include "generic_plan.h"
#include "drake/Path.h"
#include "../RobotStateDriver.hpp"

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
        &PlanEval::HandleManipPlan, this);
    sub->setQueueCapacity(1);
    
    sub = lcm_handle_.subscribe("WALKING_CONTROLLER_PLAN_REQUEST",
        &PlanEval::HandleWalkingPlan, this);
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
  std::atomic<bool> new_robot_state_;
  std::shared_ptr<GenericPlan> current_plan_;

  // output
  std::thread publisher_thread_;
  std::atomic<bool> publisher_stop_;

  void ReceiverLoop();
  void PublisherLoop();

  void HandleManipPlan(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const drc::robot_plan_t *msg);
  
  void HandleWalkingPlan(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const drc::walking_plan_request_t *msg);

  // handle robot state msg
  void HandleEstRobotState(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const bot_core::robot_state_t *msg);
};
