#pragma once
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "generic_plan.h"
#include "drake/Path.h"
#include "../RobotStateDriver.hpp"
#include "drc/foot_contact_estimate_t.hpp"
#include "utils/rate_limiter.h"
#include "bot_core/utime_t.hpp"


class PlanEval {
 public:
  PlanEval(const std::string &urdf_name, const std::string &config_name);
  void Start();
  void Stop();

  inline bool isReceiverRunning() const { return receiver_thread_.joinable(); }
  inline bool isPublisherRunning() const {
    return publisher_thread_.joinable();
  }

 private:
  std::string urdf_name_;
  std::string config_name_;

  lcm::LCM lcm_handle_receiver_;
  lcm::LCM lcm_handle_publisher_;

  // est robot state
  std::mutex state_lock_;
  DrakeRobotState est_robot_state_receiver_;
  DrakeRobotState est_robot_state_publisher_;
  bot_core::robot_state_t est_robot_state_msg_;
  std::shared_ptr<RobotStateDriver> state_driver_;

  // input
  std::mutex plan_lock_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_stop_;
  std::atomic<bool> new_plan_;
  std::atomic<bool> new_robot_state_;
  bool new_robot_state_publisher_loop_;
  bool new_robot_state_receiver_loop_;
  std::shared_ptr<GenericPlan> current_plan_;

  // output
  std::thread publisher_thread_;
  std::atomic<bool> publisher_stop_;

  PlanStatus current_plan_status_;
  RateLimiter plan_status_rate_limiter_;

  void ReceiverLoop();
  void PublisherLoop();

  // handle plans
  void HandleManipPlan(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const drc::robot_plan_t *msg);

  // void HandleDefaultManipPlan(const lcm::ReceiveBuffer *rbuf, const std::string &channel, const bot_core::utime_t *msg);

  void MakeManipPlan(const drc::robot_plan_t *msg);

  void HandleWalkingPlan(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const drc::walking_plan_request_t *msg);

  // handle robot state msg
  void HandleEstRobotStateReceiverLoop(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const bot_core::robot_state_t *msg);

  // handle robot state msg
  void HandleEstRobotStatePublisherLoop(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const bot_core::robot_state_t *msg);

  void HandleEstContactStateReceiverLoop(const lcm::ReceiveBuffer *rbuf,
                             const std::string &channel,
                             const drc::foot_contact_estimate_t *msg);

  void HandleEstContactStatePublisherLoop(const lcm::ReceiveBuffer *rbuf,
                                         const std::string &channel,
                                         const drc::foot_contact_estimate_t *msg);

  void publishPlanStatus(PlanStatus plan_status);
};
