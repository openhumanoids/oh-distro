#pragma once
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drc/robot_plan_t.hpp" 
#include "drake/lcmt_qp_controller_input.hpp"

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/robotInterfaces/BodyMotionData.h"

class PlanEval {
 public:
  PlanEval() : robot_(std::string("/home/sfeng/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim_drake.urdf"), DrakeJoint::ROLLPITCHYAW) {
    has_plan_ = false;
    receiver_stop_ = false;
    publisher_stop_ = false;

    Init();

    // kinematics related init
    q_.resize(robot_.num_positions);
    v_.resize(robot_.num_velocities);
  }
  
  void Start();
  void Stop();

  inline bool isReceiverRunning() const { return receiver_thread_.joinable(); }
  inline bool isPublisherRunning() const { return publisher_thread_.joinable(); }

 private:
  lcm::LCM lcm_handle_;

  // input
  std::mutex plan_lock_;
  drc::robot_plan_t plan_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_stop_;

  // output
  std::thread publisher_thread_;
  std::atomic<bool> publisher_stop_;

  std::atomic<bool> has_plan_;

  // robot for doing kinematics
  RigidBodyTree robot_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  // splines for joints
  PiecewisePolynomial<double> q_trajs_;
  std::vector<BodyMotionData> body_motions_;

  void Init();
  void ReceiverLoop();
  void PublisherLoop();
  
  void HandleCommittedRobotPlan(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_plan_t* msg);

  void GenerateQPInputForManip(const drc::robot_plan_t &plan);


  drake::lcmt_qp_controller_input MakeManipQPInput(double cur_time);

};
