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

#include "drake/Path.h"

struct RigidBodySupportStateElement {
  int body;
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  Eigen::Vector4d support_surface;
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;

class GenericPlan {
 public:
  GenericPlan() : robot_(Drake::getDrakePath() + std::string("/../../models/val_description/urdf/valkyrie_sim_drake.urdf"), DrakeJoint::ROLLPITCHYAW) {
    // kinematics related init
    q_.resize(robot_.num_positions);
    v_.resize(robot_.num_velocities);
    has_plan_ = false;
  }
  virtual ~GenericPlan() { ; }

  virtual void HandleCommittedRobotPlan(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_plan_t* msg) =0;
  virtual drake::lcmt_qp_controller_input MakeQPInput(double cur_time) =0;

  inline bool has_plan() const { return has_plan_; }
  inline double t0() const { return t0_; }

 protected:
  std::atomic<bool> has_plan_;
  std::atomic<double> t0_;

  // robot for doing kinematics
  RigidBodyTree robot_;
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;

  // splines for joints
  PiecewisePolynomial<double> q_trajs_;

  // spline for zmp
  PiecewisePolynomial<double> zmp_traj_;
  Eigen::Matrix<double,4,4> A_;
  Eigen::Matrix<double,4,2> B_;
  Eigen::Matrix<double,2,4> C_;
  Eigen::Matrix<double,2,2> D_;
  Eigen::Matrix<double,2,2> D_control_;
  Eigen::Matrix<double,2,2> Qy_, R_;
  Eigen::Matrix<double,4,4> S_;
  Eigen::Matrix<double,2,4> K_;
  Eigen::Vector4d s1_;
  Eigen::Vector4d s1_dot_;
  Eigen::Vector2d u0_;

  // list of tracked bodies
  std::vector<BodyMotionData> body_motions_;

  // list of support
  RigidBodySupportState support_state_;

  // setup lipm matrices
  void SetupLIPM(double height) {
    A_.setZero();
    A_.block<2, 2>(0, 2).setIdentity();
    B_.setZero();
    B_.block<2, 2>(2, 0).setIdentity();
    C_.setZero();
    C_.block<2, 2>(0, 0).setIdentity();
    D_ = -height / 9.81 * Eigen::Matrix2d::Identity();
    // TODO: take params
    D_control_ = D_;

    // TODO: take params
    Qy_ = Eigen::Matrix2d::Identity();
    R_.setZero();

    Eigen::Matrix<double,4,4> Q1 = C_.transpose() * Qy_ * C_;
    Eigen::Matrix<double,2,2> R1 = R_ + D_.transpose() * Qy_ * D_;
    Eigen::Matrix<double,4,2> N = C_.transpose() * Qy_ * D_;

    lqr(A_, B_, Q1, R1, N, K_, S_);

    s1_ = Eigen::Vector4d::Zero();
    s1_dot_.setZero();

    u0_.setZero();
  }
};

class ManipPlan : public GenericPlan {
 public:
  void HandleCommittedRobotPlan(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_plan_t* msg);
  drake::lcmt_qp_controller_input MakeQPInput(double cur_time);
};





class PlanEval {
 public:
  PlanEval() {
    receiver_stop_ = false;
    publisher_stop_ = false;

    Init();
  }

  void Start();
  void Stop();

  inline bool isReceiverRunning() const { return receiver_thread_.joinable(); }
  inline bool isPublisherRunning() const { return publisher_thread_.joinable(); }

 private:
  // only working on manip now, need to think about how to switch between manip and walking
  ManipPlan plan_;

  lcm::LCM lcm_handle_;
  double time_; ///< from est robot state

  // input
  std::mutex plan_lock_;
  std::thread receiver_thread_;
  std::atomic<bool> receiver_stop_;

  // output
  std::thread publisher_thread_;
  std::atomic<bool> publisher_stop_;

  void Init();
  void ReceiverLoop();
  void PublisherLoop();

  // handle robot state msg
  void HandleEstRobotState(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg);
};
