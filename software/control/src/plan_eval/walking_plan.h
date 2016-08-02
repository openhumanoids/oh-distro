#pragma once

#include "generic_plan.h"
#include <list>

class WalkingPlan : public GenericPlan {
 public:
  WalkingPlan(const std::string &urdf_name, const std::string &config_name) : GenericPlan(urdf_name, config_name) {
    ;
  }

  void HandleCommittedRobotPlan(const drc::robot_plan_t &msg,
                                const Eigen::VectorXd &est_q,
                                const Eigen::VectorXd &est_qd,
                                const Eigen::VectorXd &last_q_d,
                                double initial_transition_time);
  drake::lcmt_qp_controller_input MakeQPInput(double cur_time);

  Eigen::VectorXd GetLatestKeyFrame(double time) { return Eigen::VectorXd::Zero(robot_.num_positions); }

 private:
  std::list<ContactState> contact_state_;
  std::list<double> contact_switching_time_;
};

