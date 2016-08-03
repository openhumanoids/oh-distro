#pragma once

#include "generic_plan.h"

class ManipPlan : public GenericPlan {
 public:
  ManipPlan(const std::string &urdf_name, const std::string &config_name) : GenericPlan(urdf_name, config_name) {
    ;
  }

  void HandleCommittedRobotPlan(const drc::robot_plan_t &msg,
                                const Eigen::VectorXd &est_q,
                                const Eigen::VectorXd &est_qd,
                                const Eigen::VectorXd &last_q_d);
  drake::lcmt_qp_controller_input MakeQPInput(double cur_time);
  Eigen::VectorXd GetLatestKeyFrame(double time);
};

