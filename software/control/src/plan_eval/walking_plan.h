#pragma once

#include "generic_plan.h"
#include <list>

class WalkingPlan : public GenericPlan {
 public:
  WalkingPlan(const std::string &urdf_name, const std::string &config_name) : GenericPlan(urdf_name, config_name) {
    LoadConfigurationFromYAML(config_name);
  }

  void HandleCommittedRobotPlan(const void *plan_msg,
                                const DrakeRobotState &rs,
                                const Eigen::VectorXd &last_q_d);
  drake::lcmt_qp_controller_input MakeQPInput(const DrakeRobotState &rs);

  Eigen::VectorXd GetLatestKeyFrame(double time) { return Eigen::VectorXd::Zero(robot_.num_positions); }

 private:
  PiecewisePolynomial<double> weight_distribution_;

  std::list<drc::footstep_t> footstep_plan_;

  std::list<ContactState> contact_state_;
  std::list<double> contact_switching_time_;
  double contact_switch_time_ = -INFINITY;

  double p_ss_duration_;
  double p_ds_duration_;

  void LoadConfigurationFromYAML(const std::string &name);
  void GenerateTrajs(const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd);
  void SetupContactStates();

  PiecewisePolynomial<double> GenerateSwingTraj(const Eigen::Matrix<double, 7, 1> &foot0, const Eigen::Matrix<double, 7, 1> &foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_down_dur) const;
  inline static double get_weight_distribution(ContactState cs) {
    switch (cs) {
      case DSc:
        return 0.5;
      case SSL:
        return 1;
      case SSR:
        return 0;
    }
    return 0.5;
  }
};

