#pragma once

#include "generic_plan.h"
#include <list>
#include <utility>

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
  enum WalkingState {
    WEIGHT_TRANSFER,
    SWING
  };

  WalkingState cur_state_;

  std::list<drc::footstep_t> footstep_plan_;
  std::list<std::pair<ContactState, double>> contact_state_;
  PiecewisePolynomial<double> weight_distribution_;

  double contact_switch_time_ = -INFINITY;
  Eigen::VectorXd init_q_;

  double p_pre_weight_transfer_scale_;
  double p_lower_z_vel_;
  double p_ss_duration_;
  double p_ds_duration_;

  double p_swing_foot_xy_weight_mulitplier_;
  double p_pelvis_z_weight_mulitplier_;
  double p_left_foot_zmp_in_shift_;
  double p_right_foot_zmp_in_shift_;

  void LoadConfigurationFromYAML(const std::string &name);
  void GenerateTrajs(const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd, const ContactState &cur_contact_state);

  /*
  Side GetSwingFoot(ContactState contact_state) const {
    if (contact_state == ContactState::SSL)
      return Side::RIGHT;
    else if (contact_state == ContactState::SSR)
      return Side::LEFT;
    else
      throw std::runtime_error("no swing foot in double support or in air.");
  }

  Side GetStanceFoot(ContactState contact_state) const {
    if (contact_state == ContactState::SSL)
      return Side::LEFT;
    else if (contact_state == ContactState::SSR)
      return Side::RIGHT;
    else
      throw std::runtime_error("no stance foot in double support or in air.");
  }

  inline bool is_single_support(ContactState contact_state) const {
    return (contact_state == ContactState::SSL || contact_state == ContactState::SSR);
  }

  inline bool is_double_support(ContactState contact_state) const {
    return (contact_state == ContactState::DSc);
  }
  */

  inline const ContactState &cur_planned_contact_state() const {
    if (contact_state_.empty())
      throw std::runtime_error("empty planned contact_state");
    return contact_state_.front().first;
  }

  inline double cur_planned_contact_swith_time() const {
    if (contact_state_.empty())
      throw std::runtime_error("empty planned contact_state");
    return contact_state_.front().second;
  }

  void SwitchContactState(double cur_time);

  PiecewisePolynomial<double> GenerateSwingTraj(const Eigen::Matrix<double, 7, 1> &foot0, const Eigen::Matrix<double, 7, 1> &foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_transfer_dur, double swing_down_dur) const;

  static double get_weight_distribution(const ContactState &cs);
};

