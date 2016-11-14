#pragma once

#include "generic_plan.h"
#include <list>
#include <utility>
#include <lcm/lcm-cpp.hpp>


// should really get all the walking specific params into one place rather than littered around in different places
struct WalkingParams{
  bool constrain_back_bkx;
  bool constrain_back_bky;
  bool constrain_back_bkz;
};

class WalkingPlan : public GenericPlan {
 public:
  WalkingPlan(const std::string &urdf_name, const std::string &config_name) : GenericPlan(urdf_name, config_name) {
    LoadConfigurationFromYAML(config_name);
    plan_status_.planType = PlanType::WALKING;
    step_count_ = 0;

    // check the lcm handle initialization was good
    if (!lcm_handle_.good()) {
      throw std::runtime_error("lcm is not good()");
    }
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

  lcm::LCM lcm_handle_;

  WalkingState cur_state_;
  WalkingParams walking_params_;

  std::list<drc::footstep_t> footstep_plan_;
  std::list<std::pair<ContactState, double>> contact_state_;
  PiecewisePolynomial<double> weight_distribution_;

  double contact_switch_time_ = -INFINITY;
  Eigen::VectorXd init_q_;

  // these are all parameters, hence they are prefixed with a p
  double p_pelvis_height_;

  double p_extend_swing_foot_down_z_vel_;
  double p_swing_foot_touchdown_z_vel_;
  double p_swing_foot_touchdown_z_offset_;
  double p_ss_duration_;
  double p_ds_duration_;

  double p_swing_foot_xy_weight_mulitplier_;
  double p_swing_foot_z_weight_mulitplier_;
  double p_pelvis_z_weight_mulitplier_;
  double p_left_foot_zmp_y_shift_;
  double p_right_foot_zmp_y_shift_;

  bool have_tared_swing_leg_ft_ = false;

  int step_count_;

  void LoadConfigurationFromYAML(const std::string &name);
  void GenerateTrajs(double plan_time, const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd, const ContactState &cur_contact_state);

  inline BodyMotionData& get_pelvis_body_motion_data() { return body_motions_[0]; }
  inline BodyMotionData& get_torso_body_motion_data() { return body_motions_[1]; }
  inline BodyMotionData& get_stance_foot_body_motion_data() { return body_motions_[2]; }
  inline BodyMotionData& get_swing_foot_body_motion_data() { return body_motions_[3]; }

  inline Eigen::Vector7d bot_core_pose2pose(const bot_core::position_3d_t &p) const {
    Eigen::Vector7d pose;
    pose[0] = p.translation.x;
    pose[1] = p.translation.y;
    pose[2] = p.translation.z;
    pose[3] = p.rotation.w;
    pose[4] = p.rotation.x;
    pose[5] = p.rotation.y;
    pose[6] = p.rotation.z;
    pose.tail(4).normalize();
    return pose;
  }

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

  static Eigen::Isometry3d FootstepMsgToPose(drc::footstep_t msg);
  Eigen::Vector2d Footstep2DesiredZMP(Side side, const Eigen::Isometry3d &step) const;
  PiecewisePolynomial<double> PlanZMPTraj(const std::vector<Eigen::Vector2d> &zmp_d, int num_of_zmp_knots, const Eigen::Vector2d &zmp_d0, const Eigen::Vector2d &zmpd_d0, double time_before_weight_shift) const;
  void SwitchContactState(double cur_time);
  void TareSwingLegForceTorque();
  PiecewisePolynomial<double> GenerateSwingTraj(const Eigen::Matrix<double, 7, 1> &foot0, const Eigen::Matrix<double, 7, 1> &foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_transfer_dur, double swing_down_dur) const;
  PiecewisePolynomial<double> GeneratePelvisTraj(KinematicsCache<double> cache, double & pelvis_height_above_sole,
                                                 double& liftoff_time, double & next_liftoff_time, Eigen::Isometry3d nxt_stance_foot_pose,
  Eigen::Isometry3d nxt_swing_foot_pose);
  static double get_weight_distribution(const ContactState &cs);
};

