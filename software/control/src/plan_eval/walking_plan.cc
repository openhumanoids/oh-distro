#include "walking_plan.h"
#include "generate_spline.h"
#include "drake/util/lcmUtil.h"

#include <fstream>
#include <iomanip>

inline double avg_angle(double a, double b) {
  double x = fmod(fabs(a-b), 2*M_PI);
  if (x >= 0 && x <= M_PI)
    return fmod((a + b) / 2., 2*M_PI);
  else if (x >= M_PI && x < 1.5*M_PI)
    return fmod((a + b) / 2., 2*M_PI) + M_PI;
  else
    return fmod((a + b) / 2., 2*M_PI) - M_PI;
}

double WalkingPlan::get_weight_distribution(const ContactState &cs) {
  if (cs.is_in_contact(ContactState::L_FOOT) && cs.is_in_contact(ContactState::R_FOOT))
    return 0.5;
  if (cs.is_in_contact(ContactState::L_FOOT) && !cs.is_in_contact(ContactState::R_FOOT))
    return 1;
  if (!cs.is_in_contact(ContactState::L_FOOT) && cs.is_in_contact(ContactState::R_FOOT))
    return 0;
  return 0.5;
}

void WalkingPlan::LoadConfigurationFromYAML(const std::string &name) {
  p_ss_duration_ = config_["ss_duration"].as<double>();
  p_ds_duration_ = config_["ds_duration"].as<double>();
  p_lower_z_vel_ = config_["lower_z_vel"].as<double>();

  std::cout << "p_lower_z_vel_: " << p_lower_z_vel_ << std::endl;
  std::cout << "p_ss_duration_: " << p_ss_duration_ << std::endl;
  std::cout << "p_ds_duration_: " << p_ds_duration_ << std::endl;
}

// called on every touch down
void WalkingPlan::GenerateTrajs(const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd, const ContactState &planned_cs) {
  // current state
  KinematicsCache<double> cache_est = robot_.doKinematics(est_q, est_qd);

  Eigen::Vector2d com0, comd0, com1;
  Eigen::Matrix<double,7,1> pelv0, feet0[2], pelv1;
  Eigen::Isometry3d feet_pose[2];

  com0 = robot_.centerOfMass(cache_est).head(2);
  comd0 = (robot_.centerOfMassJacobian(cache_est) * est_qd).head(2);
  pelv0 = Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, rpc_.pelvis_id));
  feet_pose[Side::LEFT] = robot_.relativeTransform(cache_est, 0, rpc_.foot_ids.at(Side::LEFT));
  feet_pose[Side::RIGHT] = robot_.relativeTransform(cache_est, 0, rpc_.foot_ids.at(Side::RIGHT));
  feet0[Side::LEFT] = Isometry3dToVector7d(feet_pose[Side::LEFT]);
  feet0[Side::RIGHT] = Isometry3dToVector7d(feet_pose[Side::RIGHT]);

  // reinit body trajs
  body_motions_.resize(3);  /// 0 is pelvis, 1 is stance foot, 2 is swing foot

  // figure out the current contact state
  ContactState nxt_contact_state = ContactState::DS();

  // no more steps, go to double support middle
  if (footstep_plan_.empty()) {
    int num_T = 2;
    for (int i = 0; i < 3; i++)
      body_motions_[i] = MakeDefaultBodyMotionData(num_T);

    // time tape
    std::vector<double> Ts(num_T);
    Ts[0] = 0;
    Ts[1] = p_ds_duration_;

    // com / cop
    Eigen::Isometry3d mid_stance_foot[2];
    for (int i = 0; i < 2; i++) {
      mid_stance_foot[i].linear() = Eigen::Matrix3d::Identity();
      mid_stance_foot[i].translation() = Eigen::Vector3d(0.04, 0, 0);
      mid_stance_foot[i] = feet_pose[i] * mid_stance_foot[i];
    }
    com1 = (mid_stance_foot[0].translation().head(2) + mid_stance_foot[1].translation().head(2)) / 2.;

    // make zmp
    std::vector<Eigen::Vector2d> com_knots(num_T);
    com_knots[0] = com0;
    com_knots[1] = com1;
    zmp_traj_ = GeneratePCHIPSpline(Ts, com_knots);
    Eigen::Vector4d x0(Eigen::Vector4d::Zero());
    x0.head(2) = com0;
    // TODO: assuming zero comd?
    x0.tail(2) = comd0;
    zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

    // make pelv
    pelv1.head(3) = (mid_stance_foot[0].translation() + mid_stance_foot[1].translation()) / 2.;
    pelv1[2] = std::min(mid_stance_foot[0].translation()[2], mid_stance_foot[1].translation()[2]) + p_zmp_height_;
    Eigen::Vector4d foot_quat[2] = {feet0[0].tail<4>(), feet0[1].tail<4>()};
    double yaw = avg_angle(quat2rpy(foot_quat[0])[2], quat2rpy(foot_quat[1])[2]);
    pelv1.tail(4) = rpy2quat(Eigen::Vector3d(0, 0, yaw));

    std::vector<Eigen::Vector7d> pelv_knots(num_T);
    pelv_knots[0] = pelv0;
    pelv_knots[1] = pelv1;

    body_motions_[0].body_or_frame_id = rpc_.pelvis_id;
    body_motions_[0].control_pose_when_in_contact.resize(num_T, true);
    body_motions_[0].trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make stance foot
    Side sides[2] = {Side::LEFT, Side::RIGHT};
    for (int i = 0; i < 2; i++) {
      std::vector<Eigen::Vector7d> stance_knots(num_T, feet0[i]);
      body_motions_[1+i].body_or_frame_id = rpc_.foot_ids.at(sides[i]);
      body_motions_[1+i].trajectory = GenerateCubicCartesianSpline(Ts, stance_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));
    }

    // make weight distribuition
    std::vector<Eigen::Matrix<double,1,1>> WL(num_T);
    WL[0](0,0) = get_weight_distribution(planned_cs);
    WL[1](0,0) = get_weight_distribution(ContactState::DS());
    weight_distribution_ = GenerateLinearSpline(Ts, WL);

    // setup contact_state tape
    contact_state_.clear();
    contact_state_.push_back(std::pair<ContactState, double>(nxt_contact_state, INFINITY));
  }
  else {
    int num_T = 3;
    for (int i = 0; i < 3; i++)
      body_motions_[i] = MakeDefaultBodyMotionData(num_T);

    Side stance_foot = Side::LEFT;
    Side swing_foot = Side::RIGHT;
    nxt_contact_state = ContactState::SSL();
    drc::footstep_t &cur_step = footstep_plan_.front();
    // ssr
    if (!cur_step.is_right_foot) {
      stance_foot = Side::RIGHT;
      swing_foot = Side::LEFT;
      nxt_contact_state = ContactState::SSR();
    }

    // TODO: change to vel
    //double p_ss_duration_ = cur_step.params.step_speed;
    //double p_ds_duration_ = cur_step.params.drake_min_hold_time;

    // time tape
    std::vector<double> Ts(num_T);
    Ts[0] = 0;
    Ts[1] = p_ds_duration_;
    Ts[2] = p_ss_duration_ + p_ds_duration_;

    // hold arm joints, I am assuming the leg joints will just be ignored..
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(est_q.size());
    q_trajs_ = GenerateCubicSpline(Ts, std::vector<Eigen::VectorXd>(num_T, init_q_), zero, zero);

    // com / cop
    double zmp_shift_in = cur_step.params.drake_instep_shift;
    if (stance_foot == Side::LEFT)
      zmp_shift_in = -zmp_shift_in;

    Eigen::Isometry3d mid_stance_foot(Eigen::Isometry3d::Identity());
    mid_stance_foot.translation() = Eigen::Vector3d(0.04, zmp_shift_in, 0);
    mid_stance_foot = feet_pose[stance_foot.underlying()] * mid_stance_foot;
    com1 = mid_stance_foot.translation().head(2);

    // make zmp
    std::vector<Eigen::Vector2d> com_knots(num_T);
    com_knots[0] = com0;
    com_knots[1] = com_knots[2] = com1;
    zmp_traj_ = GeneratePCHIPSpline(Ts, com_knots);
    Eigen::Vector4d x0(Eigen::Vector4d::Zero());
    x0.head(2) = com0;
    // TODO: assuming zero comd?
    x0.tail(2) = comd0;
    zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

    // make pelv
    pelv1.head(3) = mid_stance_foot.translation();
    pelv1[2] += p_zmp_height_;
    Eigen::Vector4d foot_quat(feet0[stance_foot.underlying()].tail(4));
    double yaw = quat2rpy(foot_quat)[2];
    pelv1.tail(4) = rpy2quat(Eigen::Vector3d(0, 0, yaw));

    std::vector<Eigen::Vector7d> pelv_knots(num_T);
    pelv_knots[0] = pelv0;
    pelv_knots[1] = pelv_knots[2] = pelv1;

    body_motions_[0].body_or_frame_id = rpc_.pelvis_id;
    body_motions_[0].control_pose_when_in_contact.resize(num_T, true);
    body_motions_[0].trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make stance foot
    std::vector<Eigen::Vector7d> stance_knots(num_T, feet0[stance_foot.underlying()]);
    body_motions_[1].body_or_frame_id = rpc_.foot_ids.at(stance_foot);
    body_motions_[1].trajectory = GenerateCubicCartesianSpline(Ts, stance_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make swing foot
    Eigen::Vector7d swing_touchdown_pose;
    swing_touchdown_pose(0) = cur_step.pos.translation.x;
    swing_touchdown_pose(1) = cur_step.pos.translation.y;
    swing_touchdown_pose(2) = cur_step.pos.translation.z;
    swing_touchdown_pose(3) = cur_step.pos.rotation.w;
    swing_touchdown_pose(4) = cur_step.pos.rotation.x;
    swing_touchdown_pose(5) = cur_step.pos.rotation.y;
    swing_touchdown_pose(6) = cur_step.pos.rotation.z;
    swing_touchdown_pose.tail(4).normalize();

    body_motions_[2].body_or_frame_id = rpc_.foot_ids.at(swing_foot);
    body_motions_[2].trajectory = GenerateSwingTraj(feet0[swing_foot.underlying()], swing_touchdown_pose, cur_step.params.step_height, p_ds_duration_, p_ss_duration_ / 2., p_ss_duration_ / 2.);

    // make weight distribuition
    std::vector<Eigen::Matrix<double,1,1>> WL(num_T);
    WL[0](0,0) = get_weight_distribution(planned_cs);
    WL[1](0,0) = get_weight_distribution(nxt_contact_state);
    WL[2](0,0) = get_weight_distribution(nxt_contact_state);
    weight_distribution_ = GenerateLinearSpline(Ts, WL);

    // setup contact state
    contact_state_.clear();
    contact_state_.push_back(std::pair<ContactState, double>(ContactState::DS(), p_ds_duration_));
    contact_state_.push_back(std::pair<ContactState, double>(nxt_contact_state, p_ss_duration_ + p_ds_duration_));
  }

  // reset clock
  interp_t0_ = -1;
}

PiecewisePolynomial<double> WalkingPlan::GenerateSwingTraj(const Eigen::Vector7d &foot0, const Eigen::Vector7d &foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_down_dur) const {
  int num_T = 4;
  std::vector<double> swingT(num_T);
  swingT[0] = 0;
  swingT[1] = pre_swing_dur;
  swingT[2] = pre_swing_dur + swing_up_dur;
  swingT[3] = pre_swing_dur + swing_up_dur + swing_down_dur;

  Eigen::Vector7d swing_mid = foot1;
  swing_mid.head(3) = (foot0.head(3) + foot1.head(3)) / 2.;
  swing_mid[2] = std::max(foot1[2], foot0[2]) + mid_z_offset;

  std::vector<Eigen::Vector7d> swing_pose_knots(num_T);
  std::vector<Eigen::Vector7d> swing_vel_knots(num_T, Eigen::Vector7d::Zero());
  swing_pose_knots[0] = swing_pose_knots[1] = foot0;
  swing_pose_knots[2] = swing_mid;
  swing_pose_knots[3] = foot1;
  swing_vel_knots[2].head(2) = (foot1.head(2) - foot0.head(2)) / (swing_up_dur + swing_down_dur);

  return GenerateCubicCartesianSpline(swingT, swing_pose_knots, swing_vel_knots);
}

void WalkingPlan::HandleCommittedRobotPlan(const void *plan_msg,
                                           const DrakeRobotState &est_rs,
                                           const Eigen::VectorXd &last_q_d) {
  // state machine
  cur_state_ = WEIGHT_TRANSFER;

  footstep_plan_.clear();
  contact_state_.clear();

  const drc::walking_plan_request_t *msg = (const drc::walking_plan_request_t *)plan_msg;
  for (size_t i = 0; i < msg->footstep_plan.footsteps.size(); i++) {
    footstep_plan_.push_back(msg->footstep_plan.footsteps[i]);
  }

  // save the current joint configuration, that's what we are tracking during walking
  init_q_ = est_rs.q;

  // generate all trajs
  GenerateTrajs(est_rs.q, est_rs.qd, ContactState::DS());

  // constrained DOFs
  constrained_dofs_.clear();
  for (auto it = rpc_.position_indices.arms.begin(); it != rpc_.position_indices.arms.end(); it++) {
    const std::vector<int> &indices = it->second;
    for (size_t i = 0; i < indices.size(); i++)
      constrained_dofs_.push_back(indices[i]);
  }
  // neck
  for (size_t i = 0; i < rpc_.position_indices.neck.size(); i++)
    constrained_dofs_.push_back(rpc_.position_indices.neck[i]);
  // back
  constrained_dofs_.push_back(rpc_.position_indices.back_bkz);
  //constrained_dofs_.push_back(rpc_.position_indices.back_bky);

}

void WalkingPlan::SwitchContactState(double cur_time) {
  contact_state_.pop_front();
  contact_switch_time_ = cur_time;
}

drake::lcmt_qp_controller_input WalkingPlan::MakeQPInput(const DrakeRobotState &est_rs) {
  static double last_time = 0;
  double cur_time = est_rs.t;
  double dt = std::min(cur_time - last_time, 1.0);
  last_time = cur_time;

  if (interp_t0_ == -1)
    interp_t0_ = cur_time;

  double plan_time = cur_time - interp_t0_;

  // can't use reference for this one because ref becomes invalid when queue pops
  const ContactState planned_cs = cur_planned_contact_state();
  const ContactState &est_cs = est_rs.contact_state;
  double planned_contact_swith_time = cur_planned_contact_swith_time();

  // state machine part
  switch (cur_state_) {
    case WEIGHT_TRANSFER:
      if (plan_time >= planned_contact_swith_time) {
        // change contact
        SwitchContactState(cur_time);
        cur_state_ = SWING;
        std::cout << "Weight transfer -> Swing @ " << plan_time << std::endl;
      }

      break;

    case SWING:
      // hack the swing traj if we are past the planned touchdown time
      if (plan_time >= planned_contact_swith_time) {
        // swing trajectory is always idx 2..
        int last_idx = body_motions_[2].trajectory.getNumberOfSegments() - 1;
        Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> last_knots = body_motions_[2].trajectory.getPolynomialMatrix(last_idx);

        // hack the z position and velocity only
        double t0 = body_motions_[2].trajectory.getStartTime(last_idx);
        double t1 = body_motions_[2].trajectory.getEndTime(last_idx);
        double z0 = body_motions_[2].trajectory.value(t0)(2,0);
        double z1 = body_motions_[2].trajectory.value(t1)(2,0) + p_lower_z_vel_ * dt;
        double v0 = 0;
        double v1 = p_lower_z_vel_;
        Eigen::Vector4d new_z_coeffs = GetCubicSplineCoeffs(t1-t0, z0, z1, v0, v1);

        // first one is position
        Polynomial<double> new_z(new_z_coeffs);
        last_knots(2,0) = new_z;
        body_motions_[2].trajectory.setPolynomialMatrixBlock(last_knots, last_idx);
      }

      // check for touch down only after half swing
      if (plan_time >= planned_contact_swith_time - 0.5 * p_ss_duration_ &&
          est_cs.is_in_contact(ContactState::L_FOOT) &&
          est_cs.is_in_contact(ContactState::R_FOOT)) {
        // change contact
        SwitchContactState(cur_time);
        cur_state_ = WEIGHT_TRANSFER;
        std::cout << "Swing -> Weight transfer @ " << plan_time << std::endl;

        // dequeue foot steps, generate new trajectories
        footstep_plan_.pop_front();
        GenerateTrajs(est_rs.q, est_rs.qd, planned_cs);

        // all tapes assumes 0 sec start, so need to reset clock
        interp_t0_ = cur_time;
        plan_time = cur_time - interp_t0_;
      }

      break;

    default:
      throw std::runtime_error("Unknown walking state");
  }

  // make contact support data
  support_state_ = MakeDefaultSupportState(contact_state_.front().first);
  // interp weight distribution
  double wl = weight_distribution_.value(plan_time).value();
  wl = std::min(wl, 1.0);
  wl = std::max(wl, 0.0);

  for (size_t s = 0; s < support_state_.size(); s++) {
    // only foot
    if (support_state_[s].body == rpc_.foot_ids.at(Side::LEFT)) {
      support_state_[s].total_normal_force_upper_bound *= wl;
      support_state_[s].total_normal_force_lower_bound = p_min_Fz_;
    }
    else if (support_state_[s].body == rpc_.foot_ids.at(Side::RIGHT)) {
      support_state_[s].total_normal_force_upper_bound *= (1 - wl);
      support_state_[s].total_normal_force_lower_bound = p_min_Fz_;
    }

    support_state_[s].total_normal_force_upper_bound = std::max(support_state_[s].total_normal_force_upper_bound, support_state_[s].total_normal_force_lower_bound);
  }

  // make qp input
  bool apply_torque_alpha_filter = (plan_time < p_initial_transition_time_) || (cur_time - contact_switch_time_ < p_initial_transition_time_);
  return MakeDefaultQPInput(cur_time, plan_time, "walking", apply_torque_alpha_filter);
}


