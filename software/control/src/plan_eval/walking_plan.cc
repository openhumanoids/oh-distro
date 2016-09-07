#include "walking_plan.h"
#include "generate_spline.h"
#include "drake/util/lcmUtil.h"

#include <fstream>
#include <iomanip>
#include <drc/string_t.hpp>

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
  p_pre_weight_transfer_scale_ = config_["pre_weight_transfer_scale"].as<double>();
  p_pre_weight_transfer_scale_ = std::min(0.5, p_pre_weight_transfer_scale_);
  p_pre_weight_transfer_scale_ = std::max(0.0, p_pre_weight_transfer_scale_);

  p_swing_foot_xy_weight_mulitplier_ = config_["swing_foot_xy_weight_mulitplier"].as<double>();
  p_pelvis_z_weight_mulitplier_ = config_["pelvis_z_weight_mulitplier"].as<double>();
  p_left_foot_zmp_y_shift_ = config_["left_foot_zmp_y_shift"].as<double>();
  p_right_foot_zmp_y_shift_ = config_["right_foot_zmp_y_shift"].as<double>();

  std::cout << "p_lower_z_vel_: " << p_lower_z_vel_ << std::endl;
  std::cout << "p_ss_duration_: " << p_ss_duration_ << std::endl;
  std::cout << "p_ds_duration_: " << p_ds_duration_ << std::endl;

  std::cout << "p_swing_foot_xy_weight_mulitplier_: " << p_swing_foot_xy_weight_mulitplier_ << std::endl;
  std::cout << "p_pelvis_z_weight_mulitplier_: " << p_pelvis_z_weight_mulitplier_ << std::endl;
  std::cout << "p_left_foot_zmp_y_shift_: " << p_left_foot_zmp_y_shift_ << std::endl;
  std::cout << "p_right_foot_zmp_y_shift_: " << p_right_foot_zmp_y_shift_ << std::endl;
}

Eigen::Vector2d WalkingPlan::Footstep2DesiredZMP(Side side, const Eigen::Isometry3d &step) const {
  double zmp_shift_in = p_left_foot_zmp_y_shift_;
  if (side == Side::RIGHT) {
    zmp_shift_in = p_right_foot_zmp_y_shift_;
  }
  const Eigen::Matrix3Xd &offsets = contact_offsets.find(rpc_.foot_ids.at(side))->second;
  Eigen::Vector3d avg_contact_offset(Eigen::Vector3d::Zero());
  for (int i = 0; i < offsets.cols(); i++)
    avg_contact_offset += offsets.col(i);
  avg_contact_offset = avg_contact_offset / offsets.cols();
  // offset y on in body frame
  avg_contact_offset[1] += zmp_shift_in;
  Eigen::Isometry3d mid_stance_foot(Eigen::Isometry3d::Identity());
  mid_stance_foot.translation() = avg_contact_offset;
  mid_stance_foot = step * mid_stance_foot;
  return mid_stance_foot.translation().head(2);
}

// called on every touch down
void WalkingPlan::GenerateTrajs(const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd, const ContactState &planned_cs) {
  // current state
  KinematicsCache<double> cache_est = robot_.doKinematics(est_q, est_qd);

  Eigen::Vector2d com0, comd0;
  Eigen::Vector2d zmp_d0, zmp_d1;
  Eigen::Matrix<double,7,1> pelv0, feet0[2], pelv1, torso0, torso1;
  Eigen::Isometry3d feet_pose[2];

  com0 = robot_.centerOfMass(cache_est).head(2);
  comd0 = (robot_.centerOfMassJacobian(cache_est) * est_qd).head(2);
  Eigen::Vector4d x0;
  x0.head(2) = com0;
  x0.tail(2) = comd0;

  pelv0 = Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, rpc_.pelvis_id));
  torso0 = Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, rpc_.torso_id));

  feet_pose[Side::LEFT] = robot_.relativeTransform(cache_est, 0, rpc_.foot_ids.at(Side::LEFT));
  feet_pose[Side::RIGHT] = robot_.relativeTransform(cache_est, 0, rpc_.foot_ids.at(Side::RIGHT));
  feet0[Side::LEFT] = Isometry3dToVector7d(feet_pose[Side::LEFT]);
  feet0[Side::RIGHT] = Isometry3dToVector7d(feet_pose[Side::RIGHT]);

  // reinit body trajs
  body_motions_.resize(4);  /// 0 is pelvis, 1 is stance foot, 2 is swing foot

  // figure out the current contact state
  ContactState nxt_contact_state = ContactState::DS();

  // no more steps, go to double support middle
  if (footstep_plan_.empty()) {
    // 0: the current configuration
    // 1: start of weight tranfer
    // 2: end of weight transfer
    int num_T = 3;
    for (size_t i = 0; i < body_motions_.size(); i++)
      body_motions_[i] = MakeDefaultBodyMotionData(num_T);

    // time tape
    std::vector<double> Ts(num_T);
    Ts[0] = 0;
    Ts[1] = p_pre_weight_transfer_scale_ * p_ds_duration_;
    Ts[2] = p_ds_duration_;

    // com / cop
    Eigen::Isometry3d mid_stance_foot[2];
    for (int i = 0; i < 2; i++) {
      mid_stance_foot[i].linear() = Eigen::Matrix3d::Identity();
      mid_stance_foot[i].translation() = Eigen::Vector3d(0.04, 0, 0);
      mid_stance_foot[i] = feet_pose[i] * mid_stance_foot[i];
    }
    zmp_d1 = (mid_stance_foot[0].translation().head(2) + mid_stance_foot[1].translation().head(2)) / 2.;

    // make zmp
    std::vector<Eigen::Vector2d> zmp_knots(num_T);
    zmp_d0 = com0;
    zmp_knots[0] = zmp_d0;
    zmp_knots[1] = zmp_d0;
    zmp_knots[2] = zmp_d1;
    zmp_traj_ = GeneratePCHIPSpline(Ts, zmp_knots);
    zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

    // make pelv
    pelv1.head(3) = (mid_stance_foot[0].translation() + mid_stance_foot[1].translation()) / 2.;
    pelv1[2] = std::min(mid_stance_foot[0].translation()[2], mid_stance_foot[1].translation()[2]) + p_zmp_height_;
    Eigen::Vector4d foot_quat[2] = {feet0[0].tail<4>(), feet0[1].tail<4>()};
    double yaw = avg_angle(quat2rpy(foot_quat[0])[2], quat2rpy(foot_quat[1])[2]);
    pelv1.tail(4) = rpy2quat(Eigen::Vector3d(0, 0, yaw));

    std::vector<Eigen::Vector7d> pelv_knots(num_T);
    pelv_knots[0] = pelv0;
    pelv_knots[1] = pelv0;
    pelv_knots[2] = pelv1;

    BodyMotionData &pelvis_BMD = get_pelvis_body_motion_data();
    pelvis_BMD.body_or_frame_id = rpc_.pelvis_id;
    pelvis_BMD.control_pose_when_in_contact.resize(num_T, true);
    pelvis_BMD.trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make torso
    torso1 = pelv1; // the position doesn't matter, we only track orientation
    Eigen::AngleAxisd torso_rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    torso_rot = torso_rot * Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY());
    torso1.tail(4) = rotmat2quat(torso_rot.toRotationMatrix());

    std::vector<Eigen::Vector7d> torso_knots(num_T);
    torso_knots[0] = torso0;
    torso_knots[1] = torso0;
    torso_knots[2] = torso1;
    BodyMotionData &torso_BMD = get_torso_body_motion_data();
    torso_BMD.body_or_frame_id = rpc_.torso_id;
    torso_BMD.trajectory = GenerateCubicCartesianSpline(Ts, torso_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make stance foot
    Side sides[2] = {Side::LEFT, Side::RIGHT};
    for (int i = 0; i < 2; i++) {
      std::vector<Eigen::Vector7d> stance_knots(num_T, feet0[i]);
      body_motions_[2+i].body_or_frame_id = rpc_.foot_ids.at(sides[i]);
      body_motions_[2+i].trajectory = GenerateCubicCartesianSpline(Ts, stance_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));
    }

    // make weight distribuition
    std::vector<Eigen::Matrix<double,1,1>> WL(num_T);
    WL[0](0,0) = get_weight_distribution(planned_cs);
    WL[1](0,0) = get_weight_distribution(planned_cs);
    WL[2](0,0) = get_weight_distribution(ContactState::DS());
    weight_distribution_ = GenerateLinearSpline(Ts, WL);

    // setup contact_state tape
    contact_state_.clear();
    contact_state_.push_back(std::pair<ContactState, double>(nxt_contact_state, INFINITY));
  }
  else {
    // 0: start of weight tranfer
    // 1: end of weight transfer
    // 2: swing phase
    int num_T = 3;

    /// 0 is pelvis, 1 is torso, 2 is stance foot, 3 is swing foot
    for (size_t i = 0; i < body_motions_.size(); i++)
      body_motions_[i] = MakeDefaultBodyMotionData(num_T);

    Side stance_foot = Side::LEFT;
    Side swing_foot = Side::RIGHT;
    nxt_contact_state = ContactState::SSL();
    drc::footstep_t &cur_step = footstep_plan_.front();
    // single support right
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

    // hold all joints. I am assuming the leg joints will just be ignored in the qp..
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(est_q.size());
    q_trajs_ = GenerateCubicSpline(Ts, std::vector<Eigen::VectorXd>(num_T, init_q_), zero, zero);

    // make zmp
    int num_look_ahead = 3;

    // figure out the desired zmps
    std::vector<Eigen::Vector2d> desired_zmps;
    desired_zmps.push_back(Footstep2DesiredZMP(stance_foot, feet_pose[stance_foot.underlying()])); // the touchdoen foot xy position

    for (const auto &msg : footstep_plan_) {
      Eigen::Isometry3d footstep;
      footstep.translation() = Eigen::Vector3d(msg.pos.translation.x, msg.pos.translation.y, msg.pos.translation.z);
      Eigen::Quaterniond rot(msg.pos.rotation.w, msg.pos.rotation.x, msg.pos.rotation.y, msg.pos.rotation.z);
      footstep.linear() = Eigen::Matrix3d(rot.normalized());
      Side side = msg.is_right_foot ? Side::RIGHT : Side::LEFT;

      desired_zmps.push_back(Footstep2DesiredZMP(side, footstep));
    }

    // figure out where we want the desired zmp for NOW
    if (planned_cs.is_in_contact(ContactState::L_FOOT) && planned_cs.is_in_contact(ContactState::R_FOOT)) {
      zmp_d0 = (Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]) + Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT])) / 2.;
    }
    else if (planned_cs.is_in_contact(ContactState::L_FOOT)) {
      zmp_d0 = Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]);
    }
    else if (planned_cs.is_in_contact(ContactState::R_FOOT)) {
      zmp_d0 = Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT]);
    }
    else {
      throw std::runtime_error("robot flying.");
    }

    zmp_traj_ = PlanZMPTraj(desired_zmps, num_look_ahead, zmp_d0);
    zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

    // make pelv
    pelv1.head(2) = zmp_d0;
    pelv1[2] = p_zmp_height_ + feet_pose[stance_foot.underlying()].translation()[2]; ///< This offset the pelv z relative to ankle.
    Eigen::Vector4d foot_quat(feet0[stance_foot.underlying()].tail(4));
    double yaw = quat2rpy(foot_quat)[2];
    pelv1.tail(4) = rpy2quat(Eigen::Vector3d(0, 0, yaw));

    std::vector<Eigen::Vector7d> pelv_knots(num_T);
    pelv_knots[0] = pelv0;
    pelv_knots[1] = pelv_knots[2] = pelv1;
    BodyMotionData &pelvis_BMD = get_pelvis_body_motion_data();
    pelvis_BMD.body_or_frame_id = rpc_.pelvis_id;
    pelvis_BMD.control_pose_when_in_contact.resize(num_T, true);
    pelvis_BMD.trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make torso
    torso1 = pelv1; // the position doesn't matter, we only track orientation
    Eigen::AngleAxisd torso_rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()); // yaw
    torso_rot = torso_rot * Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()); // pitch
    torso1.tail(4) = rotmat2quat(torso_rot.toRotationMatrix());
    std::vector<Eigen::Vector7d> torso_knots(num_T);
    torso_knots[0] = torso0;
    torso_knots[1] = torso_knots[2] = torso1;
    BodyMotionData &torso_BMD = get_torso_body_motion_data();
    torso_BMD.body_or_frame_id = rpc_.torso_id;
    torso_BMD.trajectory = GenerateCubicCartesianSpline(Ts, torso_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make stance foot.
    // This is not quite right, since the touchdown foot pose will change after solid contact.
    // Assume the QP will not try to control pose for the contact feet.
    std::vector<Eigen::Vector7d> stance_knots(num_T, feet0[stance_foot.underlying()]);
    BodyMotionData &stance_BMD = get_stance_foot_body_motion_data();
    stance_BMD.body_or_frame_id = rpc_.foot_ids.at(stance_foot);
    stance_BMD.trajectory = GenerateCubicCartesianSpline(Ts, stance_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // swing foot traj will be planned at liftoff
    std::vector<Eigen::Vector7d> swing_knots(num_T, feet0[swing_foot.underlying()]);
    BodyMotionData &swing_BMD = get_swing_foot_body_motion_data();
    swing_BMD.body_or_frame_id = rpc_.foot_ids.at(swing_foot);
    swing_BMD.trajectory = GenerateCubicCartesianSpline(Ts, swing_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

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

  // pelvis Z weight multiplier
  // This is really dumb, but the multipler is ang then pos, check instQP.
  get_pelvis_body_motion_data().weight_multiplier[5] = 5;
  // don't track x and y position of the pelvis
  get_pelvis_body_motion_data().weight_multiplier[4] = 0;
  get_pelvis_body_motion_data().weight_multiplier[3] = 0;

  // don't track xyz of the torso
  for (int i = 3; i < 6; i++)
    get_torso_body_motion_data().weight_multiplier[i] = 0;

  // reset clock
  interp_t0_ = -1;
}

PiecewisePolynomial<double> WalkingPlan::GenerateSwingTraj(const Eigen::Vector7d &foot0, const Eigen::Vector7d &foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_transfer_dur, double swing_down_dur) const {
  int num_T = 5;
  std::vector<double> swingT(num_T);
  swingT[0] = 0;
  swingT[1] = pre_swing_dur;
  swingT[2] = pre_swing_dur + swing_up_dur;
  swingT[3] = pre_swing_dur + swing_up_dur + swing_transfer_dur;
  swingT[4] = pre_swing_dur + swing_up_dur + swing_transfer_dur + swing_down_dur;

  Eigen::Vector7d swing_mid = foot1;
  swing_mid.head(3) = (foot0.head(3) + foot1.head(3)) / 2.;
  swing_mid[2] = std::max(foot1[2], foot0[2]) + mid_z_offset;

  std::vector<Eigen::Vector7d> swing_pose_knots(num_T);
  std::vector<Eigen::Vector7d> swing_vel_knots(num_T, Eigen::Vector7d::Zero());
  swing_pose_knots[0] = swing_pose_knots[1] = foot0;
  swing_pose_knots[2] = swing_mid;
  // right above the touchdown pose, only increase z
  swing_pose_knots[3] = foot1;
  swing_pose_knots[3][2] = swing_mid[2];
  swing_pose_knots[4] = foot1;
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
  //constrained_dofs_.push_back(rpc_.position_indices.back_bkz);
  //constrained_dofs_.push_back(rpc_.position_indices.back_bky);

}

void WalkingPlan::SwitchContactState(double cur_time) {
  contact_state_.pop_front();
  contact_switch_time_ = cur_time;

  //reset the tare FT flag
  have_tared_swing_leg_ft_ = false;
}

void WalkingPlan::TareSwingLegForceTorque() {
  drc::string_t msg;

  const ContactState planned_cs = cur_planned_contact_state();

  // need to tare the opposite foot of the one that is in contact
  if (planned_cs.is_in_contact(ContactState::ContactBody::L_FOOT)){
    std::cout << "sending tare RIGHT FT message" << std::endl;
    msg.data = "right";
  } else if (planned_cs.is_in_contact(ContactState::ContactBody::R_FOOT)){
    std::cout << "sending tare LEFT FT message" << std::endl;
    msg.data = "left";
  } else {
    std::cout << "neither right or left foot is in contact, not sending tare FT message" << std::endl;
    return;
  }

  // first test it out without actually publishing
 lcm_handle_.publish("TARE_FOOT_SENSORS", &msg);
  have_tared_swing_leg_ft_ = true;
}

PiecewisePolynomial<double> WalkingPlan::PlanZMPTraj(const std::vector<Eigen::Vector2d> &zmp_d, int num_of_zmp_knots, const Eigen::Vector2d &current_mid_stance_foot) const {
  std::vector<double> zmp_T; // double support weight transfer, followed by single support
  std::vector<Eigen::Vector2d> zmp_knots;

  int min_size = std::min(num_of_zmp_knots, (int)zmp_d.size());

  double cur_time = 0;
  zmp_knots.push_back(current_mid_stance_foot);
  zmp_T.push_back(cur_time);

  for (int i = 0; i < min_size; i++) {
    cur_time = zmp_T.back();
    // add a double support phase
    // last step, need to stop in the middle
    if (i == min_size - 1) {
      zmp_knots.push_back((zmp_knots.back() + zmp_d[i]) / 2.);
      zmp_T.push_back(cur_time + p_ds_duration_);
    }
    else {
      zmp_knots.push_back(zmp_d[i]);
      zmp_T.push_back(cur_time + p_ds_duration_);
    }

    // add a single support phase
    cur_time = zmp_T.back();
    zmp_knots.push_back(zmp_knots.back());
    zmp_T.push_back(cur_time + p_ss_duration_);
  }

  return GeneratePCHIPSpline(zmp_T, zmp_knots);
}

drake::lcmt_qp_controller_input WalkingPlan::MakeQPInput(const DrakeRobotState &est_rs) {
  static double last_time = 0;
  double cur_time = est_rs.t;
  double dt = std::min(cur_time - last_time, 1.0);
  last_time = cur_time;

  if (interp_t0_ == -1)
    interp_t0_ = cur_time;

  double plan_time = cur_time - interp_t0_;

  // update the plan status depending on whether or not we are finished
  if (footstep_plan_.empty() && (plan_time > p_ds_duration_)){
    plan_status_.executionStatus = PlanExecutionStatus::FINISHED;
  } else{
    plan_status_.executionStatus = PlanExecutionStatus::EXECUTING;
  }

  // can't use reference for this one because ref becomes invalid when queue pops
  const ContactState planned_cs = cur_planned_contact_state();
  const ContactState &est_cs = est_rs.contact_state;
  double planned_contact_swith_time = cur_planned_contact_swith_time();

  bool late_touchdown = false;

  // state machine part
  switch (cur_state_) {
    case WEIGHT_TRANSFER:
      if (plan_time >= planned_contact_swith_time) {
        // replace the swing up segment with a new one that starts from the current foot pose.
        drc::footstep_t &cur_step = footstep_plan_.front();
        Side swing_foot = cur_step.is_right_foot ? Side::RIGHT : Side::LEFT;

        KinematicsCache<double> cache = robot_.doKinematics(est_rs.q, est_rs.qd);
        Eigen::Isometry3d swing_foot0 = robot_.relativeTransform(cache, 0, rpc_.foot_ids.at(swing_foot));

        Eigen::Vector7d swing_touchdown_pose;
        swing_touchdown_pose(0) = cur_step.pos.translation.x;
        swing_touchdown_pose(1) = cur_step.pos.translation.y;
        swing_touchdown_pose(2) = cur_step.pos.translation.z;
        swing_touchdown_pose(3) = cur_step.pos.rotation.w;
        swing_touchdown_pose(4) = cur_step.pos.rotation.x;
        swing_touchdown_pose(5) = cur_step.pos.rotation.y;
        swing_touchdown_pose(6) = cur_step.pos.rotation.z;
        swing_touchdown_pose.tail(4).normalize();

        BodyMotionData& swing_BMD = get_swing_foot_body_motion_data();
        swing_BMD.body_or_frame_id = rpc_.foot_ids.at(swing_foot);
        swing_BMD.trajectory = GenerateSwingTraj(Isometry3dToVector7d(swing_foot0), swing_touchdown_pose, cur_step.params.step_height, p_ds_duration_, p_ss_duration_ / 3., p_ss_duration_ / 3., p_ss_duration_ / 3.);
        // increase weights for swing foot xy tracking
        swing_BMD.weight_multiplier[4] = 15;
        swing_BMD.weight_multiplier[3] = 15;

        // change contact
        SwitchContactState(cur_time);
        cur_state_ = SWING;
        std::cout << "Weight transfer -> Swing @ " << plan_time << std::endl;
      }

      break;

    case SWING:
      // hack the swing traj if we are past the planned touchdown time
      if (plan_time >= planned_contact_swith_time) {
        late_touchdown = true;

        BodyMotionData& swing_BMD = get_swing_foot_body_motion_data();
        int last_idx = swing_BMD.trajectory.getNumberOfSegments() - 1;
        Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> last_knots = swing_BMD.trajectory.getPolynomialMatrix(last_idx);

        // hack the z position and velocity only
        double t0 = swing_BMD.trajectory.getStartTime(last_idx);
        double t1 = swing_BMD.trajectory.getEndTime(last_idx);
        double z0 = swing_BMD.trajectory.value(t0)(2,0);
        double z1 = swing_BMD.trajectory.value(t1)(2,0) + p_lower_z_vel_ * dt;
        double v0 = 0;
        double v1 = p_lower_z_vel_;
        Eigen::Vector4d new_z_coeffs = GetCubicSplineCoeffs(t1-t0, z0, z1, v0, v1);

        // first one is position
        Polynomial<double> new_z(new_z_coeffs);
        last_knots(2,0) = new_z;
        swing_BMD.trajectory.setPolynomialMatrixBlock(last_knots, last_idx);
      }

      // tare the FT sensor during swing if we haven't already
      if ((plan_time >= planned_contact_swith_time - 0.5 * p_ss_duration_) && !have_tared_swing_leg_ft_){
        this->TareSwingLegForceTorque();
      }

      // check for touch down only after half swing
      // if we are in contact switch to the double support contact state and
      // plan/re-plan all trajectories (i.e. zmp, body motion, foot swing etc.)
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

  // apply the trq alpha filter at contact switch
  bool apply_torque_alpha_filter = (plan_time < p_initial_transition_time_) || (cur_time - contact_switch_time_ < p_initial_transition_time_);

  // make qp input
  drake::lcmt_qp_controller_input qp_input = MakeDefaultQPInput(cur_time, plan_time, "walking", apply_torque_alpha_filter);

  // late touch down logic
  if (late_touchdown)
    qp_input.zmp_data = zmp_planner_.EncodeZMPData(planned_contact_swith_time);

  return qp_input;
}


