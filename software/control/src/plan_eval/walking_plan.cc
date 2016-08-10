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


void WalkingPlan::LoadConfigurationFromYAML(const std::string &name) {
  p_ss_duration_ = config_["ss_duration"].as<double>();
  p_ds_duration_ = config_["ds_duration"].as<double>();

  std::cout << "p_ss_duration_: " << p_ss_duration_ << std::endl;
  std::cout << "p_ds_duration_: " << p_ds_duration_ << std::endl;
}

// called on every touch down
void WalkingPlan::GenerateTrajs(const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd, ContactState cur_contact_state) {
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
  ContactState nxt_contact_state;
  // cur_contact_state = DSc;
  // if (!contact_state_.empty())
  //   cur_contact_state = contact_state_.front();

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
    // TODO: probably need to change val
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
    WL[0](0,0) = get_weight_distribution(cur_contact_state);
    WL[1](0,0) = get_weight_distribution(DSc);
    weight_distribution_ = GenerateLinearSpline(Ts, WL);
  }
  else {
    int num_T = 3;
    for (int i = 0; i < 3; i++)
      body_motions_[i] = MakeDefaultBodyMotionData(num_T);

    Side stance_foot = Side::LEFT;
    Side swing_foot = Side::RIGHT;
    drc::footstep_t &cur_step = footstep_plan_.front();
    nxt_contact_state = SSL;
    // ssr
    if (!cur_step.is_right_foot) {
      stance_foot = Side::RIGHT;
      swing_foot = Side::LEFT;
      nxt_contact_state = SSR;
    }

    // TODO: not right
    //double p_ss_duration_ = cur_step.params.step_speed;
    //double p_ds_duration_ = cur_step.params.drake_min_hold_time;

    // time tape
    std::vector<double> Ts(num_T);
    Ts[0] = 0;
    Ts[1] = p_ds_duration_;
    Ts[2] = p_ss_duration_ + p_ds_duration_;

    // hold arm joints, I am assuming the leg joints will just be ignored..
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(est_q.size());
    q_trajs_ = GenerateCubicSpline(Ts, std::vector<Eigen::VectorXd>(num_T, est_q), zero, zero);

    // com / cop
    double zmp_shift_in = cur_step.params.drake_instep_shift;
    if (stance_foot == Side::LEFT)
      zmp_shift_in = -zmp_shift_in;
    std::cout << stance_foot.underlying() << " " << zmp_shift_in << std::endl;

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
    // TODO: probably need to change val
    body_motions_[0].trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make stance foot
    std::vector<Eigen::Vector7d> stance_knots(num_T, feet0[stance_foot.underlying()]);
    body_motions_[1].body_or_frame_id = rpc_.foot_ids.at(stance_foot);
    body_motions_[1].trajectory = GenerateCubicCartesianSpline(Ts, stance_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

    // make swing foot
    Eigen::Vector7d swing_touchdown_pose;
    swing_touchdown_pose(0) = cur_step.pos.translation.x;
    swing_touchdown_pose(1) = cur_step.pos.translation.y;
    swing_touchdown_pose(2) = cur_step.pos.translation.z - 0.02;
    swing_touchdown_pose(3) = cur_step.pos.rotation.w;
    swing_touchdown_pose(4) = cur_step.pos.rotation.x;
    swing_touchdown_pose(5) = cur_step.pos.rotation.y;
    swing_touchdown_pose(6) = cur_step.pos.rotation.z;
    swing_touchdown_pose.tail(4).normalize();

    body_motions_[2].body_or_frame_id = rpc_.foot_ids.at(swing_foot);
    body_motions_[2].trajectory = GenerateSwingTraj(feet0[swing_foot.underlying()], swing_touchdown_pose, cur_step.params.step_height, p_ds_duration_, p_ss_duration_ / 2., p_ss_duration_ / 2.);


    // make weight distribuition
    std::vector<Eigen::Matrix<double,1,1>> WL(num_T);
    WL[0](0,0) = get_weight_distribution(cur_contact_state);
    WL[1](0,0) = get_weight_distribution(nxt_contact_state);
    WL[2](0,0) = get_weight_distribution(nxt_contact_state);
    weight_distribution_ = GenerateLinearSpline(Ts, WL);
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

void WalkingPlan::SetupContactStates() {
  contact_state_.clear();
  contact_switching_time_.clear();

  // no more steps
  if (footstep_plan_.empty()) {
    contact_switching_time_.push_back(INFINITY);
    contact_state_.push_back(DSc);
  }
  else {
    // weight transfer
    contact_state_.push_back(DSc);
    contact_switching_time_.push_back(p_ds_duration_);

    // ssl
    if (footstep_plan_.front().is_right_foot)
      contact_state_.push_back(SSL);
    else
      contact_state_.push_back(SSR);

    // TODO: relying on detecting real contact switching from state est??
    //contact_switching_time_.push_back(INFINITY);
    contact_switching_time_.push_back(p_ss_duration_ + p_ds_duration_);
  }
}

// we will hack something up for now.
void WalkingPlan::HandleCommittedRobotPlan(const void *plan_msg,
                                           const DrakeRobotState &est_rs,
                                           const Eigen::VectorXd &last_q_d) {
  footstep_plan_.clear();
  contact_state_.clear();
  contact_switching_time_.clear();

  const drc::walking_plan_request_t *msg = (const drc::walking_plan_request_t *)plan_msg;
  for (size_t i = 0; i < msg->footstep_plan.footsteps.size(); i++) {
    footstep_plan_.push_back(msg->footstep_plan.footsteps[i]);
  }

  // generate all cart trajs
  GenerateTrajs(est_rs.q, est_rs.qd, DSc);

  // generate contact switching tape
  SetupContactStates();
}

void WalkingPlan::switchContactState(double cur_time){
  contact_state_.pop_front();
  contact_switching_time_.pop_front();
  contact_switch_time_ = cur_time;
}


// tells us whether we are in single support or not
bool WalkingPlan::inSingleSupport(){
  ContactState currentContactState = contact_state_.front();
  if ((currentContactState == ContactState::SSL) || (currentContactState == ContactState::SSR) ){
    return true;
  } else {
    return false;
  }

}

Side WalkingPlan::getSwingFoot(ContactState currentContactState){
  if (currentContactState == ContactState::SSL){
    return Side::RIGHT;
  } else if (currentContactState == ContactState::SSR){
    return Side::LEFT;
  } else{
    std::cout << "called it with double support, returning LEFT by default " << std::endl;
    return Side::LEFT;
  }
}


drake::lcmt_qp_controller_input WalkingPlan::MakeQPInput(const DrakeRobotState &est_rs) {
  double cur_time = est_rs.t;

  if (interp_t0_ == -1)
    interp_t0_ = cur_time;
  double plan_time = cur_time - interp_t0_;

  assert(!contact_state_.empty());
  ContactState last_contact_state = contact_state_.front();

  // switch contact state tape
  if (plan_time >= contact_switching_time_.front()) {
    contact_state_.pop_front();
    contact_switching_time_.pop_front();
    contact_switch_time_ = cur_time;
  }

  // early contact logic
  double early_contact_threshold = 0.5;
  if ((plan_time >= contact_switching_time_.front() - early_contact_threshold) || this->inSingleSupport()){
    // check that we are currently in single support


  }

  // touch down. need to make new tapes
  if (contact_state_.empty()) {
    // dequeue foot steps
    footstep_plan_.pop_front();
    GenerateTrajs(est_rs.q, est_rs.qd, last_contact_state);
    SetupContactStates();

    // all tapes assumes 0 sec start, so need to reset clock
    interp_t0_ = cur_time;
    plan_time = cur_time - interp_t0_;
  }

  // make contact support data
  support_state_ = MakeDefaultSupportState(contact_state_.front());
  // interp weight distribution
  double wl = weight_distribution_.value(plan_time).value();
  wl = std::min(wl, 1.0);
  wl = std::max(wl, 0.0);
  for (size_t s = 0; s < support_state_.size(); s++) {
    if (support_state_[s].side == Side::LEFT) {
      support_state_[s].total_normal_force_upper_bound *= wl;
      support_state_[s].total_normal_force_lower_bound *= wl;
    }
    else {
      support_state_[s].total_normal_force_upper_bound *= (1 - wl);
      support_state_[s].total_normal_force_lower_bound *= (1 - wl);
    }
  }

  ////////////////////////////////////////
  drake::lcmt_qp_controller_input qp_input;
  qp_input.be_silent = false;
  qp_input.timestamp = static_cast<int64_t>(cur_time * 1e6);
  qp_input.param_set_name = "walking";

  ////////////////////////////////////////
  // no body wrench data
  qp_input.num_external_wrenches = 0;

  ////////////////////////////////////////
  // no joint pd override
  qp_input.num_joint_pd_overrides = 0;

  ////////////////////////////////////////
  // make whole_body_data
  auto q_des = q_trajs_.value(plan_time);
  std::vector<float> &q_des_std_vector = qp_input.whole_body_data.q_des;
  q_des_std_vector.resize(q_des.size());
  for (int i = 0; i < q_des.size(); i++) {
    q_des_std_vector[i] = static_cast<float>(q_des(i));
  }
  int qtrajSegmentIdx = q_trajs_.getSegmentIndex(plan_time);
  int num_segments =
      std::min(2, q_trajs_.getNumberOfSegments() - qtrajSegmentIdx);
  PiecewisePolynomial<double> qtrajSlice =
      q_trajs_.slice(qtrajSegmentIdx, num_segments);
  qtrajSlice.shiftRight(interp_t0_);

  encodePiecewisePolynomial(qtrajSlice, qp_input.whole_body_data.spline);

  qp_input.whole_body_data.timestamp = 0;
  qp_input.whole_body_data.num_positions = robot_.num_positions;

  // constrained DOFs
  // arms
  for (auto it = rpc_.position_indices.arms.begin(); it != rpc_.position_indices.arms.end(); it++) {
    const std::vector<int> &indices = it->second;
    for (size_t i = 0; i < indices.size(); i++)
      qp_input.whole_body_data.constrained_dofs.push_back(indices[i]);
  }
  // neck
  for (size_t i = 0; i < rpc_.position_indices.neck.size(); i++)
    qp_input.whole_body_data.constrained_dofs.push_back(rpc_.position_indices.neck[i]);
  // back
  qp_input.whole_body_data.constrained_dofs.push_back(rpc_.position_indices.back_bkz);
  //qp_input.whole_body_data.constrained_dofs.push_back(rpc_.position_indices.back_bky);
  // add 1 offset to match matlab indexing, for backward compatibility
  for (size_t i = 0; i < qp_input.whole_body_data.num_constrained_dofs; i++)
    qp_input.whole_body_data.constrained_dofs[i]++;

  qp_input.whole_body_data.num_constrained_dofs =
      qp_input.whole_body_data.constrained_dofs.size();

  ////////////////////////////////////////
  // encode zmp data
  qp_input.zmp_data = zmp_planner_.EncodeZMPData(plan_time);

  ////////////////////////////////////////
  // encode body motion data
  qp_input.num_tracked_bodies = body_motions_.size();
  qp_input.body_motion_data.resize(qp_input.num_tracked_bodies);
  for (size_t b = 0; b < qp_input.body_motion_data.size(); b++)
    qp_input.body_motion_data[b] = EncodeBodyMotionData(plan_time, body_motions_[b]);

  ////////////////////////////////////////
  // encode support data
  qp_input.num_support_data = support_state_.size();
  qp_input.support_data.resize(qp_input.num_support_data);
  for (size_t i = 0; i < qp_input.support_data.size(); i++)
    qp_input.support_data[i] = EncodeSupportData(support_state_[i]);


  ////////////////////////////////////////
  // torque alpha filter
  //qp_input.torque_alpha_filter = 0.;
  if (plan_time < p_initial_transition_time_ || cur_time - contact_switch_time_ < p_initial_transition_time_)
    qp_input.torque_alpha_filter = 0.9;
  else
    qp_input.torque_alpha_filter = 0.;

  return qp_input;
}


