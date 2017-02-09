#include "walking_plan.h"
#include "generate_spline.h"
#include "time.h"
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

Eigen::Isometry3d WalkingPlan::FootstepMsgToPose(drc::footstep_t msg){
  Eigen::Isometry3d footstep;
  footstep.translation() = Eigen::Vector3d(msg.pos.translation.x, msg.pos.translation.y, msg.pos.translation.z);
  Eigen::Quaterniond rot(msg.pos.rotation.w, msg.pos.rotation.x, msg.pos.rotation.y, msg.pos.rotation.z);
  footstep.linear() = Eigen::Matrix3d(rot.normalized());

  return footstep;
}


Eigen::Vector2d WalkingPlan::Footstep2DesiredZMP(Side side, const Eigen::Isometry3d &step) const {

  // figure out foot contact offset and contact points
  double zmp_shift_in;
  ContactState::ContactBody contact_body;
  if (side == Side::LEFT){
    zmp_shift_in = generic_plan_config_.walking_plan_config.left_foot_zmp_y_shift;
    contact_body = ContactState::ContactBody::L_FOOT;
  }
  else{
    zmp_shift_in = generic_plan_config_.walking_plan_config.right_foot_zmp_y_shift;
    contact_body = ContactState::ContactBody::R_FOOT;
  }

  const Eigen::Matrix3Xd &offsets = generic_plan_config_.contact_point_data.at(contact_body)->getAllContactPoints();

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
void WalkingPlan::GenerateTrajs(double plan_time, const Eigen::VectorXd &est_q, const Eigen::VectorXd &est_qd, const ContactState &planned_cs) {
  // current state

  clock_t start_clock_time = clock();

  KinematicsCache<double> cache_est = robot_.doKinematics(est_q, est_qd);

  Eigen::Vector2d com0, comd0;
  Eigen::Vector2d zmp_d0, zmpd_d0;
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
  generic_plan_state_.body_motions.resize(4);  /// 0 is pelvis, 1 is stance foot, 2 is swing foot

  if (planned_cs.is_double_support() && walking_plan_state_.footstep_plan.empty()) {
    throw std::runtime_error("Should not call generate trajectory from rest with empty steps");
  }

  Side nxt_stance_foot;
  Side nxt_swing_foot;
  Eigen::Isometry3d nxt_stance_foot_pose;
  Eigen::Isometry3d nxt_swing_foot_pose;

  ContactState nxt_contact_state;
  // empty queue, going to double support at the center
  if (walking_plan_state_.footstep_plan.empty()) {
    // these aren't meaningful anymore
    nxt_stance_foot = Side::LEFT;
    nxt_swing_foot = Side::RIGHT;
    nxt_contact_state = ContactState::DS();

    // if there isn't a next swing foot, i.e. this is the end of the plan, then this doesn't really matter
    // this is used in generating pelvis trajectory
    nxt_swing_foot_pose = feet_pose[nxt_swing_foot.underlying()];
  }
  else {
    // single support right
    if (walking_plan_state_.footstep_plan.front().is_right_foot) {
      nxt_stance_foot = Side::LEFT;
      nxt_swing_foot = Side::RIGHT;
      nxt_contact_state = ContactState::SSL();
    }
    else {
      nxt_stance_foot = Side::RIGHT;
      nxt_swing_foot = Side::LEFT;
      nxt_contact_state = ContactState::SSR();
    }

    nxt_swing_foot_pose = this->FootstepMsgToPose(walking_plan_state_.footstep_plan.front());
  }
  nxt_stance_foot_pose = feet_pose[nxt_stance_foot.underlying()];

  // wait before the weight shift if this is the first step
  bool is_first_step = planned_cs.is_double_support();
  double wait_period_before_weight_shift = 0;
  if (is_first_step)
    wait_period_before_weight_shift = 2;

  // make zmp
  // this determines how many steps we should be looking ahead
  int num_look_ahead = 3;

  // figure out the desired zmps
  // presumably these are just the center of the feet, maybe with a bit of offsets hacked in
  std::vector<Eigen::Vector2d> desired_zmps;



  // the foot that we want to shift weight to
  if (walking_plan_state_.footstep_plan.empty()) {


    // note there is special logic inside planZMP to detect if it's the final footstep or not
    // figure out which foot we just put down
    if (planned_cs.is_single_support_left())
      desired_zmps.push_back(Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT]));
    else if (planned_cs.is_single_support_right())
      desired_zmps.push_back(Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]));
    else
      throw std::runtime_error("robot flying.");
  }
  else {
    desired_zmps.push_back(Footstep2DesiredZMP(nxt_stance_foot, feet_pose[nxt_stance_foot.underlying()]));
  }

  for (const auto &msg : walking_plan_state_.footstep_plan) {
    Eigen::Isometry3d footstep;
    footstep.translation() = Eigen::Vector3d(msg.pos.translation.x, msg.pos.translation.y, msg.pos.translation.z);
    Eigen::Quaterniond rot(msg.pos.rotation.w, msg.pos.rotation.x, msg.pos.rotation.y, msg.pos.rotation.z);
    footstep.linear() = Eigen::Matrix3d(rot.normalized());
    Side side = msg.is_right_foot ? Side::RIGHT : Side::LEFT;

    desired_zmps.push_back(Footstep2DesiredZMP(side, footstep));
  }

  // figure out where we want the desired zmp for NOW
  // current contact state is double support
  if (planned_cs.is_double_support()) {
    zmp_d0 = (Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]) + Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT])) / 2.;
    zmpd_d0.setZero();
  }
  // single support left
  else if (planned_cs.is_single_support_left()) {
    //zmp_d0 = Footstep2DesiredZMP(Side::LEFT, feet_pose[Side::LEFT]);
    //zmpd_d0.setZero();
    zmp_d0 = zmp_planner_.GetDesiredZMP(plan_time);
    zmpd_d0 = zmp_planner_.GetDesiredZMPd(plan_time);
  }
  // single support right
  else if (planned_cs.is_single_support_right()) {
    //zmp_d0 = Footstep2DesiredZMP(Side::RIGHT, feet_pose[Side::RIGHT]);
    //zmpd_d0.setZero();
    zmp_d0 = zmp_planner_.GetDesiredZMP(plan_time);
    zmpd_d0 = zmp_planner_.GetDesiredZMPd(plan_time);
  }
  else {
    throw std::runtime_error("robot flying.");
  }


  // plan the zmp trajectory
  generic_plan_state_.zmp_traj = PlanZMPTraj(desired_zmps, num_look_ahead, zmp_d0, zmpd_d0, wait_period_before_weight_shift);

  // solve the LQR problem for tracking the generic_plan_state_.zmp_traj
  // note, what we are controlling is the pelvis height, this indirectly affects the zmp_height
  // ensure that in the config file the zmp_height and pelvis_height are consistent.
  // Essentially zmp_height should correspond to com height when pelvis is at given location
  // this is not quite right, since we are controlling the pelvis height, rather than com height
  zmp_planner_.Plan(generic_plan_state_.zmp_traj, x0, generic_plan_config_.zmp_height);

  /*
  // Save zmp trajs to a files
  static int step_ctr = 0;
  std::string file_name = std::string("/home/siyuanfeng/zmp_d") + std::to_string(step_ctr);
  step_ctr++;
  zmp_planner_.WriteToFile(file_name, 0.01);
  */

  // time tape for body motion data and joint trajectories
  // This function is only getting called when we make contact (or in the initial timestep)
  // These tapes last until the next touchdown event
  std::vector<double> Ts(2);
  Ts[0] = 0;
  Ts[1] = wait_period_before_weight_shift + generic_plan_config_.walking_plan_config.ss_duration + generic_plan_config_.walking_plan_config.ds_duration;


  // for pelvis stuff should add another knot point at time = wait_period_before_weight_shift + generic_plan_config_.walking_plan_config.ss_duration (this is just before liftoff)

  /// Initialize body motion data
  for (size_t i = 0; i < generic_plan_state_.body_motions.size(); i++)
    generic_plan_state_.body_motions[i] = MakeDefaultBodyMotionData(Ts.size());

  // hold all joints. I am assuming the leg joints will just be ignored in the qp..
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(est_q.size());
  q_trajs_ = GenerateCubicSpline(Ts, std::vector<Eigen::VectorXd>(Ts.size(), init_q_), zero, zero);

//  // make pelv
//  pelv1.head(2) = generic_plan_state_.zmp_traj.value(generic_plan_state_.zmp_traj.getEndTime());
//
//  ///< This offset the pelv z relative to foot frame
//  pelv1[2] = generic_plan_config_.walking_plan_config.pelvis_height + feet_pose[nxt_stance_foot.underlying()].translation()[2];
//  Eigen::Vector4d foot_quat(feet0[nxt_stance_foot.underlying()].tail(4));
//  double yaw = quat2rpy(foot_quat)[2];
//  pelv1.tail(4) = rpy2quat(Eigen::Vector3d(0, 0, yaw));
//
//  std::vector<Eigen::Vector7d> pelv_knots(Ts.size());
//  pelv_knots[0] = pelv0;
//  pelv_knots[1] = pelv1;
  BodyMotionData &pelvis_BMD = get_pelvis_body_motion_data();
  pelvis_BMD.body_or_frame_id = rpc_.pelvis_id;

  int num_knots_in_pelvis_traj = 3;
  pelvis_BMD.control_pose_when_in_contact.resize(num_knots_in_pelvis_traj, true);
  double liftoff_time = wait_period_before_weight_shift + generic_plan_config_.walking_plan_config.ss_duration;
  double next_liftoff_time = wait_period_before_weight_shift + generic_plan_config_.walking_plan_config.ss_duration + generic_plan_config_.walking_plan_config.ds_duration;
  pelvis_BMD.trajectory = this->GeneratePelvisTraj(cache_est, generic_plan_config_.walking_plan_config.pelvis_height, liftoff_time, next_liftoff_time,
  nxt_stance_foot_pose, nxt_swing_foot_pose);

  // make torso
  Eigen::Vector4d foot_quat(feet0[nxt_stance_foot.underlying()].tail(4));
  double yaw = quat2rpy(foot_quat)[2];
  torso1 = torso0; // this is just so that plotting looks reasonable
  Eigen::AngleAxisd torso_rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()); // yaw

  // not sure what this line is doing
  torso_rot = torso_rot * Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()); // pitch, set to zero currently
  torso1.tail(4) = rotmat2quat(torso_rot.toRotationMatrix());
  std::vector<Eigen::Vector7d> torso_knots(Ts.size());
  torso_knots[0] = torso0;
  torso_knots[1] = torso1;
  BodyMotionData &torso_BMD = get_torso_body_motion_data();
  torso_BMD.body_or_frame_id = rpc_.torso_id;
  torso_BMD.trajectory = GenerateCubicCartesianSpline(Ts, torso_knots, std::vector<Eigen::Vector7d>(Ts.size(), Eigen::Vector7d::Zero()));

  // make stance foot.
  // This is not quite right, since the touchdown foot pose might change after solid contact.
  // Assume the QP will not try to control pose for the contact feet.
  std::vector<Eigen::Vector7d> stance_knots(Ts.size(), feet0[nxt_stance_foot.underlying()]);
  BodyMotionData &stance_BMD = get_stance_foot_body_motion_data();
  stance_BMD.body_or_frame_id = rpc_.foot_ids.at(nxt_stance_foot);
  stance_BMD.trajectory = GenerateCubicCartesianSpline(Ts, stance_knots, std::vector<Eigen::Vector7d>(Ts.size(), Eigen::Vector7d::Zero()));

  // swing foot traj will be planned at liftoff . . .
  std::vector<Eigen::Vector7d> swing_knots(Ts.size(), feet0[nxt_swing_foot.underlying()]);
  BodyMotionData &swing_BMD = get_swing_foot_body_motion_data();
  swing_BMD.body_or_frame_id = rpc_.foot_ids.at(nxt_swing_foot);
  swing_BMD.trajectory = GenerateCubicCartesianSpline(Ts, swing_knots, std::vector<Eigen::Vector7d>(Ts.size(), Eigen::Vector7d::Zero()));

  // setup contact state
  walking_plan_state_.contact_state.clear();

  // 0: start of weight tranfer
  // 1: end of weight transfer
  // 2: swing phase
  Ts.resize(3);
  Ts[0] = 0;
  Ts[1] = wait_period_before_weight_shift + generic_plan_config_.walking_plan_config.ds_duration;
  Ts[2] = wait_period_before_weight_shift + generic_plan_config_.walking_plan_config.ss_duration + generic_plan_config_.walking_plan_config.ds_duration;
  std::cout << "lift off time " << Ts[1] << std::endl;
  // run out of step, set transition tape's time to inf
  if (walking_plan_state_.footstep_plan.empty()) {
    walking_plan_state_.contact_state.push_back(std::pair<ContactState, double>(nxt_contact_state, INFINITY));
  }
  else {
    walking_plan_state_.contact_state.push_back(std::pair<ContactState, double>(ContactState::DS(), Ts[1]));
    walking_plan_state_.contact_state.push_back(std::pair<ContactState, double>(nxt_contact_state, Ts[2]));
  }

  // make weight distribuition
  std::vector<Eigen::Matrix<double,1,1>> WL(Ts.size());
  WL[0](0,0) = get_weight_distribution(planned_cs);
  WL[1](0,0) = get_weight_distribution(nxt_contact_state);
  WL[2](0,0) = get_weight_distribution(nxt_contact_state);
  Ts[0] = wait_period_before_weight_shift;
  weight_distribution_ = GenerateCubicSpline(Ts, WL);

  // pelvis Z weight multiplier
  // This is really dumb, but the multiplier is ang then pos, check instQP.
  get_pelvis_body_motion_data().weight_multiplier[5] = generic_plan_config_.walking_plan_config.pelvis_z_weight_multiplier;
  // don't track x and y position of the pelvis
  get_pelvis_body_motion_data().weight_multiplier[4] = 0;
  get_pelvis_body_motion_data().weight_multiplier[3] = 0;

  // don't track xyz of the torso
  for (int i = 3; i < 6; i++)
    get_torso_body_motion_data().weight_multiplier[i] = 0;

  // reset clock
  generic_plan_state_.plan_start_time = -1; // what does this do exactly?


  float elapsedTimeInSeconds = ((float) (clock() - start_clock_time))/CLOCKS_PER_SEC;
  std::cout << "Generate Trajs took " << elapsedTimeInSeconds << " seconds \n";
}


// This is called whenever the foot lifts off . . .
PiecewisePolynomial<double> WalkingPlan::GenerateSwingTraj(const Eigen::Vector7d &foot0, const Eigen::Vector7d &foot1, double mid_z_offset, double pre_swing_dur, double swing_up_dur, double swing_transfer_dur, double swing_down_dur) const {
  double z_height_mid_swing = std::max(foot1[2], foot0[2]) + mid_z_offset;
  int num_T = 5;
  std::vector<double> swingT(num_T);
  swingT[0] = 0;
  swingT[1] = pre_swing_dur;
  swingT[2] = pre_swing_dur + swing_up_dur;
  swingT[3] = pre_swing_dur + swing_up_dur + swing_transfer_dur;
  swingT[4] = pre_swing_dur + swing_up_dur + swing_transfer_dur + swing_down_dur;

  std::vector<Eigen::Vector7d> swing_pose_knots(num_T);
  std::vector<Eigen::Vector7d> swing_vel_knots(num_T, Eigen::Vector7d::Zero());
  swing_pose_knots[0] = swing_pose_knots[1] = foot0;

  // xy will be set later
  swing_pose_knots[2] = foot1;
  swing_pose_knots[2][2] = z_height_mid_swing;
  // right above the touchdown pose, only increase z
  swing_pose_knots[3] = foot1;
  swing_pose_knots[3][2] = z_height_mid_swing;
  swing_pose_knots[4] = foot1;

  // helper method to figure out x,y velocity at midpoint
  std::vector<double> T_xy = {0, swing_up_dur + swing_transfer_dur};
  std::vector<Eigen::Vector2d> xy_pos = {foot0.head(2) , foot1.head(2)};
  std::vector<Eigen::Vector2d> xy_vel(2, Eigen::Vector2d::Zero());

  auto xy_spline = GenerateCubicSpline(T_xy, xy_pos, xy_vel);
  auto xy_spline_deriv = xy_spline.derivative();

  swing_pose_knots[2].head(2) = xy_spline.value(swing_up_dur);
  swing_vel_knots[2].head(2) = xy_spline_deriv.value(swing_up_dur);

  // end velocity specification for spline

  swing_vel_knots[4][2] = generic_plan_config_.walking_plan_config.swing_foot_touchdown_z_vel;

  return GenerateCubicCartesianSpline(swingT, swing_pose_knots, swing_vel_knots);
}

void WalkingPlan::HandleCommittedRobotPlan(const void *plan_msg,
                                           const DrakeRobotState &est_rs,
                                           const Eigen::VectorXd &last_q_d) {


  // state machine
  walking_plan_state_.walking_state = WEIGHT_TRANSFER;

  walking_plan_state_.footstep_plan.clear();
  walking_plan_state_.contact_state.clear();

  const drc::walking_plan_request_t *msg = (const drc::walking_plan_request_t *)plan_msg;
  if (msg->footstep_plan.footsteps.size() <= 2)
    throw std::runtime_error("too few number of steps.");

  // remove the first two footsteps as they are just in the current footstep locations
  for (size_t i = 2; i < msg->footstep_plan.footsteps.size(); i++)
    walking_plan_state_.footstep_plan.push_back(msg->footstep_plan.footsteps[i]);

  // save the current joint configuration, that's what we are tracking during walking
  init_q_ = est_rs.q;

  // generate all trajs
  GenerateTrajs(0, est_rs.q, est_rs.qd, ContactState::DS());

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

  // right now only support constraining the bky and bkz, not bkx
  if(generic_plan_config_.walking_plan_config.constrain_back_bky){
    constrained_dofs_.push_back(rpc_.position_indices.back_bky);
  }
  if(generic_plan_config_.walking_plan_config.constrain_back_bkz){
    constrained_dofs_.push_back(rpc_.position_indices.back_bkz);
  }
}

void WalkingPlan::SwitchContactState(double cur_time) {
  walking_plan_state_.contact_state.pop_front();
  walking_plan_state_.contact_switch_time = cur_time;
  //reset the tare FT flag
  walking_plan_state_.have_tared_swing_leg_ft = false;
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
  walking_plan_state_.have_tared_swing_leg_ft = true;
}

PiecewisePolynomial<double> WalkingPlan::PlanZMPTraj(const std::vector<Eigen::Vector2d> &zmp_d, int num_of_zmp_knots, const Eigen::Vector2d &zmp_d0, const Eigen::Vector2d &zmpd_d0, double time_before_first_weight_shift) const {
  if (zmp_d.size() < 1)
    throw std::runtime_error("zmp_d traj must have size >= 1");

  std::vector<double> zmp_T; // double support weight transfer, followed by single support
  std::vector<Eigen::Vector2d> zmp_knots;

  int min_size = std::min(num_of_zmp_knots, (int)zmp_d.size());

  double cur_time = 0;
  zmp_knots.push_back(zmp_d0);
  zmp_T.push_back(cur_time);

  if (time_before_first_weight_shift > 0) {
    cur_time += time_before_first_weight_shift;
    // hold current place for sometime for the first step
    zmp_knots.push_back(zmp_d0);
    zmp_T.push_back(cur_time);
  }

  for (int i = 0; i < min_size; i++) {
    // add a double support phase
    cur_time += generic_plan_config_.walking_plan_config.ds_duration;
    // last step, need to stop in the middle
    if (i == min_size - 1) {
      zmp_knots.push_back((zmp_knots.back() + zmp_d[i]) / 2.);
      zmp_T.push_back(cur_time);
    }
    else {
      zmp_knots.push_back(zmp_d[i]);
      zmp_T.push_back(cur_time);
    }

    // add a single support phase
    cur_time += generic_plan_config_.walking_plan_config.ss_duration;
    zmp_knots.push_back(zmp_knots.back());
    zmp_T.push_back(cur_time);
  }

  Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  return GeneratePCHIPSpline(zmp_T, zmp_knots, zmpd_d0, zero);
}

drake::lcmt_qp_controller_input WalkingPlan::MakeQPInput(const DrakeRobotState &est_rs) {
  static double last_time = 0;
  double cur_time = est_rs.t;
  double dt = std::min(cur_time - last_time, 1.0);
  last_time = cur_time;

  if (generic_plan_state_.plan_start_time == -1)
    generic_plan_state_.plan_start_time = cur_time;

  double plan_time = cur_time - generic_plan_state_.plan_start_time;

  // update the plan status depending on whether or not we are finished
  if (walking_plan_state_.footstep_plan.empty() && (plan_time > generic_plan_config_.walking_plan_config.ds_duration)) {
    generic_plan_state_.plan_status.executionStatus = PlanExecutionStatus::FINISHED;
  } else{
    generic_plan_state_.plan_status.executionStatus = PlanExecutionStatus::EXECUTING;
  }

  // can't use reference for this one because ref becomes invalid when queue pops
  const ContactState planned_cs = cur_planned_contact_state();
  const ContactState &est_cs = est_rs.contact_state;
  double planned_contact_swith_time = cur_planned_contact_swith_time();

  bool late_touchdown = false;

  drc::footstep_t &cur_step = walking_plan_state_.footstep_plan.front();
  Side swing_foot = cur_step.is_right_foot ? Side::RIGHT : Side::LEFT;

  // state machine part
  switch (walking_plan_state_.walking_state) {
    case WEIGHT_TRANSFER:
      if (plan_time >= planned_contact_swith_time) {
        // replace the swing up segment with a new one that starts from the current foot pose.
        KinematicsCache<double> cache = robot_.doKinematics(est_rs.q, est_rs.qd);
        Eigen::Isometry3d swing_foot0 = robot_.relativeTransform(cache, 0, rpc_.foot_ids.at(swing_foot));

        Eigen::Vector7d swing_touchdown_pose = bot_core_pose2pose(cur_step.pos);


        // TODO: THIS IS A HACK
        swing_touchdown_pose[2] += generic_plan_config_.walking_plan_config.swing_foot_touchdown_z_offset;

        BodyMotionData& swing_BMD = get_swing_foot_body_motion_data();
        swing_BMD.body_or_frame_id = rpc_.foot_ids.at(swing_foot);
        swing_BMD.trajectory = GenerateSwingTraj(Isometry3dToVector7d(swing_foot0), swing_touchdown_pose, cur_step.params.step_height, plan_time, generic_plan_config_.walking_plan_config.ss_duration / 3., generic_plan_config_.walking_plan_config.ss_duration / 3., generic_plan_config_.walking_plan_config.ss_duration / 3.);
        // increase weights for swing foot xy tracking
        swing_BMD.weight_multiplier[3] = generic_plan_config_.walking_plan_config.swing_foot_xy_weight_multiplier;
        swing_BMD.weight_multiplier[4] = generic_plan_config_.walking_plan_config.swing_foot_xy_weight_multiplier;
        swing_BMD.weight_multiplier[5] = generic_plan_config_.walking_plan_config.swing_foot_z_weight_multiplier;

        // change contact
        SwitchContactState(cur_time);
        walking_plan_state_.walking_state = SWING;
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
        double z1 = swing_BMD.trajectory.value(t1)(2,0) + generic_plan_config_.walking_plan_config.extend_foot_down_z_vel * dt;
        double v1 = generic_plan_config_.walking_plan_config.extend_foot_down_z_vel;
        //Eigen::Vector4d new_z_coeffs = GetCubicSplineCoeffs(t1-t0, z0, z1, v0, v1);
        Eigen::Vector4d new_z_coeffs(z0 + (z1 - z0 - v1 * (t1 -t0)), v1, 0, 0);

        // first one is position
        Polynomial<double> new_z(new_z_coeffs);
        last_knots(2,0) = new_z;
        swing_BMD.trajectory.setPolynomialMatrixBlock(last_knots, last_idx);
        swing_BMD.trajectory.setEndTime(100000.);
      }

      // tare the FT sensor during swing if we haven't already
      if ((plan_time >= planned_contact_swith_time - 0.5 * generic_plan_config_.walking_plan_config.ss_duration) && !walking_plan_state_.have_tared_swing_leg_ft) {
        this->TareSwingLegForceTorque();
      }

      // check for touch down only after half swing
      // if we are in contact switch to the double support contact state and
      // plan/re-plan all trajectories (i.e. zmp, body motion, foot swing etc.)
      if (plan_time >= planned_contact_swith_time - 0.5 * generic_plan_config_.walking_plan_config.ss_duration &&
          est_cs.is_foot_in_contact(swing_foot)) {
        // change contact
        SwitchContactState(cur_time);
        walking_plan_state_.walking_state = WEIGHT_TRANSFER;
        std::cout << "Swing -> Weight transfer @ " << plan_time << std::endl;

        // dequeue foot steps
        walking_plan_state_.footstep_plan.pop_front(); // this could potentially be empty after this pop if it's the last step
        walking_plan_state_.step_count++;
        // generate new trajectories
        GenerateTrajs(plan_time, est_rs.q, est_rs.qd, planned_cs);

        // all tapes assumes 0 sec start, so need to reset clock
        generic_plan_state_.plan_start_time = cur_time;
        plan_time = cur_time - generic_plan_state_.plan_start_time;
        late_touchdown = false;
      }
      break;

    default:
      throw std::runtime_error("Unknown walking state");
  }

  // make contact support data
  generic_plan_state_.support_state = MakeDefaultSupportState(cur_planned_contact_state());
  // interp weight distribution
  double wl = weight_distribution_.value(plan_time).value();
  wl = std::min(wl, 1.0);
  wl = std::max(wl, 0.0);

  for (size_t s = 0; s < generic_plan_state_.support_state.size(); s++) {

    if (generic_plan_config_.walking_plan_config.use_force_bounds){
      // only foot
      if (generic_plan_state_.support_state[s].body == rpc_.foot_ids.at(Side::LEFT)) {
        // in making the default support state this is set to 3 times the robot's mass
        generic_plan_state_.support_state[s].total_normal_force_upper_bound *= wl;
        generic_plan_state_.support_state[s].total_normal_force_lower_bound = generic_plan_config_.walking_plan_config.min_Fz;
      }
      else if (generic_plan_state_.support_state[s].body == rpc_.foot_ids.at(Side::RIGHT)) {
        generic_plan_state_.support_state[s].total_normal_force_upper_bound *= (1 - wl);
        generic_plan_state_.support_state[s].total_normal_force_lower_bound = generic_plan_config_.walking_plan_config.min_Fz;
      }

      generic_plan_state_.support_state[s].total_normal_force_upper_bound = std::max(generic_plan_state_.support_state[s].total_normal_force_upper_bound, generic_plan_state_.support_state[s].total_normal_force_lower_bound);
    } else{
      generic_plan_state_.support_state[s].total_normal_force_lower_bound = 0;
      // the upper bound is already set to 3 times the robot's weight in the MakeDefaultSupportState method
    }

  }

  // apply the trq alpha filter at contact switch
  bool apply_torque_alpha_filter = false; // (plan_time < generic_plan_config_.initial_transition_time) || (cur_time - walking_plan_state_.contact_switch_time < generic_plan_config_.initial_transition_time);

  // make qp input
  drake::lcmt_qp_controller_input qp_input = MakeDefaultQPInput(cur_time, plan_time, "walking", apply_torque_alpha_filter);

  // late touch down logic
  //if (late_touchdown)
  //  qp_input.zmp_data = zmp_planner_.EncodeZMPData(planned_contact_swith_time);

  // record some debug data
  this->RecordDefaultDebugData(plan_time);
  debug_data_.plan_type = "walking";

  return qp_input;
}


PiecewisePolynomial<double> WalkingPlan::GeneratePelvisTraj(KinematicsCache<double> cache,
                                                            double &pelvis_height_above_sole, double &liftoff_time,
                                                            double &next_liftoff_time,
                                                            Eigen::Isometry3d nxt_stance_foot_pose,
                                                            Eigen::Isometry3d nxt_swing_foot_pose) {

  std::vector<double> Ts(3);
  Ts[0] = 0;
  Ts[1] = liftoff_time;
  Ts[2] = next_liftoff_time;

  // there will be three knot points
  std::vector<Eigen::Matrix<double,7,1>> pelvis_pose_vec(3);
  pelvis_pose_vec[0] = Isometry3dToVector7d(robot_.relativeTransform(cache, 0, rpc_.pelvis_id));

  // need to adjust z-height on poses 1 and 2;
  pelvis_pose_vec[1][2] = nxt_stance_foot_pose.translation()[2] + pelvis_height_above_sole;
  Eigen::Vector4d foot_quat1(Isometry3dToVector7d(nxt_stance_foot_pose).tail(4));
  double yaw1 = quat2rpy(foot_quat1)[2];
  pelvis_pose_vec[1].tail(4) = rpy2quat(Eigen::Vector3d(0,0,yaw1));


  pelvis_pose_vec[2][2] = nxt_swing_foot_pose.translation()[2] + pelvis_height_above_sole;
  Eigen::Vector4d foot_quat2(Isometry3dToVector7d(nxt_swing_foot_pose).tail(4));
  double yaw2 = quat2rpy(foot_quat2)[2];
  pelvis_pose_vec[2].tail(4) = rpy2quat(Eigen::Vector3d(0,0,yaw2));

  // need to create velocities, should be zero everywhere
  std::vector<Eigen::Vector7d> velocities = std::vector<Eigen::Vector7d>(Ts.size(), Eigen::Vector7d::Zero());
  PiecewisePolynomial<double> pelvisTraj = GenerateCubicCartesianSpline(Ts, pelvis_pose_vec, velocities);



  // some debugging stuff
  std::cout << "pelvis heights " << pelvis_pose_vec[0][2] << " " << pelvis_pose_vec[1][2] << " " << pelvis_pose_vec[2][2] << std::endl;
//  std::cout << "Ts " << Ts << std::endl;
  std::cout << "next stance foot height " << nxt_stance_foot_pose.translation()[2] << std::endl;


  return pelvisTraj;
}
