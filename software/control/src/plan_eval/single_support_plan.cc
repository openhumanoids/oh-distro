#include "single_support_plan.h"
#include "generate_spline.h"
#include "drake/util/lcmUtil.h"

#include <fstream>
#include <iomanip>

void SingleSupportPlan::LoadConfigurationFromYAML(const std::string &name) {
  p_ss_duration_ = config_["ss_duration"].as<double>();
  p_ds_duration_ = config_["ds_duration"].as<double>();

  std::cout << "p_ss_duration_: " << p_ss_duration_ << std::endl;
  std::cout << "p_ds_duration_: " << p_ds_duration_ << std::endl;
}

// we will hack something up for now.
void SingleSupportPlan::HandleCommittedRobotPlan(const void *plan_msg,
                                                 const DrakeRobotState &est_rs,
                                                 const Eigen::VectorXd &last_q_d) {
  const drc::walking_plan_request_t *msg = (const drc::walking_plan_request_t *)plan_msg;
  // current state
  KinematicsCache<double> cache_est = robot_.doKinematics(est_rs.q, est_rs.qd);
  Eigen::Isometry3d feet_pose[2];

  int b = 0;
  for (const auto& side : Side::values) {
    int id = rpc_.foot_ids[side];
    feet_pose[b] = robot_.relativeTransform(cache_est, 0, id);
    b++;
  }

  // shift to middle left foot
  Eigen::Isometry3d com_end_d(Eigen::Isometry3d::Identity());
  com_end_d.translation() = Eigen::Vector3d(0.04, 0, 0);
  auto stance_foot = Side::RIGHT;
  auto swing_foot = Side::LEFT;
  com_end_d = feet_pose[stance_foot] * com_end_d;

  // compute end points for com / pelvis
  Eigen::Vector2d com0 = robot_.centerOfMass(cache_est).segment<2>(0);
  Eigen::Vector2d com1 = com_end_d.translation().segment<2>(0);
  Eigen::Matrix<double,7,1> pelv0 = Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, rpc_.pelvis_id));
  Eigen::Matrix<double,7,1> pelv1;
  pelv1.segment<3>(0) = com_end_d.translation();
  pelv1[2] += p_zmp_height_;
  pelv1.segment<4>(3) = rotmat2quat(feet_pose[stance_foot].linear());

  std::cout << "pelv0" << pelv0.transpose() << std::endl;
  std::cout << "pelv1" << pelv1.transpose() << std::endl;

  // make a zmp traj for ds
  int num_T = 2;
  std::vector<double> Ts(num_T);
  std::vector<Eigen::Vector2d> com_d(num_T);
  for (int i = 0; i < num_T; i++) {
    double a = (double)i / (double)(num_T - 1);
    Ts[i] = p_ds_duration_ * a;
    com_d[i] = com0 + a * (com1 - com0);
    printf("t %g %g %g\n", Ts[i], com_d[i][0], com_d[i][1]);
  }
  zmp_traj_ = GeneratePCHIPSpline(Ts, com_d);
  Eigen::Vector4d x0(Eigen::Vector4d::Zero());
  x0.head(2) = com_d[0];
  zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

  // make pelvis traj, 0 is pelvis,
  body_motions_.resize(3);
  num_T = 2;
  Ts.resize(num_T);
  Ts[0] = 0;
  Ts[1] = p_ds_duration_;

  for (int i = 0; i < 3; i++)
    body_motions_[i] = MakeDefaultBodyMotionData(num_T);

  // pelvis body motion data
  body_motions_[0].body_or_frame_id = rpc_.pelvis_id;
  body_motions_[0].control_pose_when_in_contact.resize(num_T, true);
  std::vector<Eigen::Vector7d> pelv_knots(num_T);
  pelv_knots[0] = pelv0;
  pelv_knots[1] = pelv1;
  body_motions_[0].trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

  // stance foot body motion data
  int id = rpc_.foot_ids[stance_foot];
  body_motions_[1].body_or_frame_id = id;
  body_motions_[1].trajectory = GenerateCubicCartesianSpline(Ts, std::vector<Eigen::Vector7d>(num_T, Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, id))), std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));


  // swing foot body motion data make swing up traj for right foot
  id = rpc_.foot_ids[swing_foot];
  body_motions_[2] = MakeDefaultBodyMotionData(3);
  body_motions_[2].body_or_frame_id = id;
  std::vector<double> swingTs(3);
  swingTs[0] = 0;
  swingTs[1] = p_ds_duration_;
  swingTs[2] = p_ss_duration_ + p_ds_duration_;

  std::vector<Eigen::Vector7d> swing_foot_d;
  swing_foot_d.resize(3, Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, id)));
  swing_foot_d[2][2] += 0.2;
  std::vector<Eigen::Vector7d> swing_footd_d = std::vector<Eigen::Vector7d>(swingTs.size(), Eigen::Vector7d::Zero());
  body_motions_[2].trajectory = GenerateCubicCartesianSpline(swingTs, swing_foot_d, swing_footd_d);

  // hold arm joints, I am assuming the leg joints will just be ignored..
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(est_rs.q.size());
  q_trajs_ = GenerateCubicSpline(Ts, std::vector<Eigen::VectorXd>(num_T, est_rs.q), zero, zero);

  // contact state and contact switching time
  contact_state_.clear();
  contact_state_.push_back(DSc);
  if (swing_foot == Side::LEFT)
    contact_state_.push_back(SSR);
  else
    contact_state_.push_back(SSL);

  contact_switching_time_.clear();
  contact_switching_time_.push_back(p_ds_duration_);
  contact_switching_time_.push_back(INFINITY);
}

drake::lcmt_qp_controller_input SingleSupportPlan::MakeQPInput(const DrakeRobotState &est_rs) {
  double cur_time = est_rs.t;

  if (interp_t0_ == -1)
    interp_t0_ = cur_time;
  double plan_time = cur_time - interp_t0_;

  // switch contact state tape
  if (plan_time >= contact_switching_time_.front()) {
    contact_state_.pop_front();
    contact_switching_time_.pop_front();
    contact_switch_time_ = cur_time;
  }
  support_state_ = MakeDefaultSupportState(contact_state_.front());

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

 
