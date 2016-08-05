#include "manip_plan.h"
#include "generate_spline.h"
#include "drake/util/lcmUtil.h"

#include <fstream>
#include <iomanip>

drake::lcmt_qp_controller_input ManipPlan::MakeQPInput(double cur_time) {
  if (interp_t0_ == -1)
    interp_t0_ = cur_time;
  double plan_time = cur_time - interp_t0_;

  drake::lcmt_qp_controller_input qp_input;
  qp_input.be_silent = false;
  qp_input.timestamp = static_cast<int64_t>(cur_time * 1e6);
  qp_input.param_set_name = "manip";

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
  qp_input.whole_body_data.constrained_dofs.push_back(rpc_.position_indices.back_bky);
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
  if (plan_time < p_initial_transition_time_)
    qp_input.torque_alpha_filter = 0.9;
  else
    qp_input.torque_alpha_filter = 0.;

  return qp_input;
}

Eigen::VectorXd ManipPlan::GetLatestKeyFrame(double cur_time) {
  if (interp_t0_ == -1)
    interp_t0_ = cur_time;
  double plan_time = cur_time - interp_t0_;

  return q_trajs_.value(plan_time);
}

void ManipPlan::HandleCommittedRobotPlan(const void *plan_msg,
                                         const Eigen::VectorXd &est_q,
                                         const Eigen::VectorXd &est_qd,
                                         const Eigen::VectorXd &last_q_d) {
  const drc::robot_plan_t *msg = (const drc::robot_plan_t *)plan_msg;
  std::cout << "committed robot plan handler called\n";
  std::ofstream out;

  size_t num_T = msg->plan.size();
  //size_t num_T = msg->plan.size() + 1;

  std::vector<double> Ts(num_T);
  std::vector<Eigen::Vector2d> com_d(num_T);
  std::vector<Eigen::VectorXd> q_d(num_T);  // t steps by n

  std::vector<std::string> body_names;
  body_names.push_back(robot_.getBodyOrFrameName(rpc_.pelvis_id));
  for (auto it = rpc_.foot_ids.begin(); it != rpc_.foot_ids.end(); it++) {
    body_names.push_back(robot_.getBodyOrFrameName(it->second));
  }
  size_t num_bodies = body_names.size();

  std::vector<std::vector<Eigen::Matrix<double,7,1>>> x_d(num_bodies);
  std::vector<std::vector<Eigen::Matrix<double,7,1>>> xd_d(num_bodies);
  for (size_t i = 0; i < num_bodies; i++) {
    x_d[i].resize(num_T);
    xd_d[i].resize(num_T);
  }

  // generate the current tracked body poses from the estimated robot state
  // maybe useful eventually
  KinematicsCache<double> cache_est = robot_.doKinematics(est_q, est_qd);
  std::vector<Eigen::Matrix<double,7,1>> x_est(num_bodies);
  for (size_t b = 0; b < num_bodies; b++) {
    int id = robot_.findLink(body_names[b])->body_index;
    Eigen::Isometry3d pose = robot_.relativeTransform(cache_est, 0, id);
    x_est[b].segment<3>(0) = pose.translation();
    x_est[b].segment<4>(3) = rotmat2quat(pose.linear());
    std::cout << "cur pose " << body_names[b] << " " << x_est[b].segment<4>(3).transpose() << std::endl;
  }

  // generate q_traj first w. cubic spline, which gives velocities.
  for (size_t t = 0; t < num_T; t++) {
    const bot_core::robot_state_t &keyframe = msg->plan[t];
    KeyframeToState(keyframe, q_, v_);
    Ts[t] = (double)keyframe.utime / 1e6;
    q_d[t] = q_;
  }

  // make q, v splines, make sure to set t0 and t1 vel to zero
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(q_d[0].size());
  q_trajs_ = GenerateCubicSpline(Ts, q_d, zero, zero);
  auto v_trajs = q_trajs_.derivative();

  // go through Ts again to make trajs for all the cartesian stuff
  for (size_t t = 0; t < num_T; t++) {
    q_ = q_trajs_.value(Ts[t]);
    v_ = v_trajs.value(Ts[t]);

    KinematicsCache<double> cache_plan = robot_.doKinematics(q_, v_);

    for (size_t b = 0; b < num_bodies; b++) {
      int id = robot_.findLink(body_names[b])->body_index;
      Eigen::Isometry3d pose = robot_.relativeTransform(cache_plan, 0, id);
      x_d[b][t].segment<3>(0) = pose.translation();
      x_d[b][t].segment<4>(3) = rotmat2quat(pose.linear());

      Eigen::Vector6d xd =
          getTaskSpaceVel(robot_, cache_plan, id, Eigen::Vector3d::Zero());
      xd_d[b][t].segment<3>(0) = xd.segment<3>(3);
      // http://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf
      Eigen::Vector4d W(0, xd[0], xd[1], xd[2]);
      xd_d[b][t].segment<4>(3) = 0.5 * quatProduct(W, x_d[b][t].segment<4>(3));
    }

    // get com
    com_d[t] = robot_.centerOfMass(cache_plan).segment<2>(0);
  }


  // make zmp traj, since we are manip, com ~= zmp, zmp is created with pchip
  zmp_traj_ = GeneratePCHIPSpline(Ts, com_d);
  // TODO: make traj for s1, and com
  // drake/examples/ZMP/LinearInvertedPendulum.m

  Eigen::Vector4d x0(Eigen::Vector4d::Zero());
  x0.head(2) = com_d[0];
  zmp_planner_.Plan(zmp_traj_, x0, p_zmp_height_);

  // make body motion splines
  body_motions_.resize(num_bodies);
  for (size_t b = 0; b < num_bodies; b++) {
    body_motions_[b] = MakeDefaultBodyMotionData(num_T);

    body_motions_[b].body_or_frame_id =
        robot_.findLink(body_names[b])->body_index;
    body_motions_[b].trajectory =
        GenerateCubicCartesianSpline(Ts, x_d[b], xd_d[b]);
    if (body_names[b].compare(robot_.getBodyOrFrameName(rpc_.pelvis_id)) == 0)
      body_motions_[b].control_pose_when_in_contact.resize(num_T, true);
  }

  // make support, dummy here since we are always in double support
  support_state_ = MakeDefaultSupportState(DSc);
  std::cout << "committed robot plan proced\n";
}

