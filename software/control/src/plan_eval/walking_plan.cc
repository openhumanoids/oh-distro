#include "walking_plan.h"
#include "generate_spline.h"
#include "drake/util/lcmUtil.h"

#include <fstream>
#include <iomanip> 

// we will hack something up for now.
void WalkingPlan::HandleCommittedRobotPlan(const void *plan_msg,
                                const Eigen::VectorXd &est_q,
                                const Eigen::VectorXd &est_qd,
                                const Eigen::VectorXd &last_q_d) {
  const drc::walking_plan_request_t *msg = (const drc::walking_plan_request_t *)plan_msg;
  // current state
  KinematicsCache<double> cache_est = robot_.doKinematics(est_q, est_qd);
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
  com_end_d = feet_pose[0] * com_end_d;

  // compute end points for com / pelvis
  Eigen::Vector2d com0 = robot_.centerOfMass(cache_est).segment<2>(0);
  Eigen::Vector2d com1 = com_end_d.translation().segment<2>(0);
  Eigen::Matrix<double,7,1> pelv0 = Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, rpc_.pelvis_id));
  Eigen::Matrix<double,7,1> pelv1;
  pelv1.segment<3>(0) = com_end_d.translation();
  pelv1[2] += p_zmp_height_;
  pelv1.segment<4>(3) = rotmat2quat(feet_pose[0].linear());

  std::cout << "pelv0" << pelv0.transpose() << std::endl;
  std::cout << "pelv1" << pelv1.transpose() << std::endl;

  // make a zmp traj for ds
  int num_T = 2;
  double ds_duration = 2.;
  double ss_duration = 2.;
  std::vector<double> Ts(num_T);
  std::vector<Eigen::Vector2d> com_d(num_T);
  for (int i = 0; i < num_T; i++) {
    double a = (double)i / (double)(num_T - 1);
    Ts[i] = ds_duration * a;
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
  Ts[1] = ds_duration;

  for (int i = 0; i < 3; i++)
    MakeDefaultBodyMotionData(body_motions_[i], num_T);

  body_motions_[0].body_or_frame_id = rpc_.pelvis_id;
  body_motions_[0].control_pose_when_in_contact.resize(num_T, true);
  std::vector<Eigen::Vector7d> pelv_knots(num_T);
  pelv_knots[0] = pelv0;
  pelv_knots[1] = pelv1;
  body_motions_[0].trajectory = GenerateCubicCartesianSpline(Ts, pelv_knots, std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));

  // foot stays the same
  int id = rpc_.foot_ids[Side::LEFT];
  body_motions_[1].body_or_frame_id = id;
  body_motions_[1].trajectory = GenerateCubicCartesianSpline(Ts, std::vector<Eigen::Vector7d>(num_T, Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, id))), std::vector<Eigen::Vector7d>(num_T, Eigen::Vector7d::Zero()));
  // make swing up traj for right foot
  id = rpc_.foot_ids[Side::RIGHT];
  MakeDefaultBodyMotionData(body_motions_[2], 3);
  body_motions_[2].body_or_frame_id = id;
  std::vector<double> rightTs(3);
  rightTs[0] = 0;
  rightTs[1] = ds_duration;
  rightTs[2] = ss_duration + ds_duration;

  std::vector<Eigen::Vector7d> r_foot_d;
  r_foot_d.resize(3, Isometry3dToVector7d(robot_.relativeTransform(cache_est, 0, id)));
  r_foot_d[2][2] += 0.2;
  std::vector<Eigen::Vector7d> r_footd_d = std::vector<Eigen::Vector7d>(rightTs.size(), Eigen::Vector7d::Zero());
  body_motions_[2].trajectory = GenerateCubicCartesianSpline(rightTs, r_foot_d, r_footd_d);

  // hold arm joints, I am assuming the leg joints will just be ignored..
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(est_q.size());
  q_trajs_ = GenerateCubicSpline(Ts, std::vector<Eigen::VectorXd>(num_T, est_q), zero, zero);

  // contact state and contact switching time
  contact_state_.clear();
  contact_state_.push_back(DSc);
  contact_state_.push_back(SSL);

  contact_switching_time_.clear();
  contact_switching_time_.push_back(ds_duration);
  contact_switching_time_.push_back(INFINITY);
}

drake::lcmt_qp_controller_input WalkingPlan::MakeQPInput(double cur_time) {
  if (interp_t0_ == -1)
    interp_t0_ = cur_time;
  double plan_time = cur_time - interp_t0_;

  // switch contact state tape
  if (plan_time >= contact_switching_time_.front()) {
    contact_state_.pop_front();
    contact_switching_time_.pop_front();
  }
  MakeSupportState(contact_state_.front());

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
  qp_input.whole_body_data.constrained_dofs.push_back(rpc_.position_indices.back_bky);
  // add 1 offset to match matlab indexing, for backward compatibility
  for (size_t i = 0; i < qp_input.whole_body_data.num_constrained_dofs; i++)
    qp_input.whole_body_data.constrained_dofs[i]++;

  qp_input.whole_body_data.num_constrained_dofs =
      qp_input.whole_body_data.constrained_dofs.size();

  ////////////////////////////////////////
  // make zmp data
  qp_input.zmp_data = zmp_planner_.MakeMessage(plan_time);

  ////////////////////////////////////////
  // make body motion data
  qp_input.num_tracked_bodies = body_motions_.size();
  qp_input.body_motion_data.resize(body_motions_.size());
  for (size_t b = 0; b < body_motions_.size(); b++) {
    const BodyMotionData &body_motion = body_motions_[b];
    int body_or_frame_id = body_motion.getBodyOrFrameId();
    // int body_id = robot_.parseBodyOrFrameID(body_or_frame_id);
    int body_motion_segment_index = body_motion.findSegmentIndex(plan_time);

    // TODO: swing etc.
    bool is_foot = false;
    if (is_foot) {
    }

    // extract the right knot points
    PiecewisePolynomial<double> body_motion_trajectory_slice =
        body_motion.getTrajectory().slice(
            body_motion_segment_index,
            std::min(2, body_motion.getTrajectory().getNumberOfSegments() -
                            body_motion_segment_index));
    body_motion_trajectory_slice.shiftRight(interp_t0_);

    // make lcmt_body_motion_data msg
    drake::lcmt_body_motion_data &body_motion_data_for_support_lcm =
        qp_input.body_motion_data[b];
    body_motion_data_for_support_lcm.timestamp = 0;
    body_motion_data_for_support_lcm.body_or_frame_name =
        PrimaryBodyOrFrameName(robot_.getBodyOrFrameName(body_or_frame_id));

    encodePiecewisePolynomial(body_motion_trajectory_slice,
                              body_motion_data_for_support_lcm.spline);

    body_motion_data_for_support_lcm.in_floating_base_nullspace =
        body_motion.isInFloatingBaseNullSpace(body_motion_segment_index);
    body_motion_data_for_support_lcm.control_pose_when_in_contact =
        body_motion.isPoseControlledWhenInContact(body_motion_segment_index);

    const Eigen::Isometry3d &transform_task_to_world =
        body_motion.getTransformTaskToWorld();
    Eigen::Vector4d quat_task_to_world =
        rotmat2quat(transform_task_to_world.linear());
    Eigen::Vector3d translation_task_to_world =
        transform_task_to_world.translation();
    eigenVectorToCArray(quat_task_to_world,
                        body_motion_data_for_support_lcm.quat_task_to_world);
    eigenVectorToCArray(
        translation_task_to_world,
        body_motion_data_for_support_lcm.translation_task_to_world);
    eigenVectorToCArray(body_motion.getXyzProportionalGainMultiplier(),
                        body_motion_data_for_support_lcm.xyz_kp_multiplier);
    eigenVectorToCArray(
        body_motion.getXyzDampingRatioMultiplier(),
        body_motion_data_for_support_lcm.xyz_damping_ratio_multiplier);
    body_motion_data_for_support_lcm.expmap_kp_multiplier =
        body_motion.getExponentialMapProportionalGainMultiplier();
    body_motion_data_for_support_lcm.expmap_damping_ratio_multiplier =
        body_motion.getExponentialMapDampingRatioMultiplier();
    eigenVectorToCArray(body_motion.getWeightMultiplier(),
                        body_motion_data_for_support_lcm.weight_multiplier);
  }

  ////////////////////////////////////////
  // make support data
  qp_input.num_support_data = support_state_.size();
  qp_input.support_data.resize(support_state_.size());
  for (size_t s = 0; s < support_state_.size(); s++) {
    drake::lcmt_support_data &support_data_element_lcm =
        qp_input.support_data[s];
    const RigidBodySupportStateElement &element = support_state_[s];

    support_data_element_lcm.timestamp = 0;
    support_data_element_lcm.body_name = PrimaryBodyOrFrameName(
        robot_.getBodyOrFrameName(static_cast<int32_t>(element.body)));

    support_data_element_lcm.num_contact_pts = element.contact_points.cols();
    eigenToStdVectorOfStdVectors(element.contact_points,
                                 support_data_element_lcm.contact_pts);

    for (int i = 0; i < 4; i++) {
      support_data_element_lcm.support_logic_map[i] = true;
      support_data_element_lcm.support_surface[i] = element.support_surface[i];
    }

    support_data_element_lcm.mu = p_mu_;
    support_data_element_lcm.use_support_surface = true;
  }
  
  ////////////////////////////////////////
  // torque alpha filter
  qp_input.torque_alpha_filter = 0.;

  return qp_input;
}


