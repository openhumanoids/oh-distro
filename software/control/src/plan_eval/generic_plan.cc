#include "generic_plan.h"
#include "drake/util/lcmUtil.h"

void GenericPlan::LoadConfigurationFromYAML(const std::string &name) {
  config_ = YAML::LoadFile(name);
  rpc_ = parseKinematicTreeMetadata(config_["kinematic_tree_metadata"], robot_);

  std::map<Side, std::string> side_identifiers = {{Side::LEFT, "l"}, {Side::RIGHT, "r"}};
  for (const auto& side : Side::values)
    rpc_.hand_ids[side] = robot_.findLinkId(config_["kinematic_tree_metadata"]["body_names"]["hands"][side_identifiers[side]].as<std::string>());
  rpc_.pelvis_id = robot_.findLinkId(config_["kinematic_tree_metadata"]["body_names"]["pelvis"].as<std::string>());

  for (auto it = rpc_.foot_ids.begin(); it != rpc_.foot_ids.end(); it++) {
    std::cout << it->first.toString() << " foot name: " << robot_.getBodyOrFrameName(it->second) << std::endl;
  }

  for (auto it = rpc_.position_indices.legs.begin(); it != rpc_.position_indices.legs.end(); it++) {
    std::cout << it->first.toString() << " leg joints: ";
    for (size_t i = 0; i < it->second.size(); i++)
      std::cout << robot_.getPositionName(it->second[i]) << " ";
    std::cout << std::endl;
  }

  for (auto it = rpc_.position_indices.arms.begin(); it != rpc_.position_indices.arms.end(); it++) {
    std::cout << it->first.toString() << " arm joints: ";
    for (size_t i = 0; i < it->second.size(); i++)
      std::cout << robot_.getPositionName(it->second[i]) << " ";
    std::cout << std::endl;
  }

  // contacts
  std::string xyz[3] = {"x", "y", "z"};
  std::string corners[4] = {"left_toe", "right_toe", "left_heel", "right_heel"};
  for (auto it = rpc_.foot_ids.begin(); it != rpc_.foot_ids.end(); it++) {
    std::string foot_name = robot_.getBodyOrFrameName(it->second);
    Eigen::Matrix3Xd contacts(3, 4);
    for (int n = 0; n < 4; n++) {
      for (int i = 0; i < 3; i++)
        contacts(i, n) = config_["foot_contact_offsets"][foot_name][corners[n]][xyz[i]].as<double>();
    }

    contact_offsets[it->second] = contacts;
    std::cout << foot_name << " contacts:\n" << contacts << std::endl;
  }

  // other params
  p_mu_ = get(config_, "mu").as<double>();
  p_zmp_height_ = get(config_, "zmp_height").as<double>();
  p_initial_transition_time_ = get(config_, "initial_transition_time").as<double>();
  p_transition_trq_alpha_filter_ = get(config_, "transition_trq_alpha_filter").as<double>();
  p_min_Fz_ = get(config_, "min_Fz").as<double>();

  std::cout << "p_zmp_height: " << p_zmp_height_ << std::endl;
  std::cout << "mu: " << p_mu_ << std::endl;
  std::cout << "initial_transition_time: " << p_initial_transition_time_ << std::endl;
  std::cout << "transition_trq_alpha_filter: " << p_transition_trq_alpha_filter_ << std::endl;
  std::cout << "p_min_Fz_: " << p_min_Fz_ << std::endl;
}

drake::lcmt_qp_controller_input GenericPlan::MakeDefaultQPInput(double real_time, double plan_time, const std::string &param_set_name, bool apply_torque_alpha_filter) const {
  drake::lcmt_qp_controller_input qp_input;
  qp_input.be_silent = false;
  qp_input.timestamp = static_cast<int64_t>(real_time * 1e6);
  qp_input.param_set_name = "walking";
  qp_input.param_set_name = param_set_name;

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
  // add 1 offset to match matlab indexing, for backward compatibility
  qp_input.whole_body_data.constrained_dofs = constrained_dofs_;
  for (size_t i = 0; i < constrained_dofs_.size(); i++)
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
  if (apply_torque_alpha_filter)
    qp_input.torque_alpha_filter = p_transition_trq_alpha_filter_;
  else
    qp_input.torque_alpha_filter = 0.;

  return qp_input;
}

RigidBodySupportState GenericPlan::MakeDefaultSupportState(const ContactState &cs) const {
  int s = 0;
  const std::list<ContactState::ContactBody> all_contacts = cs.bodies_in_contact();
  RigidBodySupportState support_state(all_contacts.size());
  for (auto it = all_contacts.begin(); it != all_contacts.end(); it++) {
    int body_idx;
    if (*it == ContactState::PELVIS)
      body_idx = rpc_.pelvis_id;
    else if (*it == ContactState::L_FOOT)
      body_idx = rpc_.foot_ids.at(Side::LEFT);
    else if (*it == ContactState::R_FOOT)
      body_idx = rpc_.foot_ids.at(Side::RIGHT);
    else if (*it == ContactState::L_HAND)
      body_idx = rpc_.hand_ids.at(Side::LEFT);
    else if (*it == ContactState::R_HAND)
      body_idx = rpc_.hand_ids.at(Side::RIGHT);
    else
      throw std::runtime_error("UNKNOW BODY in contact");

    support_state[s].body = body_idx;
    support_state[s].total_normal_force_upper_bound = 1.5 * robot_.getMass() * 9.81;
    support_state[s].total_normal_force_lower_bound = p_min_Fz_;
    support_state[s].use_contact_surface = true;
    support_state[s].support_surface = Eigen::Vector4d(0, 0, 1, 0);

    support_state[s].contact_points = contact_offsets.at(body_idx);
    s++;
  }

  return support_state;
}

BodyMotionData GenericPlan::MakeDefaultBodyMotionData(size_t num_segments) const {
  BodyMotionData body_motion;

  body_motion.toe_off_allowed.resize(num_segments, false);
  body_motion.in_floating_base_nullspace.resize(num_segments, false);
  body_motion.control_pose_when_in_contact.resize(num_segments, false);
  body_motion.transform_task_to_world = Eigen::Isometry3d::Identity();
  body_motion.xyz_proportional_gain_multiplier =
    Eigen::Vector3d::Constant(1);
  body_motion.xyz_damping_ratio_multiplier =
    Eigen::Vector3d::Constant(1);
  body_motion.exponential_map_proportional_gain_multiplier = 1;
  body_motion.exponential_map_damping_ratio_multiplier = 1;
  body_motion.weight_multiplier = Eigen::Vector6d::Constant(1);

  return body_motion;
}

drake::lcmt_support_data GenericPlan::EncodeSupportData(const RigidBodySupportStateElement &element) const {
  drake::lcmt_support_data msg;

  msg.timestamp = 0;
  msg.body_name = PrimaryBodyOrFrameName(
      robot_.getBodyOrFrameName(static_cast<int32_t>(element.body)));

  msg.num_contact_pts = element.contact_points.cols();
  eigenToStdVectorOfStdVectors(element.contact_points,
      msg.contact_pts);

  msg.total_normal_force_upper_bound = element.total_normal_force_upper_bound;
  msg.total_normal_force_lower_bound = element.total_normal_force_lower_bound;

  for (int i = 0; i < 4; i++) {
    msg.support_logic_map[i] = true;
    msg.support_surface[i] = element.support_surface[i];
  }

  msg.mu = p_mu_;
  msg.use_support_surface = true;

  return msg;
}

drake::lcmt_body_motion_data GenericPlan::EncodeBodyMotionData(double plan_time, const BodyMotionData &body_motion) const {
  int body_or_frame_id = body_motion.getBodyOrFrameId();
  int segment_index = body_motion.findSegmentIndex(plan_time);

  // extract the right knot points
  PiecewisePolynomial<double> body_motion_trajectory_slice =
    body_motion.getTrajectory().slice(
        segment_index,
        std::min(2, body_motion.getTrajectory().getNumberOfSegments() -
          segment_index));
  body_motion_trajectory_slice.shiftRight(interp_t0_);

  // make message
  drake::lcmt_body_motion_data msg;
  msg.timestamp = 0;
  msg.body_or_frame_name =
    PrimaryBodyOrFrameName(robot_.getBodyOrFrameName(body_or_frame_id));

  // encode traj
  encodePiecewisePolynomial(body_motion_trajectory_slice,
      msg.spline);

  msg.in_floating_base_nullspace =
    body_motion.isInFloatingBaseNullSpace(segment_index);
  msg.control_pose_when_in_contact =
    body_motion.isPoseControlledWhenInContact(segment_index);

  const Eigen::Isometry3d &transform_task_to_world =
    body_motion.getTransformTaskToWorld();
  Eigen::Vector4d quat_task_to_world =
    rotmat2quat(transform_task_to_world.linear());
  Eigen::Vector3d translation_task_to_world =
    transform_task_to_world.translation();
  eigenVectorToCArray(quat_task_to_world,
      msg.quat_task_to_world);
  eigenVectorToCArray(
      translation_task_to_world,
      msg.translation_task_to_world);
  eigenVectorToCArray(body_motion.getXyzProportionalGainMultiplier(),
      msg.xyz_kp_multiplier);
  eigenVectorToCArray(
      body_motion.getXyzDampingRatioMultiplier(),
      msg.xyz_damping_ratio_multiplier);
  msg.expmap_kp_multiplier =
    body_motion.getExponentialMapProportionalGainMultiplier();
  msg.expmap_damping_ratio_multiplier =
    body_motion.getExponentialMapDampingRatioMultiplier();
  eigenVectorToCArray(body_motion.getWeightMultiplier(),
      msg.weight_multiplier);

  return msg;
}


PlanStatus GenericPlan::getPlanStatus() {
  return plan_status_;
}



// utils
std::string PrimaryBodyOrFrameName(const std::string &full_body_name) {
  int i;
  for (i = 0; i < full_body_name.size(); i++) {
    if (std::strncmp(&full_body_name[i], "+", 1) == 0) {
      break;
    }
  }

  return full_body_name.substr(0, i);
}

Eigen::Matrix<double,7,1> Isometry3dToVector7d(const Eigen::Isometry3d &pose) {
  Eigen::Matrix<double,7,1> ret;
  ret.segment<3>(0) = pose.translation();
  ret.segment<4>(3) = rotmat2quat(pose.linear());
  return ret;
}

void KeyframeToState(const bot_core::robot_state_t &keyframe,
                     Eigen::VectorXd &q, Eigen::VectorXd &v) {
  q.resize(6 + keyframe.joint_position.size());
  v.resize(6 + keyframe.joint_velocity.size());
  // assuming floating base
  q[0] = keyframe.pose.translation.x;
  q[1] = keyframe.pose.translation.y;
  q[2] = keyframe.pose.translation.z;

  Eigen::Matrix<double, 4, 1> quat;
  quat[0] = keyframe.pose.rotation.w;
  quat[1] = keyframe.pose.rotation.x;
  quat[2] = keyframe.pose.rotation.y;
  quat[3] = keyframe.pose.rotation.z;
  q.segment<3>(3) = quat2rpy(quat);

  v[0] = keyframe.twist.linear_velocity.x;
  v[1] = keyframe.twist.linear_velocity.y;
  v[2] = keyframe.twist.linear_velocity.z;
  v[3] = keyframe.twist.angular_velocity.x;
  v[4] = keyframe.twist.angular_velocity.y;
  v[5] = keyframe.twist.angular_velocity.z;

  for (size_t i = 0; i < keyframe.joint_position.size(); i++) {
    q[i + 6] = keyframe.joint_position[i];
  }
  for (size_t i = 0; i < keyframe.joint_velocity.size(); i++) {
    v[i + 6] = keyframe.joint_velocity[i];
  }
}

