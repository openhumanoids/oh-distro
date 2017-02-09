#include "generic_plan.h"
#include "drake/util/lcmUtil.h"

void GenericPlan::LoadConfigurationFromYAML(const std::string &name) {


  // create the robot property cache
  std::cout << "in function GenericPlan::LoadConfigurationFromYAML" << std::endl;
  std::cout << "-------------" << std::endl;
  config_ = YAML::LoadFile(name);
  rpc_ = parseKinematicTreeMetadata(config_["kinematic_tree_metadata"], robot_);

  // parse the pelvis and torso ID, this is turned off by default in parseKinematicTreeMetadata
  std::string pelvis_name = config_["kinematic_tree_metadata"]["body_names"]["pelvis"].as<std::string>();
  std::string torso_name = config_["kinematic_tree_metadata"]["body_names"]["torso"].as<std::string>();
  rpc_.pelvis_id = robot_.findLinkId(pelvis_name);
  rpc_.torso_id = robot_.findLinkId(torso_name);

  std::cout << "parsed kinematic tree metadata " << std::endl;

  // Parse contact point information
  // right now we only have contact points on the feet
  GenericPlanConfig generic_plan_config;
  YAML::Node foot_contact_points_config = config_["contact_points"]["foot_contact_points"];
  std::vector<Side> side_list = {Side::LEFT, Side::RIGHT};
  std::map<Side, ContactState::ContactBody> side_to_contact_body_map;
  side_to_contact_body_map[Side::LEFT] = ContactState::ContactBody::L_FOOT;
  side_to_contact_body_map[Side::RIGHT] = ContactState::ContactBody::R_FOOT;

  for(auto & it: side_to_contact_body_map){
    Side side = it.first;
    ContactState::ContactBody contact_body = it.second;
    std::string side_string = side.toString();
    YAML::Node single_foot_contact_points_config = foot_contact_points_config[side_string];

    // TODO (manuelli): this is getting stored in two places at once, needs to be fixed
    // gets stored in two places at once
    generic_plan_config_.foot_contact_point_data[side] = std::shared_ptr<FootContactPointData>(new FootContactPointData(foot_contact_points_config[side_string]));
    generic_plan_config_.contact_point_data[contact_body] = std::shared_ptr<FootContactPointData>(new FootContactPointData(foot_contact_points_config[side_string]));;


    std::cout << "generic_plan_config_.contact_point_data[contact_body].getAllContactPoints().size() " << generic_plan_config_.contact_point_data.at(contact_body)->getAllContactPoints().size() << std::endl;
  }

  std::cout << "parsing general params " << std::endl;

  // parse general params
  YAML::Node general_params_node = config_["general"];
  generic_plan_config_.mu = general_params_node["mu"].as<double>();
  generic_plan_config_.zmp_height = general_params_node["zmp_height"].as<double>();
  generic_plan_config_.initial_transition_time = general_params_node["initial_transition_time"].as<double>();
  generic_plan_config_.transition_trq_alpha_filter = general_params_node["transition_trq_alpha_filter"].as<double>();
  generic_plan_config_.flip_foot_ft_sensor = general_params_node["flip_foot_ft_sensor"].as<bool>();

  std::cout << "parsing walking params" << std::endl;
  // parse walking params
  WalkingPlanConfig walking_plan_config;
  YAML::Node walking_params_node = config_["walking"];

  walking_plan_config.pelvis_height = walking_params_node["pelvis_height"].as<double>();
  walking_plan_config.ss_duration = walking_params_node["ss_duration"].as<double>();
  walking_plan_config.ds_duration = walking_params_node["ds_duration"].as<double>();
  walking_plan_config.pre_weight_transfer_scale = walking_params_node["pre_weight_transfer_scale"].as<double>();
  walking_plan_config.min_Fz = walking_params_node["min_Fz"].as<double>();
  walking_plan_config.use_force_bounds = walking_params_node["use_force_bounds"].as<bool>();
  walking_plan_config.extend_foot_down_z_vel = walking_params_node["extend_foot_down_z_vel"].as<double>();
  walking_plan_config.swing_foot_touchdown_z_vel = walking_params_node["swing_touchdown_z_vel"].as<double>();
  walking_plan_config.swing_foot_touchdown_z_offset = walking_params_node["swing_touchdown_z_offset"].as<double>();
  walking_plan_config.swing_foot_xy_weight_multiplier = walking_params_node["swing_foot_xy_weight_multiplier"].as<double>();
  walking_plan_config.swing_foot_z_weight_multiplier = walking_params_node["swing_foot_z_weight_multiplier"].as<double>();
  walking_plan_config.pelvis_z_weight_multiplier = walking_params_node["pelvis_z_weight_multiplier"].as<double>();
  walking_plan_config.left_foot_zmp_y_shift = walking_params_node["left_foot_zmp_y_shift"].as<double>();
  walking_plan_config.right_foot_zmp_y_shift = walking_params_node["right_foot_zmp_y_shift"].as<double>();

  walking_plan_config.constrain_back_bkx = walking_params_node["constrainBackJoints"]["back_bkx"].as<bool>();
  walking_plan_config.constrain_back_bky = walking_params_node["constrainBackJoints"]["back_bky"].as<bool>();
  walking_plan_config.constrain_back_bkz = walking_params_node["constrainBackJoints"]["back_bkz"].as<bool>();

  // save walking params into general config
  generic_plan_config_.walking_plan_config = walking_plan_config;

  std::cout << "finished parsing plan eval config " << std::endl;
  
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
  qtrajSlice.shiftRight(generic_plan_state_.plan_start_time);

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
    qp_input.torque_alpha_filter = generic_plan_config_.transition_trq_alpha_filter;
  else
    qp_input.torque_alpha_filter = 0.;


  return qp_input;
}


// records some default debug data
void GenericPlan::RecordDefaultDebugData(double & plan_time){
  // store the zmp_data into the DebugStruct
  debug_data_.com_des = zmp_planner_.com_traj_.value(plan_time);
  debug_data_.comd_des = zmp_planner_.comd_traj_.value(plan_time);
  debug_data_.comdd_des = zmp_planner_.comdd_traj_.value(plan_time);
}

drc::plan_eval_debug_t GenericPlan::EncodeDebugData(double & real_time){
  drc::plan_eval_debug_t msg;
  msg.plan_type = debug_data_.plan_type;
  msg.timestamp = static_cast<int64_t>(real_time * 1e6);
  for (int i = 0; i < 2; i++){
    msg.com_des[i] = debug_data_.com_des[i];
    msg.comdot_des[i] = debug_data_.comd_des[i];
    msg.comddot_des[i] = debug_data_.comdd_des[i];
  }

  return msg;
}

RigidBodySupportState GenericPlan::MakeDefaultSupportState(const ContactState &cs) const{

  int s = 0;
  const std::list<ContactState::ContactBody> all_contacts = cs.bodies_in_contact();
  RigidBodySupportState support_state(all_contacts.size());
  for (const auto & contact_body: all_contacts) {
    int body_idx;
    if (contact_body == ContactState::PELVIS)
      body_idx = rpc_.pelvis_id;
    else if (contact_body == ContactState::L_FOOT)
      body_idx = rpc_.foot_ids.at(Side::LEFT);
    else if (contact_body == ContactState::R_FOOT)
      body_idx = rpc_.foot_ids.at(Side::RIGHT);
    else if (contact_body == ContactState::L_HAND)
      body_idx = rpc_.hand_ids.at(Side::LEFT);
    else if (contact_body == ContactState::R_HAND)
      body_idx = rpc_.hand_ids.at(Side::RIGHT);
    else
      throw std::runtime_error("UNKNOWN BODY in contact");

    // contact_body is of type ContactState::ContactBody
    // us at() instead of [] so this can be a const method
    const std::shared_ptr<ContactPointData> & contact_point_data = generic_plan_config_.contact_point_data.at(contact_body);

    support_state[s].body = body_idx;
    support_state[s].total_normal_force_upper_bound = 3 * robot_.getMass() * 9.81;
    support_state[s].total_normal_force_lower_bound = generic_plan_config_.walking_plan_config.min_Fz;
    support_state[s].use_contact_surface = true;
    support_state[s].support_surface = Eigen::Vector4d(0, 0, 1, 0);

    support_state[s].contact_points = contact_point_data->getAllContactPoints();
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

  msg.mu = this->generic_plan_config_.mu;
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
  body_motion_trajectory_slice.shiftRight(generic_plan_state_.plan_start_time);

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

void GenericPlan::SimpleTest() {
  for(auto & it: generic_plan_config_.contact_point_data){
    std::cout << "contact_point_data key " << it.first << std::endl;
    Eigen::Matrix3Xd contact_pts = it.second->getAllContactPoints();
    std::cout << "contact_pts.size() " << contact_pts.size() << std::endl;
  }

  std::cout << "making default support state " << std::endl;
  this->MakeDefaultSupportState(ContactState::DS());
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

