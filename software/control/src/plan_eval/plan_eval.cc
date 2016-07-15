#include "plan_eval.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/drakeUtil.h"
#include "drake/solvers/qpSpline/splineGeneration.h"
#include "drake/util/lcmUtil.h"
#include "generate_spline.h"
#include "drake/util/yaml/yamlUtil.h"

#include <fstream>

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
};

static std::string primaryBodyOrFrameName(const std::string &full_body_name);
static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout);
static void KeyframeToState(const bot_core::robot_state_t &keyframe,
                            Eigen::VectorXd &q, Eigen::VectorXd &v);
static Eigen::Vector6d getTaskSpaceVel(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    int body_or_frame_id,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
static Eigen::Vector6d getTaskSpaceJacobianDotTimesV(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    int body_or_frame_id,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
static Eigen::MatrixXd getTaskSpaceJacobian(
    const RigidBodyTree &r, const KinematicsCache<double> &cache, int body,
    const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());


void GenericPlan::LoadConfigurationFromYAML(const std::string &name) {
  YAML::Node config = YAML::LoadFile(name);
  rpc_ = parseKinematicTreeMetadata(config["kinematic_tree_metadata"], robot_);
  
  std::map<Side, std::string> side_identifiers = {{Side::RIGHT, "r"}, {Side::LEFT, "l"}};
  for (const auto& side : Side::values)
    rpc_.hand_ids[side] = robot_.findLinkId(config["kinematic_tree_metadata"]["body_names"]["hands"][side_identifiers[side]].as<std::string>());
  rpc_.pelvis_id = robot_.findLinkId(config["kinematic_tree_metadata"]["body_names"]["pelvis"].as<std::string>());

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
        contacts(i, n) = config["foot_contact_offsets"][foot_name][corners[n]][xyz[i]].as<double>();
    }
    
    contact_offsets[foot_name] = contacts;
    std::cout << foot_name << " contacts:\n" << contacts << std::endl;
  }
  
  // default param
  default_mu_ = get(config, "default_zmp_height").as<double>();
  default_zmp_height_ = get(config, "default_zmp_height").as<double>();
  std::cout << "default_zmp_height: " << default_zmp_height_ << std::endl;
  std::cout << "default_mu: " << default_mu_ << std::endl;
}


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
        primaryBodyOrFrameName(robot_.getBodyOrFrameName(body_or_frame_id));

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
    support_data_element_lcm.body_name = primaryBodyOrFrameName(
        robot_.getBodyOrFrameName(static_cast<int32_t>(element.body)));

    support_data_element_lcm.num_contact_pts = element.contact_points.cols();
    eigenToStdVectorOfStdVectors(element.contact_points,
                                 support_data_element_lcm.contact_pts);

    for (int i = 0; i < 4; i++) {
      support_data_element_lcm.support_logic_map[i] = true;
      support_data_element_lcm.support_surface[i] = element.support_surface[i];
    }
    
    support_data_element_lcm.mu = default_mu_;
    support_data_element_lcm.use_support_surface = true;
  }

  ////////////////////////////////////////
  // torque alpha filter
  if (plan_time < 0.5)
    qp_input.torque_alpha_filter = 0.9;
  else
    qp_input.torque_alpha_filter = 0.;

  return qp_input;
}

void ManipPlan::HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                         const std::string &channel,
                                         const drc::robot_plan_t *msg,
                                         const Eigen::VectorXd est_q,
                                         const Eigen::VectorXd est_qd) {
  std::cout << "committed robot plan handler called\n";
  std::ofstream out;

  size_t num_T = msg->plan.size();

  std::vector<double> Ts(num_T);
  std::vector<Eigen::Vector2d> com_d(num_T);
  std::vector<Eigen::VectorXd> q_d(num_T);  // t steps by n

  // int dof = robot_.num_velocities;

  // TODO: these are hard coded now
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

  // go through set points
  for (size_t t = 0; t < num_T; t++) {
    const bot_core::robot_state_t &keyframe = msg->plan[t];
    KeyframeToState(keyframe, q_, v_);
    KinematicsCache<double> cache_plan = robot_.doKinematics(q_, v_);

    Ts[t] = (double)keyframe.utime / 1e6;
    q_d[t] = q_;

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
  zmp_planner_.Plan(zmp_traj_, x0, default_zmp_height_);

  // make q splines
  q_trajs_ = GenerateCubicSpline(Ts, q_d);

  // make body motion splines
  body_motions_.resize(num_bodies);
  for (size_t b = 0; b < num_bodies; b++) {
    body_motions_[b].body_or_frame_id =
        robot_.findLink(body_names[b])->body_index;
    body_motions_[b].trajectory =
        GenerateCubicCartesianSpline(Ts, x_d[b], xd_d[b]);
    body_motions_[b].toe_off_allowed.resize(num_T, false);
    body_motions_[b].in_floating_base_nullspace.resize(num_T, false);
    if (body_names[b].compare(robot_.getBodyOrFrameName(rpc_.pelvis_id)) == 0)
      body_motions_[b].control_pose_when_in_contact.resize(num_T, true);
    else
      body_motions_[b].control_pose_when_in_contact.resize(num_T, false);

    body_motions_[b].transform_task_to_world = Eigen::Isometry3d::Identity();
    body_motions_[b].xyz_proportional_gain_multiplier =
        Eigen::Vector3d::Constant(1);
    body_motions_[b].xyz_damping_ratio_multiplier =
        Eigen::Vector3d::Constant(1);
    body_motions_[b].exponential_map_proportional_gain_multiplier = 1;
    body_motions_[b].exponential_map_damping_ratio_multiplier = 1;
    body_motions_[b].weight_multiplier = Eigen::Vector6d::Constant(1);
  }

  // make support, dummy here since we are always in double support
  std::vector<std::string> support_names;
  // TODO
  for (auto it = rpc_.foot_ids.begin(); it != rpc_.foot_ids.end(); it++) {
    support_names.push_back(robot_.getBodyOrFrameName(it->second));
  }
  support_state_.resize(support_names.size());

  for (size_t s = 0; s < support_names.size(); s++) {
    RigidBodySupportStateElement &support = support_state_[s];
    support.body = robot_.findLink(support_names[s])->body_index;
    support.use_contact_surface = true;
    support.support_surface = Eigen::Vector4d(0, 0, 1, 0);

    support.contact_points = contact_offsets.at(support_names[s]);
  }

  std::cout << "committed robot plan proced\n";
}

void PlanEval::HandleCommittedRobotPlan(const lcm::ReceiveBuffer *rbuf,
                                        const std::string &channel,
                                        const drc::robot_plan_t *msg)
{
  state_lock_.lock();
  Eigen::VectorXd est_q = est_robot_state_.q;
  Eigen::VectorXd est_qd = est_robot_state_.qd;
  state_lock_.unlock();

  std::shared_ptr<GenericPlan> new_plan_ptr(new ManipPlan(urdf_name_, config_name_));
  new_plan_ptr->HandleCommittedRobotPlan(rbuf, channel, msg, est_q, est_qd);

  plan_lock_.lock();
  current_plan_ = new_plan_ptr;  
  new_plan_ = true;
  plan_lock_.unlock();
}

void PlanEval::ReceiverLoop() {
  std::cout << "PlanEval Receiver thread start: " << std::this_thread::get_id()
            << std::endl;
  while (!receiver_stop_) {
    const double timeout = 0.3;
    bool lcm_ready = WaitForLCM(lcm_handle_, timeout);

    if (lcm_ready) {
      if (lcm_handle_.handle() != 0) {
        std::cerr << "lcm->handle() returned non-zero\n";
        exit(-1);
      }
    }
  }
  std::cout << "PlanEval Receiver thread exit: " << std::this_thread::get_id()
            << std::endl;
}

void PlanEval::PublisherLoop() {
  std::cout << "PlanEval Publisher thread start: " << std::this_thread::get_id()
            << std::endl;

  std::shared_ptr<GenericPlan> local_ptr;
  bool has_plan = false;

  while (!publisher_stop_) {
    if (new_plan_) {
      plan_lock_.lock();
      local_ptr = current_plan_;
      new_plan_ = false;
      plan_lock_.unlock();

      has_plan = true;
    }

    if (has_plan) {
      state_lock_.lock();
      double t_now = est_robot_state_.t;
      state_lock_.unlock();

      drake::lcmt_qp_controller_input qp_input = local_ptr->MakeQPInput(t_now);
      lcm_handle_.publish("QP_CONTROLLER_INPUT", &qp_input);
    }    
  }
  std::cout << "PlanEval Publisher thread exit: " << std::this_thread::get_id()
            << std::endl;
}

void PlanEval::HandleEstRobotState(const lcm::ReceiveBuffer *rbuf,
                                   const std::string &channel,
                                   const bot_core::robot_state_t *msg) {
  state_lock_.lock();
  state_driver_->decode(msg, &est_robot_state_);
  state_lock_.unlock();
}

void PlanEval::Start() {
  receiver_stop_ = false;
  if (isReceiverRunning()) {
    std::cout << "LCM receiver already running.\n";
  } else {
    receiver_thread_ = std::thread(&PlanEval::ReceiverLoop, this);
  }

  publisher_stop_ = false;
  if (isPublisherRunning()) {
    std::cout << "LCM publisher already running.\n";
  } else {
    publisher_thread_ = std::thread(&PlanEval::PublisherLoop, this);
  }
}

void PlanEval::Stop() {
  receiver_stop_ = true;
  receiver_thread_.join();

  publisher_stop_ = true;
  publisher_thread_.join();
}

// utils
std::string primaryBodyOrFrameName(const std::string &full_body_name) {
  int i;
  for (i = 0; i < full_body_name.size(); i++) {
    if (std::strncmp(&full_body_name[i], "+", 1) == 0) {
      break;
    }
  }

  return full_body_name.substr(0, i);
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

Eigen::Vector6d getTaskSpaceVel(const RigidBodyTree &r,
                                const KinematicsCache<double> &cache,
                                int body_or_frame_id,
                                const Eigen::Vector3d &local_offset) {
  Eigen::Isometry3d H_body_to_frame;
  int body_idx = r.parseBodyOrFrameID(body_or_frame_id, &H_body_to_frame);

  const auto &element = cache.getElement(*(r.bodies[body_idx]));
  Eigen::Vector6d T = element.twist_in_world;
  Eigen::Vector3d pt = element.transform_to_world.translation();

  // get the body's task space vel
  Eigen::Vector6d v = T;
  v.tail<3>() += v.head<3>().cross(pt);

  // global offset between pt and body
  auto H_world_to_frame = element.transform_to_world * H_body_to_frame;
  Eigen::Isometry3d H_frame_to_pt(Eigen::Isometry3d::Identity());
  H_frame_to_pt.translation() = local_offset;
  auto H_world_to_pt = H_world_to_frame * H_frame_to_pt;
  Eigen::Vector3d world_offset =
      H_world_to_pt.translation() - element.transform_to_world.translation();

  // add the linear vel from the body rotation
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

Eigen::MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r,
                                     const KinematicsCache<double> &cache,
                                     int body,
                                     const Eigen::Vector3d &local_offset) {
  std::vector<int> v_or_q_indices;
  KinematicPath body_path = r.findKinematicPath(0, body);
  Eigen::MatrixXd Jg =
      r.geometricJacobian(cache, 0, body, 0, true, &v_or_q_indices);
  Eigen::MatrixXd J(6, r.num_velocities);
  // MatrixXd J(6, r.number_of_velocities());
  J.setZero();

  Eigen::Vector3d points = r.transformPoints(cache, local_offset, body, 0);

  int col = 0;
  for (std::vector<int>::iterator it = v_or_q_indices.begin();
       it != v_or_q_indices.end(); ++it) {
    // angular
    J.template block<SPACE_DIMENSION, 1>(0, *it) = Jg.block<3, 1>(0, col);
    // linear, just like the linear velocity, assume qd = 1, the column is the
    // linear velocity.
    J.template block<SPACE_DIMENSION, 1>(3, *it) = Jg.block<3, 1>(3, col);
    J.template block<SPACE_DIMENSION, 1>(3, *it).noalias() +=
        Jg.block<3, 1>(0, col).cross(points);
    col++;
  }

  return J;
}

Eigen::Vector6d getTaskSpaceJacobianDotTimesV(
    const RigidBodyTree &r, const KinematicsCache<double> &cache,
    int body_or_frame_id, const Eigen::Vector3d &local_offset) {
  // position of point in world
  Eigen::Vector3d p =
      r.transformPoints(cache, local_offset, body_or_frame_id, 0);
  Eigen::Vector6d twist = r.relativeTwist(cache, 0, body_or_frame_id, 0);
  Eigen::Vector6d J_geometric_dot_times_v =
      r.geometricJacobianDotTimesV(cache, 0, body_or_frame_id, 0);

  // linear vel of r
  Eigen::Vector3d pdot = twist.head<3>().cross(p) + twist.tail<3>();

  // each column of J_task Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // Jt * v, angular part stays the same,
  // linear part = [\dot{Jg_v}v + \dot{Jg_omega}.cross(p) +
  // Jg_omega.cross(rdot)] * v
  //             = [lin of JgdotV + ang of JgdotV.cross(p) + omega.cross(rdot)]
  Eigen::Vector6d Jdv = J_geometric_dot_times_v;
  Jdv.tail<3>() +=
      twist.head<3>().cross(pdot) + J_geometric_dot_times_v.head<3>().cross(p);

  return Jdv;
}

static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout) {
  int lcmFd = lcm_handle.getFileno();

  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout * 1e6;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcmFd, &fds);

  int status = select(lcmFd + 1, &fds, 0, 0, &tv);
  if (status == -1 && errno != EINTR) {
    printf("select() returned error: %d\n", errno);
  } else if (status == -1 && errno == EINTR) {
    printf("select() interrupted\n");
  }
  return (status > 0 && FD_ISSET(lcmFd, &fds));
}
