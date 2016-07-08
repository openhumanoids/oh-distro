#include "plan_eval.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/solvers/qpSpline/splineGeneration.h"
#include "drake/util/lcmUtil.h"

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
};
struct SimplePose {
  Eigen::Vector3d lin;
  Eigen::Quaterniond rot;
};
 
static std::string primaryBodyOrFrameName(const std::string& full_body_name);
static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout);
static void KeyframeToState(const bot_core::robot_state_t &keyframe, Eigen::VectorXd &q, Eigen::VectorXd &v);
static Eigen::Vector6d getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
static Eigen::Vector6d getTaskSpaceJacobianDotTimesV(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());
static Eigen::MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body, const Eigen::Vector3d &local_offset = Eigen::Vector3d::Zero());


// poses and vels are task space pose and vel
PiecewisePolynomial<double> nWaypointCubicCartesianSpline(const std::vector<double> &times, const std::vector<SimplePose> &poses, const std::vector<SimplePose> &vels) {
  assert(times.size() == poses.sizes());
  assert(times.size() == vels.sizes());
  assert(times.size() > 0);

  size_t T = times.size();
  std::vector<Eigen::Vector6d> expmap(poses.size()), expmap_dot(poses.size());
  Eigen::Vector3d tmp, tmpd;
  for (size_t t = 0; t < times.size(); t++) {
    quat2expmapSequence(poses[t].rot.coeffs(), vels[t].rot.coeffs(), tmp, tmpd);
    // TODO: need to do the closest expmap mumble
    expmap[t].segment<3>(0) = poses[t].lin;
    expmap[t].segment<3>(3) = tmp;
    expmap_dot[t].segment<3>(0) = vels[t].lin;
    expmap_dot[t].segment<3>(3) = tmpd;
  }

  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(T-1);
  // copy drake/util/pchipDeriv.m
  for (size_t t = 0; t < T-1; t++) {
    polynomials[t].resize(6, 1);
    double a = times[t+1] - times[t];
    double b = a * a;
    double c = b * a;
    for (size_t i = 0; i < 6; i++) {
      double c4 = expmap[t][i];
      double c3 = expmap_dot[t][i];
      double c1 = 1. / b * (expmap_dot[t][i] - c3 - 2. / a * (expmap[t+1][i] - c4 -a * c3));
      double c2 = 1. / b * (expmap[t+1][i] - c4 - a * c3 - c * c1);
      // figure out the order
      Eigen::Vector4d coeffs(c1, c2, c3, c4);
      polynomials[t](i, 0) = Polynomial<double>(coeffs);
    }
  }

  return PiecewisePolynomial<double>(polynomials, times);
}

PiecewisePolynomial<double> nWaypointMultiCubicSpline(const std::vector<double> &times, const std::vector<Eigen::VectorXd> &X) {
  assert(times.size() == X.sizes());
  assert(times.size() > 0);
  assert(X[0].size() > 0);

  size_t dof = X[0].size();
  size_t T = X.size();
  std::vector<PiecewisePolynomial<double>> trajs(dof);
  Eigen::VectorXd tmpX(T-2);

  for (size_t i = 0; i < dof; i++) {
    double q0 = X[0][i];
    double q1 = X[T-1][i];
    double v0 = 0;
    double v1 = 0;
    for (size_t t = 1; t < T-1; t++)
      tmpX[t-1] = X[t][i];
    trajs[i] = nWaypointCubicSpline(times, q0, v0, q1, v1, tmpX);
  }

  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(T-1);
  for (size_t t = 0; t < polynomials.size(); t++) {
    polynomials[t].resize(dof, 1);
    for (size_t i = 0; i < dof; i++) {
      polynomials[t](i, 0) = trajs[i].getPolynomial(t);
    }
  }
  return PiecewisePolynomial<double>(polynomials, trajs[0].getSegmentTimes());
}

void PlanEval::Init() {
  if (!lcm_handle_.good()) {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
    exit(-1);
  }

  lcm::Subscription* sub;
  sub = lcm_handle_.subscribe("COMMITTED_ROBOT_PLAN", &PlanEval::HandleCommittedRobotPlan, this);
  sub->setQueueCapacity(1);
}

void PlanEval::ReceiverLoop()
{
  std::cout << "PlanEval Receiver thread start: " << std::this_thread::get_id() << std::endl;
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
  std::cout << "PlanEval Receiver thread exit: " << std::this_thread::get_id() << std::endl;
}

void PlanEval::PublisherLoop() {
  std::cout << "PlanEval Publisher thread start: " << std::this_thread::get_id() << std::endl;
  double cur_time = 0;

  while (!publisher_stop_) {
    if (has_plan_) {

      //lcm_handle_.publish("QP_CONTROLLER_INPUT", &qp_input);
      // sleep
    }
  }
  std::cout << "PlanEval Publisher thread exit: " << std::this_thread::get_id() << std::endl;
}

drake::lcmt_qp_controller_input PlanEval::MakeManipQPInput(double cur_time) {
  auto q_des = q_trajs_.value(cur_time);
  std::vector<float> q_des_std_vector;
  q_des_std_vector.resize(q_des.size());
  for(int i = 0; i < q_des.size(); i++){
    q_des_std_vector[i] = static_cast<float>(q_des(i));
  }
  int qtrajSegmentIdx = q_trajs_.getSegmentIndex(cur_time);
  // why is it 2?
  int endSegmentIdx = std::min(2, q_trajs_.getNumberOfSegments() - qtrajSegmentIdx);
  PiecewisePolynomial<double> qtrajSlice = q_trajs_.slice(qtrajSegmentIdx, endSegmentIdx);

  drake::lcmt_qp_controller_input qp_input;
  qp_input.be_silent = false;
  qp_input.timestamp = static_cast<int64_t>(cur_time * 1e6);
  qp_input.num_support_data = 0;
  qp_input.num_tracked_bodies = 0;
  qp_input.num_external_wrenches = 0;
  qp_input.num_joint_pd_overrides = 0;


  drake::lcmt_piecewise_polynomial qtrajSplineMsg;
  encodePiecewisePolynomial(qtrajSlice, qtrajSplineMsg);
  qp_input.whole_body_data.spline = qtrajSplineMsg;
  qp_input.whole_body_data.q_des = q_des_std_vector;
  qp_input.whole_body_data.timestamp = 0;
  qp_input.whole_body_data.num_positions = robot_.num_positions;
  // hack constrained_dofs
  qp_input.whole_body_data.constrained_dofs.resize(17);
  for (int i = 0; i < 7; i++) {
    qp_input.whole_body_data.constrained_dofs[i] = 20 + i;
    qp_input.whole_body_data.constrained_dofs[i+7] = 13 + i;
  }
  qp_input.whole_body_data.constrained_dofs[14] = 10;
  qp_input.whole_body_data.constrained_dofs[15] = 7;
  qp_input.whole_body_data.constrained_dofs[16] = 8;
  qp_input.whole_body_data.num_constrained_dofs = qp_input.whole_body_data.constrained_dofs.size();

  // zmp data

  // body motion data
  for (size_t b = 0; b < body_motions_.size(); b++) {
    const BodyMotionData& body_motion = body_motions_[b];
    int body_or_frame_id = body_motion.getBodyOrFrameId();
    int body_id = robot_.parseBodyOrFrameID(body_or_frame_id);
    int body_motion_segment_index = body_motion.findSegmentIndex(cur_time);
    
    bool is_foot = false;
    if (is_foot) {
    
    }
    
    // extract the right knot points
    PiecewisePolynomial<double> body_motion_trajectory_slice =
      body_motion.getTrajectory().slice(
          body_motion_segment_index,
          std::min(2, body_motion.getTrajectory().getNumberOfSegments() -
            body_motion_segment_index)); 
    
    // make lcmt_body_motion_data msg
    drake::lcmt_body_motion_data body_motion_data_for_support_lcm;
    body_motion_data_for_support_lcm.timestamp = 0;
    body_motion_data_for_support_lcm.body_or_frame_name =
        primaryBodyOrFrameName(robot_.getBodyOrFrameName(body_or_frame_id));

    encodePiecewisePolynomial(body_motion_trajectory_slice,
                              body_motion_data_for_support_lcm.spline);

    body_motion_data_for_support_lcm.in_floating_base_nullspace =
        body_motion.isInFloatingBaseNullSpace(body_motion_segment_index);
    body_motion_data_for_support_lcm.control_pose_when_in_contact =
        body_motion.isPoseControlledWhenInContact(body_motion_segment_index);
    
    const Eigen::Isometry3d& transform_task_to_world =
        body_motion.getTransformTaskToWorld();
    Eigen::Vector4d quat_task_to_world = rotmat2quat(transform_task_to_world.linear());
    Eigen::Vector3d translation_task_to_world = transform_task_to_world.translation();
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

    qp_input.body_motion_data.push_back(body_motion_data_for_support_lcm);
    qp_input.num_tracked_bodies++; 
  }

  return qp_input;
}




void PlanEval::Start() {
  receiver_stop_ = false;
  if (isReceiverRunning()) {
    std::cout << "LCM receiver already running.\n";
  }
  else {
    receiver_thread_ = std::thread(&PlanEval::ReceiverLoop, this);
  }

  publisher_stop_ = false;
  if (isPublisherRunning()) {
    std::cout << "LCM publisher already running.\n";
  }
  else {
    publisher_thread_ = std::thread(&PlanEval::PublisherLoop, this);
  }
}

void PlanEval::Stop() {
  receiver_stop_ = true;
  receiver_thread_.join();

  publisher_stop_ = true;
  publisher_thread_.join();
}



void PlanEval::HandleCommittedRobotPlan(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_plan_t* msg) {
  plan_lock_.lock();
  plan_ = *msg;
  plan_lock_.unlock();
  has_plan_ = true;

  double t0 = plan_.utime / 1e6;
  size_t num_T = plan_.plan.size();
  
  std::vector<double> Ts(num_T);
  std::vector<Eigen::VectorXd> q_des(num_T); // t steps by n

  int dof = robot_.num_velocities;
  
  // make 3 body motion data, pelvis, 2 feet
  std::vector<std::string> body_names;
  body_names.push_back(std::string("pelvis"));
  body_names.push_back(std::string("leftFoot"));
  body_names.push_back(std::string("rightFoot"));
  size_t num_bodies = body_names.size();

  std::vector<std::vector<SimplePose>> x_d(num_bodies);
  std::vector<std::vector<SimplePose>> xd_d(num_bodies);
  for (size_t i = 0; i < num_bodies; i++) {
    x_d[i].resize(num_T);
    xd_d[i].resize(num_T);
  }
   
  // go through set points
  for (size_t t = 0; t < num_T; t++) {
    bot_core::robot_state_t &keyframe = plan_.plan[t];
    KeyframeToState(keyframe, q_, v_);
    KinematicsCache<double> cache = robot_.doKinematics(q_, v_);
    
    Ts[t] = t0 + (double)keyframe.utime / 1e6;
    q_des[t] = q_;

    for (size_t b = 0; b < num_bodies; b++) {
      int id = robot_.findLink(body_names[b])->body_index;
      Eigen::Isometry3d pose = robot_.relativeTransform(cache, 0, id);
      x_d[b][t].lin = pose.translation();
      x_d[b][t].rot = Eigen::Quaterniond(pose.linear());
      
      // TODO offset can be nonzero
      Eigen::Vector6d xd = getTaskSpaceVel(robot_, cache, id, Eigen::Vector3d::Zero());
      xd_d[b][t].lin = xd.segment<3>(3);
      // http://www.euclideanspace.com/physics/kinematics/angularvelocity/QuaternionDifferentiation2.pdf
      Eigen::Quaterniond W(0, xd[0], xd[1], xd[2]);
      W = W * x_d[b][t].rot;
      xd_d[b][t].rot = Eigen::Quaterniond(0.5*W.coeffs());
    }
  }

  // make q splines
  q_trajs_ = nWaypointMultiCubicSpline(Ts, q_des);

  // make body motion splines
  body_motions_.resize(num_bodies);
  for (size_t b = 0; b < num_bodies; b++) {
    body_motions_[b].body_or_frame_id = robot_.findLink(body_names[b])->body_index; 
    body_motions_[b].trajectory = nWaypointCubicCartesianSpline(Ts, x_d[b], xd_d[b]);
    body_motions_[b].toe_off_allowed.resize(num_T, false);
    body_motions_[b].in_floating_base_nullspace.resize(num_T, false);
    if (body_names[b].compare("pelvis") == 0)
      body_motions_[b].control_pose_when_in_contact.resize(num_T, true);
    else
      body_motions_[b].control_pose_when_in_contact.resize(num_T, false);

    body_motions_[b].transform_task_to_world = Eigen::Isometry3d::Identity();
    body_motions_[b].xyz_proportional_gain_multiplier = Eigen::Vector3d::Constant(1);
    body_motions_[b].xyz_damping_ratio_multiplier = Eigen::Vector3d::Constant(1);
    body_motions_[b].exponential_map_proportional_gain_multiplier = 1;
    body_motions_[b].exponential_map_damping_ratio_multiplier = 1;
    body_motions_[b].weight_multiplier = Eigen::Vector6d::Constant(1);
  }

  // make zmp traj
  
  // make support

}




std::string primaryBodyOrFrameName(const std::string& full_body_name) {
  int i;
  for (i = 0; i < full_body_name.size(); i++) {
    if (std::strncmp(&full_body_name[i], "+", 1) == 0) {
      break;
    }
  }

  return full_body_name.substr(0, i);
}



void KeyframeToState(const bot_core::robot_state_t &keyframe, Eigen::VectorXd &q, Eigen::VectorXd &v)
{
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
    q[i+6] = keyframe.joint_position[i];
  }
  for (size_t i = 0; i < keyframe.joint_velocity.size(); i++) {
    v[i+6] = keyframe.joint_velocity[i];
  }
}

 

Eigen::Vector6d getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Eigen::Vector3d &local_offset)
{
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
  Eigen::Vector3d world_offset = H_world_to_pt.translation() - element.transform_to_world.translation();
  
  // add the linear vel from the body rotation
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

Eigen::MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body, const Eigen::Vector3d &local_offset)
{
  std::vector<int> v_or_q_indices;
  KinematicPath body_path = r.findKinematicPath(0, body);
  Eigen::MatrixXd Jg = r.geometricJacobian(cache, 0, body, 0, true, &v_or_q_indices);
  Eigen::MatrixXd J(6, r.num_velocities);
  //MatrixXd J(6, r.number_of_velocities());
  J.setZero();

  Eigen::Vector3d points = r.transformPoints(cache, local_offset, body, 0);

  int col = 0;
  for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
    // angular
    J.template block<SPACE_DIMENSION, 1>(0,*it) = Jg.block<3,1>(0,col);
    // linear, just like the linear velocity, assume qd = 1, the column is the linear velocity.
    J.template block<SPACE_DIMENSION, 1>(3,*it) = Jg.block<3,1>(3,col);
    J.template block<SPACE_DIMENSION, 1>(3,*it).noalias() += Jg.block<3,1>(0,col).cross(points);
    col++;
  }

  return J;
}

Eigen::Vector6d getTaskSpaceJacobianDotTimesV(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Eigen::Vector3d &local_offset)
{
  // position of point in world
  Eigen::Vector3d p = r.transformPoints(cache, local_offset, body_or_frame_id, 0);
  Eigen::Vector6d twist = r.relativeTwist(cache, 0, body_or_frame_id, 0);
  Eigen::Vector6d J_geometric_dot_times_v = r.geometricJacobianDotTimesV(cache, 0, body_or_frame_id, 0);

  // linear vel of r
  Eigen::Vector3d pdot = twist.head<3>().cross(p) + twist.tail<3>();

  // each column of J_task Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // Jt * v, angular part stays the same, 
  // linear part = [\dot{Jg_v}v + \dot{Jg_omega}.cross(p) + Jg_omega.cross(rdot)] * v 
  //             = [lin of JgdotV + ang of JgdotV.cross(p) + omega.cross(rdot)]
  Eigen::Vector6d Jdv = J_geometric_dot_times_v;
  Jdv.tail<3>() += twist.head<3>().cross(pdot) + J_geometric_dot_times_v.head<3>().cross(p);

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
  if (status == -1 && errno != EINTR)
  {
    printf("select() returned error: %d\n", errno);
  }
  else if (status == -1 && errno == EINTR)
  {
    printf("select() interrupted\n");
  }
  return (status > 0 && FD_ISSET(lcmFd, &fds));
}
 
