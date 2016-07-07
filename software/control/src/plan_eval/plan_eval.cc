#include "plan_eval.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/solvers/qpSpline/splineGeneration.h"
#include "drake/util/lcmUtil.h"

static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout);

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


  // make 3 body motion data, pelvis, 2 feet
 
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


void KeyframeToState(const bot_core::robot_state_t &keyframe, Eigen::VectorXd q, Eigen::VectorXd v)
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


void PlanEval::HandleCommittedRobotPlan(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_plan_t* msg) {
  plan_lock_.lock();
  plan_ = *msg;
  plan_lock_.unlock();
  has_plan_ = true;

  double t0 = plan_.utime / 1e6;
  
  std::vector<double> Ts;
  Eigen::MatrixXd q_des; // t steps by n

  int dof = robot_.num_velocities;
  Ts.resize(plan_.plan.size());
  q_des.resize(plan_.plan.size(), dof);

  // go through set points
  for (size_t i = 0; i < plan_.plan.size(); i++) {
    bot_core::robot_state_t &keyframe = plan_.plan[i];
    KeyframeToState(keyframe, q_, v_);
    KinematicsCache<double> cache = robot_.doKinematics(q_, v_);
    
    Ts[i] = t0 + (double)keyframe.utime / 1e6;
    q_des = q_;
  }

  // TODO: extend the cubic traj gen to multi dim
  // the first 6 is bogus. sfeng believes QP controller ignores it anyway
  std::vector<PiecewisePolynomial<double>> q_trajs(dof);
  for (size_t i = 0; i < dof; i++) {
    double q0 = q_des(0, i);  // first knot q
    double qd0 = 0;
    double q1 = q_des(plan_.plan.size()-1, i);  // last knot q
    double qd1 = 0;
    q_trajs[i] = nWaypointCubicSpline(Ts, q0, qd0, q1, qd1, q_des.block(1, i, plan_.plan.size()-2, 1));
  }
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(q_trajs[0].getSegmentTimes().size()-1);
  for (size_t t = 0; t < polynomials.size(); t++) {
    polynomials[t].resize(dof, 1);
    for (size_t i = 0; i < dof; i++) {
      polynomials[t](i, 0) = q_trajs[i].getPolynomial(t);
    }
  }
  q_trajs_ = PiecewisePolynomial<double>(polynomials, q_trajs[0].getSegmentTimes());


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
 
