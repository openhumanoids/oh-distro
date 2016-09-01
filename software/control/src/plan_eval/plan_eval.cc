#include "plan_eval.h"

#include <fstream>
#include <iomanip>

#include "manip_plan.h"
#include "walking_plan.h"
#include "single_support_plan.h"
#include "drc/plan_status_t.hpp"

static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout);

PlanEval::PlanEval(const std::string &urdf_name, const std::string &config_name) {
  // names
  urdf_name_ = urdf_name;
  config_name_ = config_name;
  new_robot_state_ = false;

  // state decoder stuff
  RigidBodyTree r(urdf_name);
  est_robot_state_.q.resize(r.num_positions);
  est_robot_state_.qd.resize(r.num_velocities);
  int num_states = r.num_positions + r.num_velocities;
  std::vector<std::string> state_coordinate_names(num_states);
  for (int i = 0; i < num_states; i++) {
    state_coordinate_names[i] = r.getStateName(i);
  }
  state_driver_.reset(new RobotStateDriver(state_coordinate_names));

  // threading + lcm
  receiver_stop_ = false;
  publisher_stop_ = false;
  new_plan_ = false;

  if (!lcm_handle_.good()) {
    throw std::runtime_error("lcm is not good()");
  }

  lcm::Subscription *sub;
  sub = lcm_handle_.subscribe("COMMITTED_ROBOT_PLAN",
      &PlanEval::HandleManipPlan, this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_.subscribe("WALKING_CONTROLLER_PLAN_REQUEST",
      &PlanEval::HandleWalkingPlan, this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_.subscribe("EST_ROBOT_STATE", &PlanEval::HandleEstRobotState,
      this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_.subscribe("FOOT_CONTACT_ESTIMATE", &PlanEval::HandleEstContactState,
      this);
  sub->setQueueCapacity(1);
}

void PlanEval::HandleManipPlan(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const drc::robot_plan_t *msg)
{
  std::cout << "Makeing Manip plan\n";

  state_lock_.lock();
  DrakeRobotState local_est_rs = est_robot_state_;
  state_lock_.unlock();

  std::shared_ptr<GenericPlan> new_plan_ptr(new ManipPlan(urdf_name_, config_name_));
  Eigen::VectorXd last_key_frame = est_robot_state_.q;
  if (current_plan_) {
    last_key_frame = current_plan_->GetLatestKeyFrame(local_est_rs.t);
  }
  new_plan_ptr->HandleCommittedRobotPlan(msg, local_est_rs, last_key_frame);

  plan_lock_.lock();
  current_plan_ = new_plan_ptr;
  new_plan_ = true;
  plan_lock_.unlock();
}

void PlanEval::HandleWalkingPlan(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const drc::walking_plan_request_t *msg)
{
  std::cout << "Making Walking plan\n";
  
  DrakeRobotState local_est_rs;
  state_lock_.lock();
  local_est_rs = est_robot_state_;
  state_lock_.unlock();

  //std::shared_ptr<GenericPlan> new_plan_ptr(new SingleSupportPlan(urdf_name_, config_name_));
  std::shared_ptr<GenericPlan> new_plan_ptr(new WalkingPlan(urdf_name_, config_name_));
  Eigen::VectorXd last_key_frame = local_est_rs.q;
  if (current_plan_) {
    last_key_frame = current_plan_->GetLatestKeyFrame(local_est_rs.t);
  }
  new_plan_ptr->HandleCommittedRobotPlan(msg, local_est_rs, last_key_frame);

  plan_lock_.lock();
  current_plan_ = new_plan_ptr;
  new_plan_ = true;
  plan_lock_.unlock();
}

void PlanEval::HandleEstContactState(const lcm::ReceiveBuffer *rbuf,
                           const std::string &channel,
                           const drc::foot_contact_estimate_t *msg)
{
  state_lock_.lock();
  if (msg->left_contact > 0)
    est_robot_state_.contact_state.set_contact(ContactState::L_FOOT);
  else
    est_robot_state_.contact_state.remove_contact(ContactState::L_FOOT);

  if (msg->right_contact > 0)
    est_robot_state_.contact_state.set_contact(ContactState::R_FOOT);
  else
    est_robot_state_.contact_state.remove_contact(ContactState::R_FOOT);

  state_lock_.unlock();
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
  DrakeRobotState local_est_rs;

  while (!publisher_stop_) {
    if (new_plan_) {
      plan_lock_.lock();
      local_ptr = current_plan_;
      new_plan_ = false;
      plan_lock_.unlock();

      has_plan = true;
    }

    if (has_plan && new_robot_state_) {
      state_lock_.lock();
      local_est_rs = est_robot_state_;
      new_robot_state_ = false;
      state_lock_.unlock();

      drake::lcmt_qp_controller_input qp_input = local_ptr->MakeQPInput(local_est_rs);
      lcm_handle_.publish("QP_CONTROLLER_INPUT", &qp_input);

      // publish the plan status
      current_plan_status_ = local_ptr->getPlanStatus();
      this->publishPlanStatus(current_plan_status_);
    }
  }
  std::cout << "PlanEval Publisher thread exit: " << std::this_thread::get_id()
            << std::endl;
}

// publish the status of the plan
void PlanEval::publishPlanStatus(PlanStatus plan_status) {

  drc::plan_status_t msg;

  switch(plan_status.executionStatus){
    case (PlanExecutionStatus::UNKNOWN):
      msg.execution_status = msg.EXECUTION_STATUS_EXECUTING;
      break;

    case (PlanExecutionStatus::EXECUTING):
      msg.execution_status = msg.EXECUTION_STATUS_EXECUTING;
      break;

    case (PlanExecutionStatus::FINISHED):
      msg.execution_status = msg.EXECUTION_STATUS_FINISHED;
      break;
  }

  switch(plan_status.planType){
    case (PlanType::UNKNOWN):
      msg.plan_type = msg.UNKNOWN;
      break;

    case (PlanType::STANDING):
      msg.plan_type = msg.STANDING;
      break;

    case (PlanType::WALKING):
      msg.plan_type = msg.WALKING;
      break;

    case (PlanType::BRACING):
      msg.plan_type = msg.BRACING;
      break;

    case (PlanType::RECOVERING):
      msg.plan_type = msg.RECOVERING;
      break;
  }

  lcm_handle_.publish("PLAN_STATUS", &msg);
}

void PlanEval::HandleEstRobotState(const lcm::ReceiveBuffer *rbuf,
                                   const std::string &channel,
                                   const bot_core::robot_state_t *msg) {
  state_lock_.lock();
  state_driver_->decode(msg, &est_robot_state_);
  new_robot_state_ = true;
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
