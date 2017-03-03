#include "plan_eval.h"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include "manip_plan.h"
#include "walking_plan.h"
#include "drc/plan_status_t.hpp"
#include "utils/simple_timer.h"


namespace plan_eval {
double PLAN_STATUS_RATE_LIMIT_HZ = 3;

static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout);

PlanEval::PlanEval(const std::string &urdf_name, const std::string &config_name) {
  // names
  urdf_name_ = urdf_name;
  config_name_ = config_name;
  new_robot_state_ = false;

  // state decoder stuff
  RigidBodyTree r(urdf_name);
  est_robot_state_receiver_.q.resize(r.num_positions);
  est_robot_state_receiver_.qd.resize(r.num_velocities);

  est_robot_state_publisher_.q.resize(r.num_positions);
  est_robot_state_publisher_.qd.resize(r.num_velocities);
  int num_states = r.num_positions + r.num_velocities;
  std::vector <std::string> state_coordinate_names(num_states);
  for (int i = 0; i < num_states; i++) {
    state_coordinate_names[i] = r.getStateName(i);
  }
  state_driver_.reset(new RobotStateDriver(state_coordinate_names));

  // threading + lcm
  receiver_stop_ = false;
  publisher_stop_ = false;
  new_plan_ = false;
  plan_status_rate_limiter_.setSpeedLimit(PLAN_STATUS_RATE_LIMIT_HZ);

  if (!lcm_handle_receiver_.good()) {
    throw std::runtime_error("lcm is not good()");
  }

  if (!lcm_handle_publisher_.good()) {
    throw std::runtime_error("lcm is not good()");
  }

  this->InitializeBlankPlans();
}

void PlanEval::HandleManipPlan(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const drc::robot_plan_t *msg) {
  std::cout << "\n \n \n Makeing Manip plan\n";
  this->MakeManipPlan(msg);
}

void PlanEval::ConstructNewBlankPlan(const PlanType& plan_type) {
  if(plan_type == PlanType::MANIP){
    std::shared_ptr <GenericPlan> plan_ptr(new ManipPlan(urdf_name_, config_name_));
    blank_plans_[plan_type] = plan_ptr;
  }else if(plan_type == PlanType::WALKING) {
    std::shared_ptr <GenericPlan> plan_ptr(new WalkingPlan(urdf_name_, config_name_));
    blank_plans_[plan_type] = plan_ptr;
  }else{
    throw std::invalid_argument("unknown PlanType in ConstructNewBlankPlan");
  }
}

void PlanEval::InitializeBlankPlans() {
  std::vector<PlanType> plan_types = {PlanType::MANIP, PlanType::WALKING};
  for (const auto & plan_type: plan_types){
    this->ConstructNewBlankPlan(plan_type);
  }
}


void PlanEval::MakeManipPlan(const drc::robot_plan_t *msg) {

  using namespace plan_eval::utils;
  SimpleTimer simple_timer;
  simple_timer.Start();

  std::shared_ptr <GenericPlan> new_plan_ptr = this->blank_plans_.at(PlanType::MANIP);
  Eigen::VectorXd last_key_frame = est_robot_state_receiver_.q;
  if (current_plan_) {
    last_key_frame = current_plan_->GetLatestKeyFrame(est_robot_state_receiver_.t);
  }
  new_plan_ptr->HandleCommittedRobotPlan(msg, est_robot_state_receiver_, last_key_frame);

  plan_lock_.lock();
  current_plan_ = new_plan_ptr;
  new_plan_ = true;
  plan_lock_.unlock();

  std::cout << "handling manip plan took " << simple_timer.Elapsed().count() << " ms" << std::endl;

  // replace the ManipPlan object we just took over in blank_plans_
  this->ConstructNewBlankPlan(PlanType::MANIP);
}


void PlanEval::HandleWalkingPlan(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const drc::walking_plan_request_t *msg) {
  std::cout << "\n \n \n Making Walking plan\n";

  using namespace plan_eval::utils;
  SimpleTimer simple_timer;
  simple_timer.Start();

  std::shared_ptr <GenericPlan> new_plan_ptr = this->blank_plans_.at(PlanType::WALKING);;
  Eigen::VectorXd last_key_frame = est_robot_state_receiver_.q;
  if (current_plan_) {
    last_key_frame = current_plan_->GetLatestKeyFrame(est_robot_state_receiver_.t);
  }
  new_plan_ptr->HandleCommittedRobotPlan(msg, est_robot_state_receiver_, last_key_frame);

  plan_lock_.lock();
  current_plan_ = new_plan_ptr;
  new_plan_ = true;
  plan_lock_.unlock();

  std::cout << "handling walking plan took " << simple_timer.Elapsed().count() << " ms" << std::endl;

  // replace the WalkingPlan that we just used in blank_plans_
  this->ConstructNewBlankPlan(PlanType::WALKING);
}

void PlanEval::HandleEstContactStateReceiverLoop(const lcm::ReceiveBuffer *rbuf,
                                                 const std::string &channel,
                                                 const drc::foot_contact_estimate_t *msg) {
//  state_lock_.lock();
  if (msg->left_contact > 0)
    est_robot_state_receiver_.contact_state.set_contact(ContactState::L_FOOT);
  else
    est_robot_state_receiver_.contact_state.remove_contact(ContactState::L_FOOT);

  if (msg->right_contact > 0)
    est_robot_state_receiver_.contact_state.set_contact(ContactState::R_FOOT);
  else
    est_robot_state_receiver_.contact_state.remove_contact(ContactState::R_FOOT);

//  state_lock_.unlock();
}

void PlanEval::HandleEstContactStatePublisherLoop(const lcm::ReceiveBuffer *rbuf,
                                                  const std::string &channel,
                                                  const drc::foot_contact_estimate_t *msg) {
//  state_lock_.lock();
  if (msg->left_contact > 0)
    est_robot_state_publisher_.contact_state.set_contact(ContactState::L_FOOT);
  else
    est_robot_state_publisher_.contact_state.remove_contact(ContactState::L_FOOT);

  if (msg->right_contact > 0)
    est_robot_state_publisher_.contact_state.set_contact(ContactState::R_FOOT);
  else
    est_robot_state_publisher_.contact_state.remove_contact(ContactState::R_FOOT);

//  state_lock_.unlock();
}

void PlanEval::ReceiverLoop() {

  // setup all the subscriptions
  lcm::Subscription *sub;
  sub = lcm_handle_receiver_.subscribe("COMMITTED_ROBOT_PLAN",
                                       &PlanEval::HandleManipPlan, this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_receiver_.subscribe("WALKING_CONTROLLER_PLAN_REQUEST",
                                       &PlanEval::HandleWalkingPlan, this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_receiver_.subscribe("EST_ROBOT_STATE", &PlanEval::HandleEstRobotStateReceiverLoop,
                                       this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_receiver_.subscribe("FOOT_CONTACT_ESTIMATE", &PlanEval::HandleEstContactStateReceiverLoop,
                                       this);
  sub->setQueueCapacity(1);


  std::cout << "PlanEval Receiver thread start: " << std::this_thread::get_id()
            << std::endl;
  while (!receiver_stop_) {
    const double timeout = 0.3;
    bool lcm_ready = WaitForLCM(lcm_handle_receiver_, timeout);

    if (lcm_ready) {
      if (lcm_handle_receiver_.handle() != 0) {
        std::cerr << "lcm->handle() returned non-zero\n";
        exit(-1);
      }
    }
  }
  std::cout << "PlanEval Receiver thread exit: " << std::this_thread::get_id()
            << std::endl;
}

void PlanEval::PublisherLoop() {
  lcm::Subscription *sub;
  // the publisher and receiver each have their own lcm handles
  sub = lcm_handle_publisher_.subscribe("EST_ROBOT_STATE", &PlanEval::HandleEstRobotStatePublisherLoop,
                                        this);
  sub->setQueueCapacity(1);

  sub = lcm_handle_publisher_.subscribe("FOOT_CONTACT_ESTIMATE", &PlanEval::HandleEstContactStatePublisherLoop,
                                        this);
  sub->setQueueCapacity(1);

  std::cout << "PlanEval Publisher thread start: " << std::this_thread::get_id()
            << std::endl;

  std::shared_ptr <GenericPlan> local_ptr;
  bool has_plan = false;
  DrakeRobotState local_est_rs;

  while (!publisher_stop_) {
    const double timeout = 0.3;
    bool lcm_ready = WaitForLCM(lcm_handle_publisher_, timeout);

    if (lcm_ready) {
      if (lcm_handle_publisher_.handle() != 0) {
        std::cerr << "lcm->handle() returned non-zero\n";
        exit(-1);
      }

      if (new_plan_) {
        plan_lock_.lock();
        local_ptr = current_plan_;
        new_plan_ = false;
        new_robot_state_publisher_loop_ = false; // wait for the next robot state to come in so we don't use an old one by accident
        plan_lock_.unlock();
        has_plan = true;

        // print some debugging info about how long it took to
        // swap in this plan
        double plan_swap_time = est_robot_state_publisher_.t - local_ptr->plan_construction_time_in_seconds_;
        std::cout << "\n \n Publisher Thread: swapping in plan took " << plan_swap_time/1000.0 << " ms \n";

      }

      if (has_plan && new_robot_state_publisher_loop_) {
        new_robot_state_publisher_loop_ = false;

        drake::lcmt_qp_controller_input qp_input = local_ptr->MakeQPInput(est_robot_state_publisher_);
        lcm_handle_publisher_.publish("QP_CONTROLLER_INPUT", &qp_input);

        // publish the plan status
        current_plan_status_ = local_ptr->getPlanStatus();
        this->publishPlanStatus(current_plan_status_);

        // publish plan eval debug msg
        drc::plan_eval_debug_t debug_msg = local_ptr->EncodeDebugData(est_robot_state_publisher_.t);
        lcm_handle_publisher_.publish("PLAN_EVAL_DEBUG", &debug_msg);
      }
    }


  }
  std::cout << "PlanEval Publisher thread exit: " << std::this_thread::get_id()
            << std::endl;
}

// publish the status of the plan
void PlanEval::publishPlanStatus(PlanStatus plan_status) {

  // only publish if the rate limiter says ok
  if (!plan_status_rate_limiter_.tick()) {
    return;
  }

  drc::plan_status_t msg;

  switch (plan_status.executionStatus) {
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

  switch (plan_status.planType) {
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

  lcm_handle_publisher_.publish("PLAN_STATUS", &msg);
}

void PlanEval::HandleEstRobotStateReceiverLoop(const lcm::ReceiveBuffer *rbuf,
                                               const std::string &channel,
                                               const bot_core::robot_state_t *msg) {
//  state_lock_.lock();
  state_driver_->decode(msg, &est_robot_state_receiver_);
  new_robot_state_receiver_loop_ = true;
//  est_robot_state_msg_ = *msg; // copy the msg in case we need it later
//  std::cout << "got an est robot state msg, t = " << msg->utime/1e6 << std::endl;
//  state_lock_.unlock();
}

void PlanEval::HandleEstRobotStatePublisherLoop(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                                const bot_core::robot_state_t *msg) {
//  state_lock_.lock();
  state_driver_->decode(msg, &est_robot_state_publisher_);
  new_robot_state_publisher_loop_ = true;
//  est_robot_state_msg_ = *msg; // copy the msg in case we need it later
//  std::cout << "got an est robot state msg, t = " << msg->utime/1e6 << std::endl;
//  state_lock_.unlock();
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
}