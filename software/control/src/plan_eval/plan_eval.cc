#include "plan_eval.h"

#include <fstream>
#include <iomanip>

#include "manip_plan.h"
#include "walking_plan.h"

static bool WaitForLCM(lcm::LCM &lcm_handle, double timeout);


void PlanEval::HandleManipPlan(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const drc::robot_plan_t *msg)
{
  std::cout << "Makeing Manip plan\n";

  state_lock_.lock();
  Eigen::VectorXd est_q = est_robot_state_.q;
  Eigen::VectorXd est_qd = est_robot_state_.qd;
  double cur_time = est_robot_state_.t;
  state_lock_.unlock();

  std::shared_ptr<GenericPlan> new_plan_ptr(new ManipPlan(urdf_name_, config_name_));
  Eigen::VectorXd last_key_frame = est_robot_state_.q;
  if (current_plan_) {
    last_key_frame = current_plan_->GetLatestKeyFrame(cur_time);
  }
  new_plan_ptr->HandleCommittedRobotPlan(msg, est_q, est_qd, last_key_frame);

  plan_lock_.lock();
  current_plan_ = new_plan_ptr;
  new_plan_ = true;
  plan_lock_.unlock();
}

void PlanEval::HandleWalkingPlan(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const drc::walking_plan_request_t *msg)
{
  std::cout << "Makeing Walking plan\n";

  state_lock_.lock();
  Eigen::VectorXd est_q = est_robot_state_.q;
  Eigen::VectorXd est_qd = est_robot_state_.qd;
  double cur_time = est_robot_state_.t;
  state_lock_.unlock();

  std::shared_ptr<GenericPlan> new_plan_ptr(new WalkingPlan(urdf_name_, config_name_));
  Eigen::VectorXd last_key_frame = est_robot_state_.q;
  if (current_plan_) {
    last_key_frame = current_plan_->GetLatestKeyFrame(cur_time);
  }
  new_plan_ptr->HandleCommittedRobotPlan(msg, est_q, est_qd, last_key_frame);

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

    if (has_plan && new_robot_state_) {
      state_lock_.lock();
      double t_now = est_robot_state_.t;
      new_robot_state_ = false;
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
