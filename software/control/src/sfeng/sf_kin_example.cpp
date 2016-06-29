#include <mutex>
#include <thread>
// #include <chrono>
#include <atomic>
#include <sys/select.h>
#include "drake/lcmt_qp_controller_input.hpp"
#include "drc/controller_state_t.hpp"
#include "drc/controller_status_t.hpp"
#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/drakeGeometryUtil.h"
#include "../AtlasCommandDriver.hpp"
#include "../FootContactDriver.hpp"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/InstantaneousQPController.h"
#include <csignal>

#include "sfRobotState.h"
#include "sfQPState.h"
#include "../RobotStateDriver.hpp"
#include "QPEstimator.h"

using namespace Eigen;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
 
namespace {

std::mutex pointerMutex;
std::atomic<bool> hasNewQPIn(false);
std::atomic<bool> hasNewState(false);
std::atomic<bool> hasNewQPOut(false);

MRDLogger logger;
const char *home_dir = NULL;

bot_core::robot_state_t robot_state_msg;
drake::lcmt_qp_controller_input qp_input_msg;
drc::controller_state_t qp_output_msg;

class LCMHandler {

public:

  std::thread ThreadHandle;
  std::shared_ptr<lcm::LCM> LCMHandle;
  bool ShouldStop;

  LCMHandler()
  {
    this->ShouldStop = false;
    this->InitLCM();
  }

  void InitLCM()
  {
    this->LCMHandle = std::shared_ptr<lcm::LCM>(new lcm::LCM);
    if(!this->LCMHandle->good())
    {
      std::cout << "ERROR: lcm is not good()" << std::endl;
    }
  }

  bool WaitForLCM(double timeout)
  {
    int lcmFd = this->LCMHandle->getFileno();

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

  void ThreadLoopWithSelect()
  {

    std::cout << "ThreadLoopWithSelect " << std::this_thread::get_id() << std::endl;

    while (!this->ShouldStop)
    {
      const double timeoutInSeconds = 0.3;
      bool lcmReady = this->WaitForLCM(timeoutInSeconds);

      if (this->ShouldStop)
      {
        break;
      }

      if (lcmReady)
      {
        if (this->LCMHandle->handle() != 0)
        {
          printf("lcm->handle() returned non-zero\n");
          break;
        }
      }
    }

    std::cout << "ThreadLoopWithSelect ended " << std::this_thread::get_id() << std::endl;

  }

  void ThreadLoop()
  {
    while (!this->ShouldStop)
    {
      if (this->LCMHandle->handle() != 0)
      {
        printf("lcm->handle() returned non-zero\n");
        break;
      }
    }
  }

  bool IsRunning()
  {
    return this->ThreadHandle.joinable();
  }

  void Start()
  {
    std::cout << "LCMHandler start... " << std::this_thread::get_id() << std::endl;
    if (this->IsRunning())
    {
      std::cout << "already running lcm thread. " << std::this_thread::get_id() << std::endl;
      return;
    }

    this->ShouldStop = false;
    this->ThreadHandle = std::thread(&LCMHandler::ThreadLoopWithSelect, this);
  }

  void Stop()
  {
    this->ShouldStop = true;
    this->ThreadHandle.join();
  }

}; 


class LCMControlReceiver {
public:

  LCMControlReceiver(LCMHandler* lh)
  {
    this->lcmHandler = lh;
  }

  void InitSubscriptions()
  {
    lcm::Subscription* sub;

    sub = this->lcmHandler->LCMHandle->subscribe("QP_CONTROLLER_INPUT", &LCMControlReceiver::qp_inputHandler, this);
    sub->setQueueCapacity(1);

    sub = this->lcmHandler->LCMHandle->subscribe("EST_ROBOT_STATE", &LCMControlReceiver::rsHandler, this);
    sub->setQueueCapacity(1);

    sub = this->lcmHandler->LCMHandle->subscribe("CONTROLLER_STATE", &LCMControlReceiver::qp_outputHandler, this);
    sub->setQueueCapacity(1);
    
    //sub = this->lcmHandler->LCMHandle->subscribe("FOOT_CONTACT_ESTIMATE", &LCMControlReceiver::onFootContact, this);
    //sub->setQueueCapacity(1);
  }

  void qp_inputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drake::lcmt_qp_controller_input* msg)
  {
    //std::cout << "received qp_input on lcm thread " << std::this_thread::get_id() << std::endl;
    pointerMutex.lock();
    qp_input_msg = *msg; 
    pointerMutex.unlock();
    hasNewQPIn = true;
  }

  void rsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg)
  {
    //std::cout << "received robotstate on lcm thread " << std::this_thread::get_id() << std::endl;

    pointerMutex.lock();
    robot_state_msg = *msg;

    pointerMutex.unlock();
    hasNewState = true;
  }

  void qp_outputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::controller_state_t* msg)
  {
    pointerMutex.lock();
    qp_output_msg = *msg;
    pointerMutex.unlock();
    hasNewQPOut = true;
  }

  LCMHandler* lcmHandler;
};



std::unordered_map<std::string, int> computeBodyOrFrameNameToIdMap(
    const RigidBodyTree& robot) {
  auto id_map = std::unordered_map<std::string, int>();
  for (auto it = robot.bodies.begin(); it != robot.bodies.end(); ++it) {
    id_map[(*it)->linkname] = it - robot.bodies.begin();
  }

  for (auto it = robot.frames.begin(); it != robot.frames.end(); ++it) {
    id_map[(*it)->name] = -(it - robot.frames.begin()) - 2;
  }
  return id_map;
}

} // end anonymous namespace






void sigHandler(int signum)
{
  std::cout << "CAUGHT SIG " << signum << std::endl;
  if (signum == SIGINT) {
    //logger.writeToMRDPLOT(std::string(home_dir) + std::string("/logs/mrdplot/bal"));
    bool ret = logger.writeToFile(std::string(home_dir) + std::string("/logs/mrdplot/bal1"));
    if (ret)
      std::cout << "ALL LOGS SAVED, GOODBYE CAPTAIN\n";
    else 
      std::cerr << "failed to save logs\n";
  }
  exit(0);
}



int main ()
{
  signal(SIGINT, sigHandler);

  // start lcm stuff
  LCMHandler lcmHandler;
  LCMControlReceiver controlReceiver(&lcmHandler);
  home_dir = getenv("HOME");
  std::string urdf = std::string(home_dir) + std::string("/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim_drake.urdf");
  sfRobotState rs(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));
  sfQPState qpout;
 
  // start the lcm 
  lcmHandler.Start();
  controlReceiver.InitSubscriptions();
  
  logger.setFrequency(1./0.002);

  // state driver stuff
  DrakeRobotState robot_state;
  std::shared_ptr<RobotStateDriver> state_driver;
  int num_states = rs.robot->num_positions + rs.robot->num_velocities;
  std::vector<std::string> state_coordinate_names(num_states);
  for (int i=0; i<num_states; i++){
    state_coordinate_names[i] = rs.robot->getStateName(i);
  }
  state_driver.reset(new RobotStateDriver(state_coordinate_names)); 
  
  VectorXd trq;
  Vector6d l_ft, r_ft;
  double t0;
  
  robot_state.q.resize(rs.robot->num_positions);
  robot_state.qd.resize(rs.robot->num_velocities);
  trq.resize(rs.robot->num_velocities);
  t0 = -1; 

  // qp state estimator
  QPEstimator est(urdf);
  if (!est.loadParamFromFile(std::string(home_dir) + std::string("/code/oh-distro-private/software/control/src/sfeng/config/qpe_params")))
    throw std::runtime_error("qp state estimator cant load param file.");

  while(true) {
    if (hasNewState) {
      hasNewState = false;
      state_driver->decode(&robot_state_msg, &robot_state, &trq);
      l_ft[3] = robot_state_msg.force_torque.l_foot_force_x;
      l_ft[4] = robot_state_msg.force_torque.l_foot_force_y;
      l_ft[5] = robot_state_msg.force_torque.l_foot_force_z;
      l_ft[0] = robot_state_msg.force_torque.l_foot_torque_x;
      l_ft[1] = robot_state_msg.force_torque.l_foot_torque_y;
      l_ft[2] = robot_state_msg.force_torque.l_foot_torque_z;
      r_ft[3] = robot_state_msg.force_torque.r_foot_force_x;
      r_ft[4] = robot_state_msg.force_torque.r_foot_force_y;
      r_ft[5] = robot_state_msg.force_torque.r_foot_force_z;
      r_ft[0] = robot_state_msg.force_torque.r_foot_torque_x;
      r_ft[1] = robot_state_msg.force_torque.r_foot_torque_y;
      r_ft[2] = robot_state_msg.force_torque.r_foot_torque_z;
      
      if (t0 == -1) {
        t0 = robot_state.t;
        rs.addToLog(logger);
        est.addToLog(logger);
      }
      rs.update(robot_state.t-t0, robot_state.q, robot_state.qd, trq.segment(6,trq.size()-6), l_ft, r_ft, true);

      // qp state estimator
      est.init(robot_state.t-t0, rs.pos, rs.vel, rs.trq, rs.footFT_b[Side::LEFT], rs.footFT_b[Side::RIGHT]);
      est.estimate(robot_state.t-t0, rs.pos, rs.vel, rs.trq, rs.footFT_b[Side::LEFT], rs.footFT_b[Side::RIGHT]);

      logger.saveData();
    }
    
    if (hasNewQPIn) {
      hasNewQPIn = false;
      qpout.parseZMPInput(qp_input_msg);
    }

    if (hasNewQPOut && t0 != -1) {
      if (!qpout.hasInit()) {
        qpout.init(qp_output_msg);
        qpout.addToLog(logger, rs);
      }

      hasNewQPOut = false;
      qpout.parseMsg(qp_output_msg, rs);
    }
  }

  return 0;
}






