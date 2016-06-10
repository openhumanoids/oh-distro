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
#include "AtlasCommandDriver.hpp"
#include "FootContactDriver.hpp"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/InstantaneousQPController.h"
#include "sfRobotState.h"


using namespace Eigen;
 
namespace {

std::mutex pointerMutex;
std::atomic<bool> hasNewInput(false);
std::atomic<bool> hasNewState(false);

bot_core::robot_state_t robot_state_msg;
drake::lcmt_qp_controller_input qp_input_msg;
drc::controller_state_t qp_output_msg;
//QPControllerOutput qp_output;

std::vector<std::string> jointNames;

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
    hasNewInput = true;
  }

  void rsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg)
  {
    //std::cout << "received robotstate on lcm thread " << std::this_thread::get_id() << std::endl;

    pointerMutex.lock();
    robot_state_msg = *msg;
    //solveArgs.robot_state = state;

    const bot_core::force_torque_t& force_torque = msg->force_torque;

    //solveArgs.foot_force_torque_measurements[Side::LEFT].frame_idx = solveArgs.pdata->getRPC().foot_ids.at(Side::LEFT); // TODO: make sure that this is right
    //solveArgs.foot_force_torque_measurements[Side::LEFT].wrench << force_torque.l_foot_torque_x, force_torque.l_foot_torque_y, 0.0, 0.0, 0.0, force_torque.l_foot_force_z;

    //solveArgs.foot_force_torque_measurements[Side::RIGHT].frame_idx = solveArgs.pdata->getRPC().foot_ids.at(Side::RIGHT); // TODO: make sure that this is right
    //solveArgs.foot_force_torque_measurements[Side::RIGHT].wrench << force_torque.r_foot_torque_x, force_torque.r_foot_torque_y, 0.0, 0.0, 0.0, force_torque.r_foot_force_z;

    pointerMutex.unlock();
    hasNewState = true;
  }

  void qp_outputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::controller_state_t* msg)
  {
  
  }

  LCMHandler* lcmHandler;
};


} // end anonymous namespace


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

typedef Eigen::Matrix<double, 6, 1> Vector6d;


class sfQPInput {
public:
  VectorXd q_d;
  VectorXd qd_d;
  VectorXd qdd_d;

  Vector2d cop_d;
  Vector2d com_d;

  Vector6d pelvdd_d;
  Vector6d footdd_d[2];
};


int main ()
{
  // start lcm stuff
  LCMHandler lcmHandler;
  LCMControlReceiver controlReceiver(&lcmHandler);

  std::string urdf("/home/siyuanfeng/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim_drake.urdf");
  sfRobotState rs(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));
 
  // start the lcm 
  lcmHandler.Start();
  controlReceiver.InitSubscriptions();

  jointNames.push_back("torsoYaw");
  jointNames.push_back("torsoPitch");
  jointNames.push_back("torsoRoll");
  jointNames.push_back("leftHipYaw");
  jointNames.push_back("leftHipRoll");
  jointNames.push_back("leftHipPitch");
  jointNames.push_back("leftKneePitch");
  jointNames.push_back("leftAnklePitch");
  jointNames.push_back("leftAnkleRoll");
  jointNames.push_back("rightHipYaw");
  jointNames.push_back("rightHipRoll");
  jointNames.push_back("rightHipPitch");
  jointNames.push_back("rightKneePitch");
  jointNames.push_back("rightAnklePitch");
  jointNames.push_back("rightAnkleRoll");
  jointNames.push_back("leftShoulderPitch");
  jointNames.push_back("leftShoulderRoll");
  jointNames.push_back("leftShoulderYaw");
  jointNames.push_back("leftElbowPitch");
  jointNames.push_back("rightShoulderPitch");
  jointNames.push_back("rightShoulderRoll");
  jointNames.push_back("rightShoulderYaw");
  jointNames.push_back("rightElbowPitch");
 
  std::cout << jointNames.size() << std::endl;

  //rs.pos.resize(jointNames.size());
  //rs.vel.resize(jointNames.size());
  //rs.trq.resize(jointNames.size());

  while(1) {
    if (hasNewState) {
      if (!rs.hasInit())
        rs.init(robot_state_msg);

      rs.parseMsg(robot_state_msg);
      
      hasNewState = false;

      std::cout << "com: " << rs.com[0] << ", " << rs.com[1] << std::endl;
      //std::cout << "foot[L]: " << rs.foot[0].translation() << std::endl;
      //std::cout << "foot[R]: " << rs.foot[1].translation() << std::endl;
      //std::cout << rs.footFT_b[0] << rs.footFT_b[1] << std::endl;
      std::cout << "cop: " << rs.cop[0] << ", " << rs.cop[1] << std::endl;
    }
    else {
      std::this_thread::yield();
    }
  }

  return 0;
}
