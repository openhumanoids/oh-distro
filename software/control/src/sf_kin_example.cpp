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
#include "RobotStateDriver.hpp"
#include "AtlasCommandDriver.hpp"
#include "FootContactDriver.hpp"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/InstantaneousQPController.h"

using namespace Eigen;
 
namespace {

std::mutex pointerMutex;
std::atomic<bool> hasNewInput(false);
std::atomic<bool> hasNewState(false);

std::unique_ptr<RigidBodyTree> robot;
DrakeRobotState robot_state;
std::shared_ptr<RobotStateDriver> state_driver;
std::shared_ptr<KinematicsCache<double>> cache;

bot_core::robot_state_t robot_state_msg;
drake::lcmt_qp_controller_input qp_input_msg;
drc::controller_state_t qp_output_msg;
//QPControllerOutput qp_output;

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
    int nq = robot->num_positions;
    int nv = robot->num_velocities;
    robot_state.q.resize(nq);
    robot_state.qd.resize(nv);

    state_driver->decode(msg, &robot_state);

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

// stuff that I want to compute
class sfRobotState {
public:
  // minimum representation
  VectorXd q;
  VectorXd qd;
  VectorXd trq;

  Matrix<double,6,1> footFT_b[2];
  
  // computed from kinematics  
  Eigen::Isometry3d pelv;
  Eigen::Isometry3d foot[2];
  Vector3d com;
  
  Vector3d comd;
  Vector6d pelvd;
  Vector6d footd[2];

  Vector2d cop;
  Vector2d cop_b[2];
  
  Matrix<double,6,1> footFT_w[2];


  void genKin(const DrakeRobotState rs, const RigidBodyTree &robot, KinematicsCache<double> &cache) 
  {
    this->q = rs.q;
    this->qd = rs.qd;
    cache.initialize(this->q, this->qd);
    robot.doKinematics(cache, true);

    this->foot[0].linear() = robot.transformPoints(cache, Eigen::Vector3d::Zero(), body_or_frame_name_to_id.at("leftFoot"), 0);
    this->foot[1].linear() = robot.transformPoints(cache, Eigen::Vector3d::Zero(), body_or_frame_name_to_id.at("rightFoot"), 0);
    this->pelv.linear() = robot.transformPoints(cache, Eigen::Vector3d::Zero(), body_or_frame_name_to_id.at("pelvis"), 0);

    this->com = robot.centerOfMass(cache);
  }

private:
  std::unordered_map<std::string, int> body_or_frame_name_to_id;

  std::unordered_map<std::string, int> computeBodyOrFrameNameToIdMap(const RigidBodyTree& robot) 
  {
    auto id_map = std::unordered_map<std::string, int>();
    for (auto it = robot.bodies.begin(); it != robot.bodies.end(); ++it) {
      id_map[(*it)->linkname] = it - robot.bodies.begin();
    }

    for (auto it = robot.frames.begin(); it != robot.frames.end(); ++it) {
      id_map[(*it)->name] = -(it - robot.frames.begin()) - 2;
    }
    return id_map;
  }

  void init(const RigidBodyTree &robot)
  {
    this->body_or_frame_name_to_id = computeBodyOrFrameNameToIdMap(robot);
  }

};

class sfQPInput {
  VectorXd q_d;
  VectorXd qd_d;
  VectorXd qdd_d;

  Vector2d cop_d;
  Vector2d com_d;

  Vector6d pelvdd_d;
  Vector6d footdd_d[2];
};

class sfQPOutput {
  VectorXd qdd;
  VectorXd trq;
  VectorXd grf[2];

  Vector2d comdd;
  Vector6d pelvdd;
  Vector6d footdd;
};


int main ()
{
  // start lcm stuff
  LCMHandler lcmHandler;
  LCMControlReceiver controlReceiver(&lcmHandler);

  std::string urdf("/home/sfeng/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim.urdf");
  robot = std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW));
  
  int num_states = robot->num_positions + robot->num_velocities;
  std::vector<string> state_coordinate_names(num_states);
  for (int i=0; i<num_states; i++){
    state_coordinate_names[i] = robot->getStateName(i);
  }
  state_driver.reset(new RobotStateDriver(state_coordinate_names));
  cache.reset(new KinematicsCache<double>(robot->bodies));

  // start the lcm 
  lcmHandler.Start();
  controlReceiver.InitSubscriptions();
  
  // stuff
  Eigen::Vector3d com, comd;
  Eigen::Vector3d foot[2], footd[2];
  
  std::unordered_map<std::string, int> body_or_frame_name_to_id = computeBodyOrFrameNameToIdMap(*robot);
  int footIdx[2] = {0};
  footIdx[0] = body_or_frame_name_to_id.at("leftFoot");
  footIdx[1] = body_or_frame_name_to_id.at("rightFoot");

  while(1) {
    if (hasNewState) {
      cache->initialize(robot_state.q, robot_state.qd);
      robot->doKinematics(*cache, true);
      com = robot->centerOfMass(*cache);
      
      for (int i = 0; i < 2; i++)
        foot[i] = robot->transformPoints(*cache, Eigen::Vector3d::Zero(), footIdx[i], 0);
      
      std::cout << "com: " << com << std::endl;
      std::cout << "left: " << foot[0] << std::endl;
      std::cout << "right: " << foot[1] << std::endl;
      hasNewState = false;
    }
  }

  return 0;
}
