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

// stuff that I want to compute
class sfRobotState {
public:
  std::unique_ptr<RigidBodyTree> robot;
  
  // minimum representation
  VectorXd q;
  VectorXd qd;
  Matrix<double,6,1> footFT_b[2];
  
  VectorXd pos;
  VectorXd vel;
  VectorXd trq;
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
  

  sfRobotState(std::unique_ptr<RigidBodyTree> robot_in)
    : robot(std::move(robot_in)),
      cache(this->robot->bodies)
  {
    _init();
  }

  void parseMsg(const )

  void genKin(const DrakeRobotState &rs) 
  {
    this->q = rs.q;
    this->qd = rs.qd;

    this->cache.initialize(this->q, this->qd);
    this->robot->doKinematics(cache, true);

    int foot_id[2], pelv_id;
    foot_id[0] = body_or_frame_name_to_id.at("leftFoot");
    foot_id[1] = body_or_frame_name_to_id.at("rightFoot");
    pelv_id = body_or_frame_name_to_id.at("pelvis");
    
    KinematicPath body_path;
    MatrixXd Jcompact, Jfull;
    

    // com
    this->com = this->robot->centerOfMass(cache);

    Jfull = this->robot->centerOfMassJacobian(cache);
    this->comd = Jfull * this->qd;

    // pelvis
    this->pelv.translation() = this->robot->transformPoints(cache, Eigen::Vector3d::Zero(), pelv_id, 0);
    
    body_path = this->robot->findKinematicPath(0, pelv_id);
    Jcompact = this->robot->geometricJacobian(this->cache, 0, pelv_id, pelv_id, true);
    Jfull = this->robot->compactToFull(Jcompact, body_path.joint_path, true);
    this->pelvd = Jfull * this->qd;

    // feet 
    for (int i = 0; i < 2; i++) {
      this->foot[i].translation() = this->robot->transformPoints(cache, Eigen::Vector3d::Zero(), foot_id[i], 0);
      
      body_path = this->robot->findKinematicPath(0, foot_id[i]);
      Jcompact = this->robot->geometricJacobian(this->cache, 0, foot_id[i], foot_id[i], true);
      Jfull = this->robot->compactToFull(Jcompact, body_path.joint_path, true);
    
      this->footd[i] = Jfull * this->qd;
    }
  }

  void buildJointName2Id(const bot_core::robot_state_t &msg) 
  {
    joint_name_to_id.clear();
    for (int i = 0; i < msg.joint_name.size(); i++) {
      joint_name_to_id[msg.joint_name[i]] = i;
    }
  }

private:
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> body_or_frame_name_to_id;
  std::unordered_map<std::string, int> joint_name_to_id;

  void _init()
  {
    // build map
    this->body_or_frame_name_to_id = std::unordered_map<std::string, int>();
    for (auto it = robot->bodies.begin(); it != robot->bodies.end(); ++it) {
      this->body_or_frame_name_to_id[(*it)->linkname] = it - robot->bodies.begin();
    }

    for (auto it = robot->frames.begin(); it != robot->frames.end(); ++it) {
      this->body_or_frame_name_to_id[(*it)->name] = -(it - robot->frames.begin()) - 2;
    } 
  }

};

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

class sfQPOutput {
public:
  VectorXd qdd;
  VectorXd trq;
  Vector6d grf[2];

  Vector2d comdd;
  Vector6d pelvdd;
  Vector6d footdd;

  void parseMsg(const drc::controller_state_t &msg) 
  {
    for (int i = 0; i < msg.num_joints; i++) {
      this->qdd[i] = msg.qdd[i];
      this->trq[i] = msg.u[i]; // first 6 = zero
    }
  }

  void init(const drc::controller_state_t &msg)
  {
    this->qdd.resize(msg.num_joints);
    this->trq.resize(msg.num_joints);
  }
};


int main ()
{
  // start lcm stuff
  LCMHandler lcmHandler;
  LCMControlReceiver controlReceiver(&lcmHandler);

  std::string urdf("/home/sfeng/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim.urdf");
  sfRobotState rs(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));
  
  int num_states = rs.robot->num_positions + rs.robot->num_velocities;
  std::vector<string> state_coordinate_names(num_states);
  for (int i=0; i<num_states; i++){
    state_coordinate_names[i] = rs.robot->getStateName(i);
  }

  RobotStateDriver state_driver(state_coordinate_names);
  DrakeRobotState robot_state;

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

  rs.pos.resize(jointNames.size());
  rs.vel.resize(jointNames.size());
  rs.trq.resize(jointNames.size());

  while(1) {
    if (hasNewState) {
      robot_state.q.resize(rs.robot->num_positions);
      robot_state.qd.resize(rs.robot->num_velocities);
      state_driver.decode(&robot_state_msg, &robot_state);

      // copy pos, vel, trq and ft
      for (size_t i = 0; i < jointNames.size(); i++) {
        int idx = rs.joint_name_to_id.at(jointNames[i]);
        rs.pos[i] = robot_state_msg.position[idx];
        rs.vel[i] = robot_state_msg.position[idx];
        rs.trq[i] = robot_state_msg.position[idx];
      }

      rs.genKin(robot_state);
      
      hasNewState = false;
    }
    else {
      std::this_thread::yield();
    }
  }

  return 0;
}
