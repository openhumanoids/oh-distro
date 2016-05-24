#include <mutex>
#include <thread>
// #include <chrono>
#include <atomic>
#include <sys/select.h>
#include "drake/lcmt_qp_controller_input.hpp"
#include "drc/controller_state_t.hpp"
#include "drc/controller_status_t.hpp"
#include "drc/recovery_trigger_t.hpp"
#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "drc/behavior_command_t.hpp"
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

struct ThreadedControllerOptions {
  std::string atlas_command_channel;
  std::string robot_behavior_channel;
  int max_infocount; // If we see info < 0 more than max_infocount times, freeze Atlas. Set to -1 to disable freezing.
  bool publishControllerState;
  bool fixedBase = false;
};

std::atomic<bool> done(false);

std::atomic<bool> newInputAvailable(false);
std::atomic<bool> newStateAvailable(false);

std::mutex pointerMutex;

std::shared_ptr<bot_core::robot_state_t> robot_state_msg;
std::shared_ptr<RobotStateDriver> state_driver;
std::shared_ptr<AtlasCommandDriver> command_driver;
std::shared_ptr<FootContactDriver> foot_contact_driver;
Matrix<bool, Dynamic, 1> b_contact_force;
VectorXd drake_input_to_robot_state;
std::vector<string> state_coordinate_names_shared;

class SolveArgs {
public:

  InstantaneousQPController *pdata;

  std::shared_ptr<DrakeRobotState> robot_state;
  std::shared_ptr<drake::lcmt_qp_controller_input> qp_input;
  std::shared_ptr<QPControllerOutput> qp_output;

  Matrix<bool, Dynamic, 1> b_contact_force;
  std::map<Side, ForceTorqueMeasurement> foot_force_torque_measurements;
  std::shared_ptr<QPControllerDebugData> debug;

  int info;
};

SolveArgs solveArgs;

drc::behavior_command_t robot_behavior_msg;

int infocount = 0;

std::string CONTROLLER_STATE_CHANNEL("CONTROLLER_STATE");


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

    sub = this->lcmHandler->LCMHandle->subscribe("QP_CONTROLLER_INPUT", &LCMControlReceiver::inputHandler, this);
    sub->setQueueCapacity(1);

    sub = this->lcmHandler->LCMHandle->subscribe("EST_ROBOT_STATE", &LCMControlReceiver::onRobotState, this);
    sub->setQueueCapacity(1);

    sub = this->lcmHandler->LCMHandle->subscribe("FOOT_CONTACT_ESTIMATE", &LCMControlReceiver::onFootContact, this);
    sub->setQueueCapacity(1);
  }

  void inputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drake::lcmt_qp_controller_input* msg)
  {
    //std::cout << "received qp_input on lcm thread " << std::this_thread::get_id() << std::endl;

    std::shared_ptr<drake::lcmt_qp_controller_input> msgCopy(new drake::lcmt_qp_controller_input);
    *msgCopy = *msg;

    // std::cout << "got input msg with param set name: " << msgCopy->param_set_name << std::endl;
    pointerMutex.lock();
    solveArgs.qp_input = msgCopy;
    pointerMutex.unlock();
    newInputAvailable = true;
  }

  void onRobotState(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg)
  {
    //std::cout << "received robotstate on lcm thread " << std::this_thread::get_id() << std::endl;

    std::shared_ptr<DrakeRobotState> state(new DrakeRobotState);

    int nq = solveArgs.pdata->getRobot().num_positions;
    int nv = solveArgs.pdata->getRobot().num_velocities;
    state->q = VectorXd::Zero(nq);
    state->qd = VectorXd::Zero(nv);

    state_driver->decode(msg, state.get());

    std::shared_ptr<bot_core::robot_state_t> msgCopy(new bot_core::robot_state_t);
    *msgCopy = *msg; // copy over contents of msg to new shared_ptr

    // std::cout << "decoded robot state " << state->t << std::endl;

    pointerMutex.lock();
    robot_state_msg = msgCopy;
    solveArgs.robot_state = state;

    const bot_core::force_torque_t& force_torque = msg->force_torque;

    solveArgs.foot_force_torque_measurements[Side::LEFT].frame_idx = solveArgs.pdata->getRPC().foot_ids.at(Side::LEFT); // TODO: make sure that this is right
    solveArgs.foot_force_torque_measurements[Side::LEFT].wrench << force_torque.l_foot_torque_x, force_torque.l_foot_torque_y, 0.0, 0.0, 0.0, force_torque.l_foot_force_z;

    solveArgs.foot_force_torque_measurements[Side::RIGHT].frame_idx = solveArgs.pdata->getRPC().foot_ids.at(Side::RIGHT); // TODO: make sure that this is right
    solveArgs.foot_force_torque_measurements[Side::RIGHT].wrench << force_torque.r_foot_torque_x, force_torque.r_foot_torque_y, 0.0, 0.0, 0.0, force_torque.r_foot_force_z;

    pointerMutex.unlock();
    newStateAvailable = true;
  }

  void onFootContact(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::foot_contact_estimate_t* msg)
  {
    Matrix<bool, Dynamic, 1> b_contact_force = Matrix<bool, Dynamic, 1>::Zero(solveArgs.pdata->getRobot().bodies.size());

    foot_contact_driver->decode(msg, b_contact_force);

    pointerMutex.lock();
    solveArgs.b_contact_force = b_contact_force;
    pointerMutex.unlock();
  }


  LCMHandler* lcmHandler;
};


LCMHandler lcmHandler;
LCMControlReceiver controlReceiver(&lcmHandler);

bool isOutputSafe(QPControllerOutput &qp_output) {
  for (int i=0; i < qp_output.q_ref.size(); i++) {
    if (std::isnan(qp_output.q_ref(i))) {
      std::cout << "Error: NaN in q_ref" << std::endl;
      return false;
    }
    if (std::isnan(qp_output.qd_ref(i))) {
      std::cout << "Error: NaN in qd_ref" << std::endl;
      return false;
    }
    if (std::isnan(qp_output.qdd(i))) {
      std::cout << "Error: NaN in qdd" << std::endl;
      return false;
    }
    if (std::isinf(qp_output.q_ref(i))) {
      std::cout << "Error: Inf in q_ref" << std::endl;
      return false;
    }
    if (std::isinf(qp_output.qd_ref(i))) {
      std::cout << "Error: Inf in qd_ref" << std::endl;
      return false;
    }
    if (std::isinf(qp_output.qdd(i))) {
      std::cout << "Error: Inf in qdd" << std::endl;
      return false;
    }
  }
  for (int i=0; i < qp_output.u.size(); i++) {
    if (std::isnan(qp_output.u(i))) {
      std::cout << "Error: NaN in u" << std::endl;
      return false;
    }
    if (std::isinf(qp_output.u(i))) {
      std::cout << "Error: Inf in u" << std::endl;
      return false;
    }
  }
  return true;
}

drc::controller_state_t encodeControllerState(double t, int num_joints, const QPControllerOutput &qp_output){

  drc::controller_state_t msg;
  msg.timestamp = (long)(t*1000000);
  msg.num_joints = num_joints;
  msg.q_integrator_state.resize(num_joints);
  msg.vref_integrator_state.resize(num_joints);
  msg.q_ref.resize(num_joints);
  msg.qd_ref.resize(num_joints);
  msg.qdd.resize(num_joints);
  msg.u.resize(num_joints);
  msg.joint_name.resize(num_joints);


  // std::cout << "size of qp_output.u is " << qp_output.u.size() << std::endl;
  // std::cout << "size of qp_output.qdd is " << qp_output.qdd.size() << std::endl;
  // std::cout << "size of state coordinate names are" << solveArgs.pdata->state_coordinate_names.size() << std::endl;

  const QPControllerState& controller_state = solveArgs.pdata->getControllerState();

  for(int i=0; i<num_joints; i++){
    msg.q_integrator_state[i] = controller_state.q_integrator_state(i);
    msg.vref_integrator_state[i] = qp_output.qd_ref(i);
    msg.q_ref[i] = qp_output.q_ref(i);
    msg.qd_ref[i] = qp_output.qd_ref(i);
    msg.qdd[i] = qp_output.qdd(i);
    msg.joint_name[i] = state_coordinate_names_shared[i];
  }

  //Inputs are in a different order, need to deal with that
  int num_inputs = drake_input_to_robot_state.size();
  int robot_state_idx;
  for( int i=0; i<num_inputs; i++){
    robot_state_idx = drake_input_to_robot_state[i];
    msg.u[robot_state_idx] = qp_output.u(i);
  }

  return msg;
}

bot_core::robot_state_t encodeControllerQDes(double t, int num_joints, const QPControllerOutput &qp_output){
  bot_core::robot_state_t msg;
  msg.utime = (long)(t*1000000);
  msg.num_joints = num_joints;
  msg.joint_name.resize(num_joints);
  msg.joint_position.resize(num_joints);
  msg.joint_velocity.resize(num_joints);
  msg.joint_effort.resize(num_joints);

  for(int i = 0; i < num_joints; i++){
    msg.joint_position[i] = qp_output.q_des(i);
    msg.joint_velocity[i] = qp_output.qdot_des(i);
    msg.joint_name[i] = state_coordinate_names_shared[i];
  }

  return msg;
}

void getDrakeInputToRobotStateIndexMap(const std::vector<std::string> &input_joint_names, const std::vector<std::string> & state_coordinate_names,
                                       VectorXd & drake_input_to_robot_state) {
  int num_inputs = input_joint_names.size();
  drake_input_to_robot_state.resize(num_inputs);

  bool has_match;
  for (int i = 0; i < num_inputs; i++) {
    has_match = false;
    for (int j = 0; j < state_coordinate_names.size(); j++) {
      if (input_joint_names[i].compare(state_coordinate_names[j]) == 0) {
        has_match = true;
        drake_input_to_robot_state(i) = j;
      }
    }
    if (!has_match) {
      std::cout << "Could not match joint " << input_joint_names[i] << std::endl;
      throw std::runtime_error("Could not find a match for drake input joint name");
    }
  }
}

void threadLoop(std::shared_ptr<ThreadedControllerOptions> ctrl_opts) {

  QPControllerOutput qp_output;
  done = false;

  // original gravity vector
  Eigen::Matrix<double, TWIST_SIZE, 1> a_grav;
  a_grav << 0, 0, 0, 0, 0, -9.81;

  while (!done) {

    //std::cout << "waiting for new data... " << std::this_thread::get_id() << std::endl;

    while (!newStateAvailable || !newInputAvailable) {
      std::this_thread::yield();
    }

    // auto begin = std::chrono::high_resolution_clock::now();


    // copy pointers
    pointerMutex.lock();
    std::shared_ptr <DrakeRobotState> robot_state = solveArgs.robot_state;
    std::shared_ptr <drake::lcmt_qp_controller_input> qp_input = solveArgs.qp_input;
    b_contact_force = solveArgs.b_contact_force;
    std::map <Side, ForceTorqueMeasurement> foot_force_torque_measurements = solveArgs.foot_force_torque_measurements;
    std::shared_ptr<bot_core::robot_state_t> state_msg = robot_state_msg;
    pointerMutex.unlock();

    // newInputAvailable = false;
    newStateAvailable = false;

    //change the direction of the gravity
    if(ctrl_opts->fixedBase){
      // adjust the gravity vector!!!!
      Vector3d grav(0,0,-9.81);
      bot_core::quaternion_t quatMsg = state_msg->pose.rotation;
      Vector4d robotToWorldQuat(quatMsg.w, quatMsg.x, quatMsg.y, quatMsg.z);
      Vector4d worldToRobotQuat = quatConjugate(robotToWorldQuat);
      Vector3d transformedGrav = quatRotateVec(worldToRobotQuat,grav);
      a_grav.tail(3) = transformedGrav;

      // apply this transformed gravity to the RigidBodyTree
      solveArgs.pdata->robot->a_grav = a_grav;
    }

    // std::cout << "calling solve " << std::endl;

    if (qp_input->be_silent) {
      // Act as a dummy controller, produce no ATLAS_COMMAND, and reset all integrator states
      solveArgs.pdata->resetControllerState(robot_state->t);
      infocount = 0;
    } else {
/*
    onst drake::lcmt_qp_controller_input& qp_input,
  const DrakeRobotState& robot_state,
  const Ref<const Matrix<bool, Dynamic, 1>>& contact_detected,
  const std::map<Side, ForceTorqueMeasurement>&
      foot_force_torque_measurements,
  QPControllerOutput& qp_output, QPControllerDebugData* debug
*/
      int info = solveArgs.pdata->setupAndSolveQP(*qp_input, *robot_state, b_contact_force,
                                                  foot_force_torque_measurements, qp_output, &(*(solveArgs.debug)));

      if (!isOutputSafe(qp_output)) {
        // First priority is to halt unsafe behavior
        robot_behavior_msg.utime = 0;
        robot_behavior_msg.command = "freeze";
        lcmHandler.LCMHandle->publish(ctrl_opts->robot_behavior_channel, &robot_behavior_msg);
      }

      if (info < 0 && ctrl_opts->max_infocount > 0) {
        infocount++;
        std::cout << "Infocount incremented " << infocount << std::endl;
        if (infocount >= ctrl_opts->max_infocount) {
          if (infocount == ctrl_opts->max_infocount) {
            std::cout << "Infocount exceeded. Freezing Atlas!" << std::endl;
          }
          /*
          robot_behavior_msg.utime = 0;
          robot_behavior_msg.command = "freeze";
          lcmHandler.LCMHandle->publish(ctrl_opts->robot_behavior_channel, &robot_behavior_msg);
          */
          // we've lost control and are probably falling. cross fingers...
          drc::recovery_trigger_t trigger_msg;
          trigger_msg.utime = static_cast<int64_t> (robot_state->t * 1e6);
          trigger_msg.activate = true;
          trigger_msg.override = true;
          lcmHandler.LCMHandle->publish("BRACE_FOR_FALL", &trigger_msg);
        }
      } else {
        infocount = 0;
      }
      // std::cout << "u: " << qp_output.u << std::endl;
      // std::cout << "q: " << qp_output.q_ref << std::endl;
      // std::cout << "qd: " << qp_output.qd_ref << std::endl;
      // std::cout << "qdd: " << qp_output.qdd << std::endl;

      const QPControllerParams params = solveArgs.pdata->getParamSet(qp_input->param_set_name);

      // publish ATLAS_COMMAND
      bot_core::atlas_command_t *command_msg = command_driver->encode(robot_state->t, &qp_output, params.hardware);
      lcmHandler.LCMHandle->publish(ctrl_opts->atlas_command_channel, command_msg); // publishes the atlas_command msg

      //publish CONTROLLER_STATE lcm message for debugging purposes
      if (ctrl_opts->publishControllerState) {
        int num_joints = qp_output.q_ref.size();
        drc::controller_state_t controller_state_msg = encodeControllerState(robot_state->t, num_joints,
                                                                             qp_output);
        lcmHandler.LCMHandle->publish(CONTROLLER_STATE_CHANNEL, &controller_state_msg);

        bot_core::robot_state_t controller_qdes_msg = encodeControllerQDes(robot_state->t, num_joints, qp_output);
        lcmHandler.LCMHandle->publish("CONTROLLER_Q_DES", &controller_qdes_msg);
      }
    }
  }
}



  void controllerLoop(InstantaneousQPController *pdata, std::shared_ptr<ThreadedControllerOptions> ctrl_opts)
  {
    int num_states = pdata->getRobot().num_positions + pdata->getRobot().num_velocities;
    std::vector<string> state_coordinate_names(num_states);
    for (int i=0; i<num_states; i++){
      state_coordinate_names[i] = pdata->getRobot().getStateName(i);
    }
    state_driver.reset(new RobotStateDriver(state_coordinate_names));
    command_driver.reset(new AtlasCommandDriver(&(pdata->getJointNames()), state_coordinate_names));
    foot_contact_driver.reset(new FootContactDriver(pdata->getRPC()));

    solveArgs.pdata = pdata;
    solveArgs.b_contact_force = Matrix<bool, Dynamic, 1>::Zero(pdata->getRobot().bodies.size());


    std::vector<std::string> input_joint_names;

    auto & actuatorVec = pdata->getRobot().actuators;
    for(auto & actuator: actuatorVec){
      std::string actuatorName = actuator.body->getJoint().getName();
      std::cout << actuatorName << std::endl;
      input_joint_names.push_back(actuatorName);
    }
    //initialize the drake coordinate names
    getDrakeInputToRobotStateIndexMap(input_joint_names, state_coordinate_names, drake_input_to_robot_state);


    lcmHandler.Start();
    controlReceiver.InitSubscriptions();

    // so that the encodeControllerState method will have access to them
    state_coordinate_names_shared = state_coordinate_names;

    // std::cout << "pdata num bodies: " << pdata->getRobot().bodies.size() << std::endl;

    threadLoop(ctrl_opts);
  }


} // end anonymous namespace




