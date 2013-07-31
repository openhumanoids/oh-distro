#include "signalhandler.h"

#include "signaldata.h"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

SignalHandler::SignalHandler(const QString& channel)
{
  mChannel = channel;
  mSignalData = new SignalData;
  mSubscription = 0;
}

SignalHandler::~SignalHandler()
{
  delete mSignalData;
}

void SignalHandler::handleRobotStateMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::robot_state_t* msg)
{
  float timeNow;
  float signalValue;

  bool valid = this->extractSignalData(rbuf, msg, timeNow, signalValue);
  if (valid)
  {
    const QPointF s(timeNow, signalValue);
    mSignalData->append(s);
  }
}

void SignalHandler::subscribe(lcm::LCM* lcmInstance)
{
  if (mSubscription)
  {
    printf("error: SignalHandler::subscribe() called without first calling unsubscribe.\n");
    return;
  }
  mSubscription = lcmInstance->subscribe(mChannel.toAscii().data(), &SignalHandler::handleRobotStateMessage, this);
}

void SignalHandler::unsubscribe(lcm::LCM* lcmInstance)
{
  lcmInstance->unsubscribe(mSubscription);
  mSubscription = 0;
}


//-----------------------------------------------------------------------------


#define define_array_handler(className, fieldName) \
className::className(const QString& channel, int arrayIndex) : SignalHandler(channel), mArrayIndex(arrayIndex) { } \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue) \
{ \
  timeNow = (msg->utime)/1000000.0; \
  signalValue = msg->fieldName[mArrayIndex]; \
  return true; \
}



define_array_handler(RobotStateJointPositionHandler, joint_position);
define_array_handler(RobotStateJointVelocityHandler, joint_velocity);
define_array_handler(RobotStateMeasuredEffortHandler, measured_effort);

/*
RobotJointPositionSignalHandler::RobotJointPositionSignalHandler(const QString& channel, int jointId) : SignalHandler(channel)
{
  mJointId = jointId;
}

bool RobotJointPositionSignalHandler::extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue)
{
  //static int64_t firstTime = 0;
  //if (firstTime == 0)
  //{
  //  firstTime = msg->utime;
  //}

  if (!msg->num_joints)
  {
    return false;
  }

  int jointId = 28;
  //float timeNow = (msg->utime - firstTime)/1000000.0;
  timeNow = (msg->utime)/1000000.0;
  signalValue = msg->joint_position[mJointId];

  //printf("time now: %f.  joint_position[0]: %f\n", timeNow, signalValue);
  return true;
}
*/

