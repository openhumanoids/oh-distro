#include "signalhandler.h"

#include "signaldata.h"
#include "signaldescription.h"

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


SignalHandlerFactory& SignalHandlerFactory::instance()
{
  static SignalHandlerFactory factory;

  bool registerHandlers = true;
  if (registerHandlers)
  {
    factory.registerClass<RobotStateJointPositionHandler>();
    factory.registerClass<RobotStateJointVelocityHandler>();
    factory.registerClass<RobotStateJointEffortHandler>();
  }

  return factory;
}

SignalHandler* SignalHandlerFactory::createHandler(SignalDescription* desc) const
{
  Constructor constructor = mConstructors.value(desc->mMessageType).value(desc->mFieldName);
  if (constructor == NULL)
  {
    return NULL;
  }
  return (*constructor)(desc);
}

//-----------------------------------------------------------------------------


#define define_array_handler(className, _fieldName) \
className::className(SignalDescription* desc) : SignalHandler(desc->mChannel), mArrayIndex(desc->mArrayIndex) { } \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue) \
{ \
  timeNow = (msg->utime)/1000000.0; \
  signalValue = msg->_fieldName[mArrayIndex]; \
  return true; \
} \
QString className::fieldName() { return #_fieldName ; }



define_array_handler(RobotStateJointPositionHandler, joint_position);
define_array_handler(RobotStateJointVelocityHandler, joint_velocity);
define_array_handler(RobotStateJointEffortHandler, joint_effort);

