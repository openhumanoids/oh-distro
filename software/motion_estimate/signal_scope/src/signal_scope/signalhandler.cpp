#include "signalhandler.h"

#include "signaldata.h"
#include "signaldescription.h"
#include "jointnames.h"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <cassert>

#include <QDebug>


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
  (void)channel;

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

namespace
{
  int ArrayIndexFromIntegerKey(const QString& key)
  {
    bool ok;
    int arrayIndex = key.toInt(&ok);
    return  ok ? arrayIndex : -1;
  }

  int ArrayIndexFromJointNameKey(const QString& key)
  {
    return JointNames::indexOfJointName(key);
  }

  int ArrayIndexFromKey(const QString& key)
  {
    int arrayIndex = ArrayIndexFromIntegerKey(key);
    if (arrayIndex >= 0)
    {
      return arrayIndex;
    }

    arrayIndex = ArrayIndexFromJointNameKey(key);
    if (arrayIndex >= 0)
    {
      return arrayIndex;
    }

    qDebug() << "Failed to convert array key: " << key;
    assert(arrayIndex >= 0);
    return arrayIndex;
  }

  int ArrayIndexFromKeys(const QList<QString>& keys)
  {
    assert(keys.length() == 1);
    return ArrayIndexFromKey(keys[0]);
  }
}

#define define_array_handler(className, _fieldName) \
className::className(SignalDescription* desc) : SignalHandler(desc->mChannel), mArrayIndex(ArrayIndexFromKeys(desc->mArrayKeys)) { } \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, const drc::robot_state_t* msg, float& timeNow, float& signalValue) \
{ \
  (void)rbuf; \
  timeNow = (msg->utime)/1000000.0; \
  signalValue = msg->_fieldName[mArrayIndex]; \
  return true; \
} \
QString className::fieldName() { return #_fieldName ; } \
QString className::description() { return QString("%1.%2[%3]").arg(this->messageType()).arg(this->fieldName()).arg(this->mArrayIndex); }



define_array_handler(RobotStateJointPositionHandler, joint_position);
define_array_handler(RobotStateJointVelocityHandler, joint_velocity);
define_array_handler(RobotStateJointEffortHandler, joint_effort);

