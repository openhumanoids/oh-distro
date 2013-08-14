#include "signalhandler.h"

#include "signaldata.h"
#include "signaldescription.h"
#include "jointnames.h"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <cassert>

#include <QDebug>





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

  int ArrayIndexFromKeys(const QList<QString>& keys, int keyIndex)
  {
    assert(keys.length() > keyIndex);
    return ArrayIndexFromKey(keys[keyIndex]);
  }
}






#define define_array_handler(className, _messageType, _fieldName) \
declare_signal_handler(className); \
className::className(SignalDescription* desc) : SignalHandler(desc) \
{ \
   mArrayIndex = ArrayIndexFromKeys(desc->mArrayKeys, 0); \
   mArrayKey = desc->mArrayKeys[0]; \
} \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
{ \
  drc::_messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  timeNow = (msg.utime)/1000000.0; \
  signalValue = msg._fieldName[mArrayIndex]; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName ; } \
QString className::description() { return QString("%1.%2[%3]").arg(this->messageType()).arg(this->fieldName()).arg(this->mArrayKey); }


#define define_array_array_handler(className, _messageType, _fieldName1, _fieldName2) \
declare_signal_handler(className); \
className::className(SignalDescription* desc) : SignalHandler(desc) \
{ \
   mArrayIndex = ArrayIndexFromKeys(desc->mArrayKeys, 0); \
   mArrayIndex2 = ArrayIndexFromKeys(desc->mArrayKeys, 1); \
   mArrayKey = desc->mArrayKeys[0]; \
   mArrayKey2 = desc->mArrayKeys[1]; \
} \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
{ \
  drc::_messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  timeNow = (msg.utime)/1000000.0; \
  signalValue = msg._fieldName1[mArrayIndex]._fieldName2[mArrayIndex2]; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2 ; } \
QString className::description() { return QString("%1.%2[%3].%4[%5]").arg(this->messageType()).arg(#_fieldName1).arg(this->mArrayKey).arg(#_fieldName2).arg(this->mArrayKey2); }

#define define_field_field_handler(className, _messageType, _fieldName1, _fieldName2) \
declare_signal_handler(className); \
className::className(SignalDescription* desc) : SignalHandler(desc) { } \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
{ \
  drc::_messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  timeNow = (msg.utime)/1000000.0; \
  signalValue = msg._fieldName1._fieldName2; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2 ; } \
QString className::description() { return QString("%1.%2").arg(this->messageType()).arg(this->fieldName()); }


#define define_field_field_field_handler(className, _messageType, _fieldName1, _fieldName2, _fieldName3) \
declare_signal_handler(className); \
className::className(SignalDescription* desc) : SignalHandler(desc) { } \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
{ \
  drc::_messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  timeNow = (msg.utime)/1000000.0; \
  signalValue = msg._fieldName1._fieldName2._fieldName3; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2"."#_fieldName3 ; } \
QString className::description() { return QString("%1.%2").arg(this->messageType()).arg(this->fieldName()); }


// robot_state_t

define_array_handler(RobotStateJointPositionHandler, robot_state_t, joint_position);
define_array_handler(RobotStateJointVelocityHandler, robot_state_t, joint_velocity);
define_array_handler(RobotStateJointEffortHandler, robot_state_t, joint_effort);

define_field_field_field_handler(RobotStatePoseTranslationXHandler, robot_state_t, pose, translation, x);
define_field_field_field_handler(RobotStatePoseTranslationYHandler, robot_state_t, pose, translation, y);
define_field_field_field_handler(RobotStatePoseTranslationZHandler, robot_state_t, pose, translation, z);

define_field_field_field_handler(RobotStatePoseRotationWHandler, robot_state_t, pose, rotation, w);
define_field_field_field_handler(RobotStatePoseRotationXHandler, robot_state_t, pose, rotation, x);
define_field_field_field_handler(RobotStatePoseRotationYHandler, robot_state_t, pose, rotation, y);
define_field_field_field_handler(RobotStatePoseRotationZHandler, robot_state_t, pose, rotation, z);

define_field_field_field_handler(RobotStateTwistLinearVelocityXHandler, robot_state_t, twist, linear_velocity, x);
define_field_field_field_handler(RobotStateTwistLinearVelocityYHandler, robot_state_t, twist, linear_velocity, y);
define_field_field_field_handler(RobotStateTwistLinearVelocityZHandler, robot_state_t, twist, linear_velocity, z);

define_field_field_field_handler(RobotStateTwistAngularVelocityXHandler, robot_state_t, twist, angular_velocity, x);
define_field_field_field_handler(RobotStateTwistAngularVelocityYHandler, robot_state_t, twist, angular_velocity, y);
define_field_field_field_handler(RobotStateTwistAngularVelocityZHandler, robot_state_t, twist, angular_velocity, z);

define_field_field_handler(RobotStateForceTorqueLFootForceZHandler, robot_state_t, force_torque, l_foot_force_z);
define_field_field_handler(RobotStateForceTorqueLFootTorqueXHandler, robot_state_t, force_torque, l_foot_torque_x);
define_field_field_handler(RobotStateForceTorqueLFootTorqueYHandler, robot_state_t, force_torque, l_foot_torque_y);

define_field_field_handler(RobotStateForceTorqueRFootForceZHandler, robot_state_t, force_torque, r_foot_force_z);
define_field_field_handler(RobotStateForceTorqueRFootTorqueXHandler, robot_state_t, force_torque, r_foot_torque_x);
define_field_field_handler(RobotStateForceTorqueRFootTorqueYHandler, robot_state_t, force_torque, r_foot_torque_y);



// atlas_state_t

define_array_handler(AtlasStateJointPositionHandler, atlas_state_t, joint_position);
define_array_handler(AtlasStateJointVelocityHandler, atlas_state_t, joint_velocity);
define_array_handler(AtlasStateJointEffortHandler, atlas_state_t, joint_effort);

define_field_field_handler(AtlasStateForceTorqueLFootForceZHandler, atlas_state_t, force_torque, l_foot_force_z);
define_field_field_handler(AtlasStateForceTorqueLFootTorqueXHandler, atlas_state_t, force_torque, l_foot_torque_x);
define_field_field_handler(AtlasStateForceTorqueLFootTorqueYHandler, atlas_state_t, force_torque, l_foot_torque_y);

define_field_field_handler(AtlasStateForceTorqueRFootForceZHandler, atlas_state_t, force_torque, r_foot_force_z);
define_field_field_handler(AtlasStateForceTorqueRFootTorqueXHandler, atlas_state_t, force_torque, r_foot_torque_x);
define_field_field_handler(AtlasStateForceTorqueRFootTorqueYHandler, atlas_state_t, force_torque, r_foot_torque_y);


// atlas_raw_imu_batch_t

define_array_array_handler(AtlasRawIMUBatchIMUDeltaRotation, atlas_raw_imu_batch_t, raw_imu, delta_rotation);
define_array_array_handler(AtlasRawIMUBatchIMULinearAcceleration, atlas_raw_imu_batch_t, raw_imu, linear_acceleration);


SignalHandler::SignalHandler(SignalDescription* signalDescription)
{
  assert(signalDescription != 0);
  mDescription = *signalDescription;
  mSignalData = new SignalData;
  mSubscription = 0;
}

SignalHandler::~SignalHandler()
{
  delete mSignalData;
}

void SignalHandler::handleRobotStateMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
{
  float timeNow;
  float signalValue;
  (void)channel;

  bool valid = this->extractSignalData(rbuf, timeNow, signalValue);
  if (valid)
  {
    const QPointF s(timeNow, signalValue);
    mSignalData->append(s);
  }
  else
  {
    mSignalData->flagMessageError();
  }
}

void SignalHandler::subscribe(lcm::LCM* lcmInstance)
{
  if (mSubscription)
  {
    printf("error: SignalHandler::subscribe() called without first calling unsubscribe.\n");
    return;
  }
  mSubscription = lcmInstance->subscribe(this->channel().toAscii().data(), &SignalHandler::handleRobotStateMessage, this);
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
    factory.registerClass<RobotStatePoseTranslationXHandler>();
    factory.registerClass<RobotStatePoseTranslationYHandler>();
    factory.registerClass<RobotStatePoseTranslationZHandler>();
    factory.registerClass<RobotStatePoseRotationWHandler>();
    factory.registerClass<RobotStatePoseRotationXHandler>();
    factory.registerClass<RobotStatePoseRotationYHandler>();
    factory.registerClass<RobotStatePoseRotationZHandler>();
    factory.registerClass<RobotStateTwistLinearVelocityXHandler>();
    factory.registerClass<RobotStateTwistLinearVelocityYHandler>();
    factory.registerClass<RobotStateTwistLinearVelocityZHandler>();
    factory.registerClass<RobotStateTwistAngularVelocityXHandler>();
    factory.registerClass<RobotStateTwistAngularVelocityYHandler>();
    factory.registerClass<RobotStateTwistAngularVelocityZHandler>();
    factory.registerClass<RobotStateForceTorqueLFootForceZHandler>();
    factory.registerClass<RobotStateForceTorqueLFootTorqueXHandler>();
    factory.registerClass<RobotStateForceTorqueLFootTorqueYHandler>();
    factory.registerClass<RobotStateForceTorqueRFootForceZHandler>();
    factory.registerClass<RobotStateForceTorqueRFootTorqueXHandler>();
    factory.registerClass<RobotStateForceTorqueRFootTorqueYHandler>();
    factory.registerClass<AtlasStateJointPositionHandler>();
    factory.registerClass<AtlasStateJointVelocityHandler>();
    factory.registerClass<AtlasStateJointEffortHandler>();
    factory.registerClass<AtlasStateForceTorqueLFootForceZHandler>();
    factory.registerClass<AtlasStateForceTorqueLFootTorqueXHandler>();
    factory.registerClass<AtlasStateForceTorqueLFootTorqueYHandler>();
    factory.registerClass<AtlasStateForceTorqueRFootForceZHandler>();
    factory.registerClass<AtlasStateForceTorqueRFootTorqueXHandler>();
    factory.registerClass<AtlasStateForceTorqueRFootTorqueYHandler>();
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

