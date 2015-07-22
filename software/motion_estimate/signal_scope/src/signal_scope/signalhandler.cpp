#include "signalhandler.h"

#include "signaldata.h"
#include "signaldescription.h"
#include "jointnames.h"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/microstrain_comm.hpp>
#include <lcmtypes/vicon.hpp>
#include "lcmtypes/robotiqhand/status_t.hpp"
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "lcmtypes/drc/atlas_foot_pos_est_t.hpp"
#include "lcmtypes/drc/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_state_extra_t.hpp"
#include "lcmtypes/drc/atlas_status_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/six_axis_force_torque_t.hpp"
#include "lcmtypes/drc/controller_debug_t.hpp"
#include "lcmtypes/mav/filter_state_t.hpp"
#include "lcmtypes/drc/double_array_t.hpp"

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

  QList<QString> createIndexList(int size)
  {
    QList<QString> indexList;
    for (int i = 0; i < size; ++i)
    {
      indexList << QString::number(i);
    }
    return indexList;
  }

  int64_t timeOffset = 0;
  // #define timeOffset 1376452800000000  // start of August 14th in microseconds
}



//-----------------------------------------------------------------------------
#define compute_time_now \
  if (timeOffset == 0) timeOffset = msg.utime; \
  timeNow = (msg.utime - timeOffset)/1000000.0;


#define default_array_keys_function(className) \
  QList<QList<QString> > className::validArrayKeys() \
  { \
    return QList<QList<QString> >(); \
  }


//-----------------------------------------------------------------------------
#define define_array_handler(className, _messageType, _fieldName, _arrayKeyFunction) \
declare_signal_handler(className); \
className::className(const SignalDescription* desc) : SignalHandler(desc) \
{ \
   mArrayIndex = ArrayIndexFromKeys(desc->mArrayKeys, 0); \
   mArrayKey = desc->mArrayKeys[0]; \
} \
QList<QList<QString> > className::validArrayKeys() \
{ \
  QList<QList<QString> > arrayKeys; \
  arrayKeys << _arrayKeyFunction; \
  return arrayKeys; \
} \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) \
{ \
  _messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  compute_time_now \
  signalValue = msg._fieldName[mArrayIndex]; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName ; } \
QString className::description() { return QString("%1.%2[%3]").arg(this->messageType()).arg(this->fieldName()).arg(this->mArrayKey); }


//-----------------------------------------------------------------------------
#define define_array_array_handler(className, _messageType, _fieldName1, _fieldName2, _arrayKeyFunction1, _arrayKeyFunction2) \
declare_signal_handler(className); \
className::className(const SignalDescription* desc) : SignalHandler(desc) \
{ \
   mArrayIndex = ArrayIndexFromKeys(desc->mArrayKeys, 0); \
   mArrayIndex2 = ArrayIndexFromKeys(desc->mArrayKeys, 1); \
   mArrayKey = desc->mArrayKeys[0]; \
   mArrayKey2 = desc->mArrayKeys[1]; \
} \
QList<QList<QString> > className::validArrayKeys() \
{ \
  QList<QList<QString> > arrayKeys; \
  arrayKeys << _arrayKeyFunction1 << _arrayKeyFunction2; \
  return arrayKeys; \
} \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) \
{ \
  _messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  compute_time_now \
  signalValue = msg._fieldName1[mArrayIndex]._fieldName2[mArrayIndex2]; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2 ; } \
QString className::description() { return QString("%1.%2[%3].%4[%5]").arg(this->messageType()).arg(#_fieldName1).arg(this->mArrayKey).arg(#_fieldName2).arg(this->mArrayKey2); }


//-----------------------------------------------------------------------------
#define define_field_array_handler(className, _messageType, _fieldName1, _fieldName2, _arrayKeyFunction) \
declare_signal_handler(className); \
className::className(const SignalDescription* desc) : SignalHandler(desc) \
{ \
   mArrayIndex = ArrayIndexFromKeys(desc->mArrayKeys, 0); \
   mArrayKey = desc->mArrayKeys[0]; \
} \
QList<QList<QString> > className::validArrayKeys() \
{ \
  QList<QList<QString> > arrayKeys; \
  arrayKeys << _arrayKeyFunction; \
  return arrayKeys; \
} \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) \
{ \
  _messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  compute_time_now \
  signalValue = msg._fieldName1._fieldName2[mArrayIndex]; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2 ; } \
QString className::description() { return QString("%1.%2.%3[%4]").arg(this->messageType()).arg(#_fieldName1).arg(#_fieldName2).arg(this->mArrayKey); }


//-----------------------------------------------------------------------------
#define define_field_handler(className, _messageType, _fieldName) \
declare_signal_handler(className); \
className::className(const SignalDescription* desc) : SignalHandler(desc) { } \
default_array_keys_function(className) \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) \
{ \
  _messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  compute_time_now \
  signalValue = msg._fieldName; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName ; } \
QString className::description() { return QString("%1.%2").arg(this->messageType()).arg(this->fieldName()); }


//-----------------------------------------------------------------------------
#define define_field_field_handler(className, _messageType, _fieldName1, _fieldName2) \
declare_signal_handler(className); \
className::className(const SignalDescription* desc) : SignalHandler(desc) { } \
default_array_keys_function(className) \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) \
{ \
  _messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  compute_time_now \
  signalValue = msg._fieldName1._fieldName2; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2 ; } \
QString className::description() { return QString("%1.%2").arg(this->messageType()).arg(this->fieldName()); }

//-----------------------------------------------------------------------------
#define define_field_field_field_handler(className, _messageType, _fieldName1, _fieldName2, _fieldName3) \
declare_signal_handler(className); \
className::className(const SignalDescription* desc) : SignalHandler(desc) { } \
default_array_keys_function(className) \
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, double& timeNow, double& signalValue) \
{ \
  _messageType msg; \
  if (msg.decode(rbuf->data, 0, 1000000) < 0) \
  { \
    return false;\
  } \
  compute_time_now \
  signalValue = msg._fieldName1._fieldName2._fieldName3; \
  return true; \
} \
QString className::messageType() { return #_messageType ; } \
QString className::fieldName() { return #_fieldName1"."#_fieldName2"."#_fieldName3 ; } \
QString className::description() { return QString("%1.%2").arg(this->messageType()).arg(this->fieldName()); }


// robot_state_t

define_array_handler(RobotStateJointPositionHandler, drc::robot_state_t, joint_position, JointNames::jointNames());
define_array_handler(RobotStateJointVelocityHandler, drc::robot_state_t, joint_velocity, JointNames::jointNames());
define_array_handler(RobotStateJointEffortHandler, drc::robot_state_t, joint_effort, JointNames::jointNames());

define_field_field_field_handler(RobotStatePoseTranslationXHandler, drc::robot_state_t, pose, translation, x);
define_field_field_field_handler(RobotStatePoseTranslationYHandler, drc::robot_state_t, pose, translation, y);
define_field_field_field_handler(RobotStatePoseTranslationZHandler, drc::robot_state_t, pose, translation, z);

define_field_field_field_handler(RobotStatePoseRotationWHandler, drc::robot_state_t, pose, rotation, w);
define_field_field_field_handler(RobotStatePoseRotationXHandler, drc::robot_state_t, pose, rotation, x);
define_field_field_field_handler(RobotStatePoseRotationYHandler, drc::robot_state_t, pose, rotation, y);
define_field_field_field_handler(RobotStatePoseRotationZHandler, drc::robot_state_t, pose, rotation, z);

define_field_field_field_handler(RobotStateTwistLinearVelocityXHandler, drc::robot_state_t, twist, linear_velocity, x);
define_field_field_field_handler(RobotStateTwistLinearVelocityYHandler, drc::robot_state_t, twist, linear_velocity, y);
define_field_field_field_handler(RobotStateTwistLinearVelocityZHandler, drc::robot_state_t, twist, linear_velocity, z);

define_field_field_field_handler(RobotStateTwistAngularVelocityXHandler, drc::robot_state_t, twist, angular_velocity, x);
define_field_field_field_handler(RobotStateTwistAngularVelocityYHandler, drc::robot_state_t, twist, angular_velocity, y);
define_field_field_field_handler(RobotStateTwistAngularVelocityZHandler, drc::robot_state_t, twist, angular_velocity, z);

define_field_field_handler(RobotStateForceTorqueLFootForceZHandler, drc::robot_state_t, force_torque, l_foot_force_z);
define_field_field_handler(RobotStateForceTorqueLFootTorqueXHandler, drc::robot_state_t, force_torque, l_foot_torque_x);
define_field_field_handler(RobotStateForceTorqueLFootTorqueYHandler, drc::robot_state_t, force_torque, l_foot_torque_y);

define_field_field_handler(RobotStateForceTorqueRFootForceZHandler, drc::robot_state_t, force_torque, r_foot_force_z);
define_field_field_handler(RobotStateForceTorqueRFootTorqueXHandler, drc::robot_state_t, force_torque, r_foot_torque_x);
define_field_field_handler(RobotStateForceTorqueRFootTorqueYHandler, drc::robot_state_t, force_torque, r_foot_torque_y);

define_field_array_handler(RobotStateForceTorqueLHandForceHandler, drc::robot_state_t, force_torque, l_hand_force, createIndexList(3));
define_field_array_handler(RobotStateForceTorqueLHandTorqueHandler, drc::robot_state_t, force_torque, l_hand_torque, createIndexList(3));
define_field_array_handler(RobotStateForceTorqueRHandForceHandler, drc::robot_state_t, force_torque, r_hand_force, createIndexList(3));
define_field_array_handler(RobotStateForceTorqueRHandTorqueHandler, drc::robot_state_t, force_torque, r_hand_torque, createIndexList(3));



// atlas_state_t

define_array_handler(AtlasStateJointPositionHandler, drc::atlas_state_t, joint_position, JointNames::jointNames());
define_array_handler(AtlasStateJointVelocityHandler, drc::atlas_state_t, joint_velocity, JointNames::jointNames());
define_array_handler(AtlasStateJointEffortHandler, drc::atlas_state_t, joint_effort, JointNames::jointNames());

define_field_field_handler(AtlasStateForceTorqueLFootForceZHandler, drc::atlas_state_t, force_torque, l_foot_force_z);
define_field_field_handler(AtlasStateForceTorqueLFootTorqueXHandler, drc::atlas_state_t, force_torque, l_foot_torque_x);
define_field_field_handler(AtlasStateForceTorqueLFootTorqueYHandler, drc::atlas_state_t, force_torque, l_foot_torque_y);

define_field_field_handler(AtlasStateForceTorqueRFootForceZHandler, drc::atlas_state_t, force_torque, r_foot_force_z);
define_field_field_handler(AtlasStateForceTorqueRFootTorqueXHandler, drc::atlas_state_t, force_torque, r_foot_torque_x);
define_field_field_handler(AtlasStateForceTorqueRFootTorqueYHandler, drc::atlas_state_t, force_torque, r_foot_torque_y);

// atlas_state_extra_t

define_array_handler(AtlasStateExtraJointPositionOutHandler, drc::atlas_state_extra_t, joint_position_out, JointNames::jointNames());
define_array_handler(AtlasStateExtraJointVelocityOutHandler, drc::atlas_state_extra_t, joint_velocity_out, JointNames::jointNames());
define_array_handler(AtlasStateExtraJointPressurePosHandler, drc::atlas_state_extra_t, psi_pos, JointNames::jointNames());
define_array_handler(AtlasStateExtraJointPressureNegHandler, drc::atlas_state_extra_t, psi_neg, JointNames::jointNames());

// atlas_raw_imu_batch_t

define_array_array_handler(AtlasRawIMUBatchIMUDeltaRotation, drc::atlas_raw_imu_batch_t, raw_imu, delta_rotation,  createIndexList(15), createIndexList(3));
define_array_array_handler(AtlasRawIMUBatchIMULinearAcceleration, drc::atlas_raw_imu_batch_t, raw_imu, linear_acceleration, createIndexList(15), createIndexList(3));

// atlas_raw_imu_t ( a single message broken out from the above message )
define_array_handler(AtlasRawIMUPacketDeltaRotation, drc::atlas_raw_imu_t, delta_rotation, createIndexList(3));
define_array_handler(AtlasRawIMUPacketLinearAcceleration, drc::atlas_raw_imu_t, linear_acceleration, createIndexList(3));

// atlas_raw_imu_t ( a single message broken out from the above message )
define_array_handler(MicrostrainINSGyro, microstrain::ins_t, gyro, createIndexList(3));
define_array_handler(MicrostrainINSAccel, microstrain::ins_t, accel, createIndexList(3));

// pose_t
define_array_handler(PoseTypePositionHandler, bot_core::pose_t, pos, createIndexList(3));
define_array_handler(PoseTypeVelocityHandler, bot_core::pose_t, vel, createIndexList(3));
define_array_handler(PoseTypeOrientationHandler, bot_core::pose_t, orientation, createIndexList(4));
define_array_handler(PoseTypeRotationRateHandler, bot_core::pose_t, rotation_rate, createIndexList(3));
define_array_handler(PoseTypeAcceleration, bot_core::pose_t, accel, createIndexList(3));

// six_axis_force_torque_t
define_array_handler(SixAxisForceTorqueTypeForceHandler, drc::six_axis_force_torque_t, force, createIndexList(3));
define_array_handler(SixAxisForceTorqueTypeMomentHandler, drc::six_axis_force_torque_t, moment, createIndexList(3));

// atlas_foot_pos_est_t
define_array_handler(AtlasFootPosEstLeftPositionHandler, drc::atlas_foot_pos_est_t, left_position, createIndexList(3));
define_array_handler(AtlasFootPosEstRightPositionHandler, drc::atlas_foot_pos_est_t, right_position, createIndexList(3));


// vicon body_t
define_array_handler(ViconBodyTransHandler, vicon::body_t, trans, createIndexList(3));
define_array_handler(ViconBodyQuatHandler, vicon::body_t, quat, createIndexList(4));

// atlas_status_t
define_field_handler(AtlasStatusPumpInletPressure, drc::atlas_status_t, pump_inlet_pressure);
define_field_handler(AtlasStatusPumpSupplyPressure, drc::atlas_status_t, pump_supply_pressure);
define_field_handler(AtlasStatusPumpReturnPressure, drc::atlas_status_t, pump_return_pressure);
define_field_handler(AtlasStatusAirSumpPressure, drc::atlas_status_t, air_sump_pressure);
define_field_handler(AtlasStatusPumpRPM, drc::atlas_status_t, current_pump_rpm);
define_field_handler(AtlasStatusBehavior, drc::atlas_status_t, behavior);

// controller_debug_t
define_field_handler(ControllerDebugRightFoot, drc::controller_debug_t, r_foot_contact);
define_field_handler(ControllerDebugLeftFoot, drc::controller_debug_t, l_foot_contact);
define_field_handler(ControllerDebugSolverInfo, drc::controller_debug_t, info);
define_array_handler(ControllerDebugQddDes, drc::controller_debug_t, qddot_des, createIndexList(34));
define_array_handler(ControllerDebugU, drc::controller_debug_t, u, createIndexList(28));
define_array_handler(ControllerDebugAlpha, drc::controller_debug_t, alpha, createIndexList(90));
define_array_handler(ControllerDebugZMPerr, drc::controller_debug_t, zmp_err, createIndexList(2));

/*
define_field_array_handler(AtlasControlJointsPositionHandler, drc::atlas_control_data_t, joints, position, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsVelocityHandler, drc::atlas_control_data_t, joints, velocity, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsEffortHandler, drc::atlas_control_data_t, joints, effort, JointNames::jointNames());

define_field_array_handler(AtlasControlJointsKQPHandler, drc::atlas_control_data_t, joints, k_q_p, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsKQIHandler, drc::atlas_control_data_t, joints, k_q_i, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsKQDPHandler, drc::atlas_control_data_t, joints, k_qd_p, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsKFPHandler, drc::atlas_control_data_t, joints, k_f_p, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsFFQDHandler, drc::atlas_control_data_t, joints, ff_qd, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsFFQDDPHandler, drc::atlas_control_data_t, joints, ff_qd_d, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsFFFDHandler, drc::atlas_control_data_t, joints, ff_f_d, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsFFConstHandler, drc::atlas_control_data_t, joints, ff_const, JointNames::jointNames());
define_field_array_handler(AtlasControlJointsKEffortHandler, drc::atlas_control_data_t, joints, k_effort, JointNames::jointNames());
define_field_field_handler(AtlasControlJointsDesiredControllerPeriodHandler, drc::atlas_control_data_t, joints, desired_controller_period_ms);
*/

define_array_handler(AtlasControlJointsPositionHandler, drc::atlas_command_t, position, JointNames::jointNames());
define_array_handler(AtlasControlJointsVelocityHandler, drc::atlas_command_t, velocity, JointNames::jointNames());
define_array_handler(AtlasControlJointsEffortHandler, drc::atlas_command_t, effort, JointNames::jointNames());

define_array_handler(AtlasControlJointsKQPHandler, drc::atlas_command_t, k_q_p, JointNames::jointNames());
define_array_handler(AtlasControlJointsKQIHandler, drc::atlas_command_t, k_q_i, JointNames::jointNames());
define_array_handler(AtlasControlJointsKQDPHandler, drc::atlas_command_t, k_qd_p, JointNames::jointNames());
define_array_handler(AtlasControlJointsKFPHandler, drc::atlas_command_t, k_f_p, JointNames::jointNames());
define_array_handler(AtlasControlJointsFFQDHandler, drc::atlas_command_t, ff_qd, JointNames::jointNames());
define_array_handler(AtlasControlJointsFFQDDPHandler, drc::atlas_command_t, ff_qd_d, JointNames::jointNames());
define_array_handler(AtlasControlJointsFFFDHandler, drc::atlas_command_t, ff_f_d, JointNames::jointNames());
define_array_handler(AtlasControlJointsFFConstHandler, drc::atlas_command_t, ff_const, JointNames::jointNames());
define_array_handler(AtlasControlJointsKEffortHandler, drc::atlas_command_t, k_effort, JointNames::jointNames());
define_field_handler(AtlasControlJointsDesiredControllerPeriodHandler, drc::atlas_command_t, desired_controller_period_ms);


// foot_contact_estimate_t
define_field_handler(FootContactLeft, drc::foot_contact_estimate_t, left_contact);
define_field_handler(FootContactRight, drc::foot_contact_estimate_t, right_contact);

// mav_filter_state_t
define_array_handler(MavStateHandler, mav::filter_state_t, state, createIndexList(21));

// (depricate one of these?)
// double array
define_array_handler(DoubleArrayHandler, drc::double_array_t, values, createIndexList(6));
// generic double array
define_array_handler(LargeDoubleArrayHandler, drc::double_array_t, values, createIndexList(100));

// robotiq_hand_status_t
define_field_handler(RobotiqStatusCurrentA, robotiqhand::status_t, currentA);
define_field_handler(RobotiqStatusCurrentB, robotiqhand::status_t, currentB);
define_field_handler(RobotiqStatusCurrentC, robotiqhand::status_t, currentC);

SignalHandler::SignalHandler(const SignalDescription* signalDescription, QObject* parent) : LCMSubscriber(parent)
{
  assert(signalDescription != 0);
  mDescription = *signalDescription;
  mSignalData = new SignalData();
}

SignalHandler::~SignalHandler()
{
  delete mSignalData;
}

void SignalHandler::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
{
  double timeNow;
  double signalValue;
  (void)channel;

  bool valid = this->extractSignalData(rbuf, timeNow, signalValue);
  if (valid)
  {
    mSignalData->appendSample(timeNow, signalValue);
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
  mSubscription = lcmInstance->subscribe(this->channel().toAscii().data(), &SignalHandler::handleMessage, this);
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
    factory.registerClass<RobotStateForceTorqueLHandForceHandler>();
    factory.registerClass<RobotStateForceTorqueLHandTorqueHandler>();
    factory.registerClass<RobotStateForceTorqueRHandForceHandler>();
    factory.registerClass<RobotStateForceTorqueRHandTorqueHandler>();
    factory.registerClass<AtlasStateJointPositionHandler>();
    factory.registerClass<AtlasStateJointVelocityHandler>();
    factory.registerClass<AtlasStateJointEffortHandler>();
    factory.registerClass<AtlasStateForceTorqueLFootForceZHandler>();
    factory.registerClass<AtlasStateForceTorqueLFootTorqueXHandler>();
    factory.registerClass<AtlasStateForceTorqueLFootTorqueYHandler>();
    factory.registerClass<AtlasStateForceTorqueRFootForceZHandler>();
    factory.registerClass<AtlasStateForceTorqueRFootTorqueXHandler>();
    factory.registerClass<AtlasStateForceTorqueRFootTorqueYHandler>();
    factory.registerClass<AtlasStateExtraJointPositionOutHandler>();
    factory.registerClass<AtlasStateExtraJointVelocityOutHandler>();
    factory.registerClass<AtlasStateExtraJointPressurePosHandler>();
    factory.registerClass<AtlasStateExtraJointPressureNegHandler>();
    factory.registerClass<AtlasFootPosEstLeftPositionHandler>();
    factory.registerClass<AtlasFootPosEstRightPositionHandler>();
    factory.registerClass<AtlasRawIMUBatchIMUDeltaRotation>();
    factory.registerClass<AtlasRawIMUBatchIMULinearAcceleration>();
    factory.registerClass<AtlasRawIMUPacketDeltaRotation>();
    factory.registerClass<AtlasRawIMUPacketLinearAcceleration>();
    factory.registerClass<MicrostrainINSGyro>();
    factory.registerClass<MicrostrainINSAccel>();
    factory.registerClass<PoseTypePositionHandler>();
    factory.registerClass<PoseTypeVelocityHandler>();
    factory.registerClass<PoseTypeOrientationHandler>();
    factory.registerClass<PoseTypeRotationRateHandler>();
    factory.registerClass<SixAxisForceTorqueTypeForceHandler>();
    factory.registerClass<SixAxisForceTorqueTypeMomentHandler>();
    factory.registerClass<PoseTypeAcceleration>();
    factory.registerClass<ViconBodyTransHandler>();
    factory.registerClass<ViconBodyQuatHandler>();
    factory.registerClass<AtlasStatusPumpInletPressure>();
    factory.registerClass<AtlasStatusPumpSupplyPressure>();
    factory.registerClass<AtlasStatusPumpReturnPressure>();
    factory.registerClass<AtlasStatusAirSumpPressure>();
    factory.registerClass<AtlasStatusPumpRPM>();
    factory.registerClass<AtlasStatusBehavior>();
    factory.registerClass<ControllerDebugRightFoot>();
    factory.registerClass<ControllerDebugLeftFoot>();
    factory.registerClass<ControllerDebugSolverInfo>();
    factory.registerClass<ControllerDebugQddDes>();
    factory.registerClass<ControllerDebugU>();
    factory.registerClass<ControllerDebugAlpha>();
    factory.registerClass<ControllerDebugZMPerr>();
    factory.registerClass<AtlasControlJointsPositionHandler>();
    factory.registerClass<AtlasControlJointsVelocityHandler>();
    factory.registerClass<AtlasControlJointsEffortHandler>();
    factory.registerClass<AtlasControlJointsKQPHandler>();
    factory.registerClass<AtlasControlJointsKQIHandler>();
    factory.registerClass<AtlasControlJointsKQDPHandler>();
    factory.registerClass<AtlasControlJointsKFPHandler>();
    factory.registerClass<AtlasControlJointsFFQDHandler>();
    factory.registerClass<AtlasControlJointsFFQDDPHandler>();
    factory.registerClass<AtlasControlJointsFFFDHandler>();
    factory.registerClass<AtlasControlJointsFFConstHandler>();
    factory.registerClass<AtlasControlJointsKEffortHandler>();
    factory.registerClass<AtlasControlJointsDesiredControllerPeriodHandler>();
    factory.registerClass<FootContactLeft>();
    factory.registerClass<FootContactRight>();
    factory.registerClass<MavStateHandler>();
    factory.registerClass<DoubleArrayHandler>();
    factory.registerClass<LargeDoubleArrayHandler>();
    factory.registerClass<RobotiqStatusCurrentA>();
    factory.registerClass<RobotiqStatusCurrentB>();
    factory.registerClass<RobotiqStatusCurrentC>();

  }

  return factory;
}

SignalHandler* SignalHandlerFactory::createHandler(const SignalDescription* desc) const
{
  Constructor constructor = mConstructors.value(desc->mMessageType).value(desc->mFieldName);
  if (constructor == NULL)
  {
    return NULL;
  }
  return (*constructor)(desc);
}

//-----------------------------------------------------------------------------

