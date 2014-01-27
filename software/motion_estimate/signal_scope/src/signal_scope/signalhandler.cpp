#include "signalhandler.h"

#include "signaldata.h"
#include "signaldescription.h"
#include "jointnames.h"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/microstrain_comm.hpp>
#include <lcmtypes/vicon.hpp>
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "lcmtypes/drc/atlas_foot_pos_est_t.hpp"
#include "lcmtypes/drc/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_state_extra_t.hpp"
#include "lcmtypes/drc/atlas_status_t.hpp"
#include "lcmtypes/drc/drill_control_t.hpp"
#include "lcmtypes/drc/foot_contact_estimate_t.hpp"
#include "lcmtypes/drc/ins_update_request_t.hpp"
#include "lcmtypes/drc/ins_update_packet_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"

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
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
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
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
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
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
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
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
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
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
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
bool className::extractSignalData(const lcm::ReceiveBuffer* rbuf, float& timeNow, float& signalValue) \
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


// atlas_foot_pos_est_t
define_array_handler(AtlasFootPosEstLeftPositionHandler, drc::atlas_foot_pos_est_t, left_position, createIndexList(3));
define_array_handler(AtlasFootPosEstRightPositionHandler, drc::atlas_foot_pos_est_t, right_position, createIndexList(3));


// vicon body_t
define_array_handler(ViconBodyTransHandler, vicon::body_t, trans, createIndexList(3));
define_array_handler(ViconBodyQuatHandler, vicon::body_t, quat, createIndexList(4));

// atlas_state_t

define_field_handler(AtlasStatusPumpInletPressure, drc::atlas_status_t, pump_inlet_pressure);
define_field_handler(AtlasStatusPumpSupplyPressure, drc::atlas_status_t, pump_supply_pressure);
define_field_handler(AtlasStatusPumpReturnPressure, drc::atlas_status_t, pump_return_pressure);
define_field_handler(AtlasStatusAirSumpPressure, drc::atlas_status_t, air_sump_pressure);
define_field_handler(AtlasStatusPumpRPM, drc::atlas_status_t, current_pump_rpm);
define_field_handler(AtlasStatusBehavior, drc::atlas_status_t, behavior);

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

// ins_update_request_t

define_field_field_field_handler(INSUpdateRequestPoseTranslationXHandler, drc::ins_update_request_t, pose, translation, x);
define_field_field_field_handler(INSUpdateRequestPoseTranslationYHandler, drc::ins_update_request_t, pose, translation, y);
define_field_field_field_handler(INSUpdateRequestPoseTranslationZHandler, drc::ins_update_request_t, pose, translation, z);

define_field_field_field_handler(INSUpdateRequestTwistLinearVelocityXHandler, drc::ins_update_request_t, twist, linear_velocity, x);
define_field_field_field_handler(INSUpdateRequestTwistLinearVelocityYHandler, drc::ins_update_request_t, twist, linear_velocity, y);
define_field_field_field_handler(INSUpdateRequestTwistLinearVelocityZHandler, drc::ins_update_request_t, twist, linear_velocity, z);

define_field_field_field_handler(INSUpdateRequestPoseRotationWHandler, drc::ins_update_request_t, pose, rotation, w);
define_field_field_field_handler(INSUpdateRequestPoseRotationXHandler, drc::ins_update_request_t, pose, rotation, x);
define_field_field_field_handler(INSUpdateRequestPoseRotationYHandler, drc::ins_update_request_t, pose, rotation, y);
define_field_field_field_handler(INSUpdateRequestPoseRotationZHandler, drc::ins_update_request_t, pose, rotation, z);

define_field_field_handler(INSUpdateRequestGyroBiasEstX, drc::ins_update_request_t, gyroBiasEst, x);
define_field_field_handler(INSUpdateRequestGyroBiasEstY, drc::ins_update_request_t, gyroBiasEst, y);
define_field_field_handler(INSUpdateRequestGyroBiasEstZ, drc::ins_update_request_t, gyroBiasEst, z);

define_field_field_handler(INSUpdateRequestAccBiasEstX, drc::ins_update_request_t, accBiasEst, x);
define_field_field_handler(INSUpdateRequestAccBiasEstY, drc::ins_update_request_t, accBiasEst, y);
define_field_field_handler(INSUpdateRequestAccBiasEstZ, drc::ins_update_request_t, accBiasEst, z);

define_field_field_handler(INSUpdateRequestLocalLinearAccelerationX, drc::ins_update_request_t, local_linear_acceleration, x);
define_field_field_handler(INSUpdateRequestLocalLinearAccelerationY, drc::ins_update_request_t, local_linear_acceleration, y);
define_field_field_handler(INSUpdateRequestLocalLinearAccelerationZ, drc::ins_update_request_t, local_linear_acceleration, z);

define_field_field_handler(INSUpdateRequestLocalLinearForceX, drc::ins_update_request_t, local_linear_force, x);
define_field_field_handler(INSUpdateRequestLocalLinearForceY, drc::ins_update_request_t, local_linear_force, y);
define_field_field_handler(INSUpdateRequestLocalLinearForceZ, drc::ins_update_request_t, local_linear_force, z);

define_field_field_handler(INSUpdateRequestReferencePosLocalX, drc::ins_update_request_t, referencePos_local, x);
define_field_field_handler(INSUpdateRequestReferencePosLocalY, drc::ins_update_request_t, referencePos_local, y);
define_field_field_handler(INSUpdateRequestReferencePosLocalZ, drc::ins_update_request_t, referencePos_local, z);

define_field_field_handler(INSUpdateRequestReferenceVelLocalX, drc::ins_update_request_t, referenceVel_local, x);
define_field_field_handler(INSUpdateRequestReferenceVelLocalY, drc::ins_update_request_t, referenceVel_local, y);
define_field_field_handler(INSUpdateRequestReferenceVelLocalZ, drc::ins_update_request_t, referenceVel_local, z);

define_field_field_handler(INSUpdateRequestReferenceVelBodyX, drc::ins_update_request_t, referenceVel_body, x);
define_field_field_handler(INSUpdateRequestReferenceVelBodyY, drc::ins_update_request_t, referenceVel_body, y);
define_field_field_handler(INSUpdateRequestReferenceVelBodyZ, drc::ins_update_request_t, referenceVel_body, z);


// ins_update_packet_t
define_field_field_handler(INSUpdatePacketDBiasGyroBX, drc::ins_update_packet_t, dbiasGyro_b, x);
define_field_field_handler(INSUpdatePacketDBiasGyroBY, drc::ins_update_packet_t, dbiasGyro_b, y);
define_field_field_handler(INSUpdatePacketDBiasGyroBZ, drc::ins_update_packet_t, dbiasGyro_b, z);

define_field_field_handler(INSUpdatePacketDBiasAccBX, drc::ins_update_packet_t, dbiasAcc_b, x);
define_field_field_handler(INSUpdatePacketDBiasAccBY, drc::ins_update_packet_t, dbiasAcc_b, y);
define_field_field_handler(INSUpdatePacketDBiasAccBZ, drc::ins_update_packet_t, dbiasAcc_b, z);

define_field_field_handler(INSUpdatePacketDEX, drc::ins_update_packet_t, dE_l, x);
define_field_field_handler(INSUpdatePacketDEY, drc::ins_update_packet_t, dE_l, y);
define_field_field_handler(INSUpdatePacketDEZ, drc::ins_update_packet_t, dE_l, z);

define_field_field_handler(INSUpdatePacketDVelX, drc::ins_update_packet_t, dVel_l, x);
define_field_field_handler(INSUpdatePacketDVelY, drc::ins_update_packet_t, dVel_l, y);
define_field_field_handler(INSUpdatePacketDVelZ, drc::ins_update_packet_t, dVel_l, z);

define_field_field_handler(INSUpdatePacketDPosX, drc::ins_update_packet_t, dPos_l, x);
define_field_field_handler(INSUpdatePacketDPosY, drc::ins_update_packet_t, dPos_l, y);
define_field_field_handler(INSUpdatePacketDPosZ, drc::ins_update_packet_t, dPos_l, z);

// drill_control_t
define_array_handler(DrillControlData, drc::drill_control_t, data, createIndexList(100));

// foot_contact_estimate_t
define_field_handler(FootContactLeft, drc::foot_contact_estimate_t, left_contact);
define_field_handler(FootContactRight, drc::foot_contact_estimate_t, right_contact);

SignalHandler::SignalHandler(const SignalDescription* signalDescription)
{
  assert(signalDescription != 0);
  mDescription = *signalDescription;
  mSignalData = new SignalData();
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
    factory.registerClass<PoseTypeVelocityHandler>();
    factory.registerClass<PoseTypeOrientationHandler>();
    factory.registerClass<PoseTypeRotationRateHandler>();
    factory.registerClass<PoseTypeAcceleration>();
    factory.registerClass<ViconBodyTransHandler>();
    factory.registerClass<ViconBodyQuatHandler>();
    factory.registerClass<AtlasStatusPumpInletPressure>();
    factory.registerClass<AtlasStatusPumpSupplyPressure>();
    factory.registerClass<AtlasStatusPumpReturnPressure>();
    factory.registerClass<AtlasStatusAirSumpPressure>();
    factory.registerClass<AtlasStatusPumpRPM>();
    factory.registerClass<AtlasStatusBehavior>();
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
    factory.registerClass<INSUpdateRequestPoseTranslationXHandler>();
    factory.registerClass<INSUpdateRequestPoseTranslationYHandler>();
    factory.registerClass<INSUpdateRequestPoseTranslationZHandler>();
    factory.registerClass<INSUpdateRequestTwistLinearVelocityXHandler>();
    factory.registerClass<INSUpdateRequestTwistLinearVelocityYHandler>();
    factory.registerClass<INSUpdateRequestTwistLinearVelocityZHandler>();
    factory.registerClass<INSUpdateRequestPoseRotationWHandler>();
    factory.registerClass<INSUpdateRequestPoseRotationXHandler>();
    factory.registerClass<INSUpdateRequestPoseRotationYHandler>();
    factory.registerClass<INSUpdateRequestPoseRotationZHandler>();
    factory.registerClass<INSUpdateRequestGyroBiasEstX>();
    factory.registerClass<INSUpdateRequestGyroBiasEstY>();
    factory.registerClass<INSUpdateRequestGyroBiasEstZ>();
    factory.registerClass<INSUpdateRequestAccBiasEstX>();
    factory.registerClass<INSUpdateRequestAccBiasEstY>();
    factory.registerClass<INSUpdateRequestAccBiasEstZ>();
    factory.registerClass<INSUpdateRequestLocalLinearAccelerationX>();
    factory.registerClass<INSUpdateRequestLocalLinearAccelerationY>();
    factory.registerClass<INSUpdateRequestLocalLinearAccelerationZ>();
    factory.registerClass<INSUpdateRequestLocalLinearForceX>();
    factory.registerClass<INSUpdateRequestLocalLinearForceY>();
    factory.registerClass<INSUpdateRequestLocalLinearForceZ>();
    factory.registerClass<INSUpdateRequestReferencePosLocalX>();
    factory.registerClass<INSUpdateRequestReferencePosLocalY>();
    factory.registerClass<INSUpdateRequestReferencePosLocalZ>();
    factory.registerClass<INSUpdateRequestReferenceVelLocalZ>();
    factory.registerClass<INSUpdateRequestReferenceVelLocalY>();
    factory.registerClass<INSUpdateRequestReferenceVelLocalZ>();
    factory.registerClass<INSUpdateRequestReferenceVelBodyZ>();
    factory.registerClass<INSUpdateRequestReferenceVelBodyY>();
    factory.registerClass<INSUpdateRequestReferenceVelBodyZ>();
    factory.registerClass<INSUpdatePacketDBiasGyroBX>();
    factory.registerClass<INSUpdatePacketDBiasGyroBY>();
    factory.registerClass<INSUpdatePacketDBiasGyroBZ>();
    factory.registerClass<INSUpdatePacketDBiasAccBX>();
    factory.registerClass<INSUpdatePacketDBiasAccBY>();
    factory.registerClass<INSUpdatePacketDBiasAccBZ>();
    factory.registerClass<INSUpdatePacketDEX>();
    factory.registerClass<INSUpdatePacketDEY>();
    factory.registerClass<INSUpdatePacketDEZ>();
    factory.registerClass<INSUpdatePacketDVelX>();
    factory.registerClass<INSUpdatePacketDVelY>();
    factory.registerClass<INSUpdatePacketDVelZ>();
    factory.registerClass<INSUpdatePacketDPosX>();
    factory.registerClass<INSUpdatePacketDPosY>();
    factory.registerClass<INSUpdatePacketDPosZ>();
    factory.registerClass<DrillControlData>();
    factory.registerClass<FootContactLeft>();
    factory.registerClass<FootContactRight>();

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

