#ifndef STATEESTIMATORUTILITIES_H_
#define STATEESTIMATORUTILITIES_H_

#include <iostream>

#include <Eigen/Dense>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"

#include <inertial-odometry/InertialOdometry_Types.hpp>
#include <inertial-odometry/Odometry.hpp>

namespace StateEstimate {

struct Joints { 
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> effort;
  std::vector<std::string> name;
};

// Equivalent to bot_core_pose contents
struct PoseT { 
  int64_t utime;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector4d orientation;
  Eigen::Vector3d rotation_rate;
  Eigen::Vector3d accel;
};

// BDI POSE============================================================================
// Returns false if Pose BDI is old or hasn't appeared yet
bool convertBDIPose_ERS(const bot_core::pose_t* msg, drc::robot_state_t& ERS_msg);


void extractBDIPose(const bot_core::pose_t* msg, PoseT &pose_BDI_);
bool insertPoseBDI(const PoseT &pose_BDI_, drc::robot_state_t& msg);


// ATLAS STATE=========================================================================
bool insertAtlasState_ERS(const drc::atlas_state_t &atlasState, drc::robot_state_t &mERSMsg);


void appendJoints(drc::robot_state_t& msg_out, const Joints &joints);
void insertAtlasJoints(const drc::atlas_state_t* msg, Joints &jointContainer);


// IMU DATA============================================================================
void handle_inertial_data_temp_name(const double dt, const drc::atlas_raw_imu_t &imu, const bot_core::pose_t &bdiPose, InertialOdometry::Odometry &inert_odo, drc::robot_state_t& _ERSmsg);

//void handle_inertial_data_temp_name(const drc::atlas_raw_imu_t &imu, const bot_core::pose_t &bdiPose, InertialOdometry::Odometry &inert_odo, drc::robot_state_t& _ERSmsg) {




} // namespace StateEstimate

#endif /*STATEESTIMATORUTILITIES_H_*/
