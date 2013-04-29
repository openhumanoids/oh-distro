#ifndef VOESTIMATOR_HPP_
#define VOESTIMATOR_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds


class VoEstimator
{
public:
  VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_, 
              std::string channel_extension_ = "");
  ~VoEstimator();

  void voUpdate(int64_t utime, Eigen::Isometry3d delta_camera);
  void publishUpdate(int64_t utime);
  void publishUpdateRobotState(const drc::robot_state_t * TRUE_state_msg);
  
  Eigen::Isometry3d getCameraPose(){ return local_to_head_*head_to_camera_; }
  Eigen::Isometry3d getHeadPose(){ return local_to_head_; }

  
  void setHeadPose(Eigen::Isometry3d local_to_head_in){
    local_to_head_ = local_to_head_in;
    pose_initialized_ = true;
  }
  
  // Clamp the Z-height to this value (which comes from gazebo - as a dev cheat)
  void setHeadPoseZ(double clamp_z_value){
    local_to_head_.translation().z() = clamp_z_value;
    zheight_initialized_ = true;
  }  
  
  // If not using setHeadPoseZ, then set this:
  void setHeadPoseZInitialized(){
    zheight_initialized_=true;
  }
  
  // Input is assumed to be YPR -  this is required because of a poor decision by me. Calulations are all done in RPY within Eigen:
  void setBodyRotRateImu(Eigen::Vector3d body_rot_rate_imu_in){
    body_rot_rate_imu_  = body_rot_rate_imu_in;
  }
  
private:
  boost::shared_ptr<lcm::LCM> lcm_;
  pointcloud_vis* pc_vis_;
  
  // have we received the first pose estimate:?
  bool pose_initialized_;
  bool vo_initialized_;
  bool zheight_initialized_; // only required in the short term (due to controller limitation).

  BotFrames* botframes_;
  bot::frames* botframes_cpp_;

  std::string channel_extension_;
  Eigen::Isometry3d camera_to_head_, head_to_camera_;
  Eigen::Isometry3d local_to_head_, local_to_body_;
  Eigen::Isometry3d local_to_head_prev_, local_to_body_prev_;
  
  
  // Cache of rates: All are stored as YPR but all conversion orders are RPY - which is a bad mistake by me. TODO!
  Eigen::Vector3d local_to_head_rot_rate_, local_to_head_lin_rate_;
  Eigen::Vector3d local_to_body_rot_rate_, local_to_body_lin_rate_;
  
  // Raw sensed rates from IMU:
  Eigen::Vector3d body_rot_rate_imu_;
  
  int64_t prev_utime_;
  
};

#endif
