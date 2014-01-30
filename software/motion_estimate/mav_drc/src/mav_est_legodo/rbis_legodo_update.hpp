#ifndef RBIS_LEGODO_LIB_UPDATE_HPP_
#define RBIS_LEGODO_LIB_UPDATE_HPP_

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>


#include <string>

#include <mav_state_est/mav-est-legodo/rbis_legodo_common.hpp>

#include <estimate/leg_odometry.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <lcmtypes/drc/pose_transform_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

namespace MavStateEst {

class LegOdoHandler {
public:

  LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames);

  RBISUpdateInterface * processMessage(const drc::robot_state_t *msg);

  // Calculates the Pelvis Translation between measurements:
  leg_odometry* leg_odo_;
  
  // Converts the Pelvis Translation into a RBIS measurement
  // which is then passed to the estimator
  LegOdoCommon* leg_odo_common_;
  

  void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);  
  
  void republishHandler (const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  BotFrames* frames;
  bot::frames* frames_cpp;
  // To republish certain incoming messages, so as to produced output logs:
  bool republish_incoming_;
  
  
  // Vicon state:
  Eigen::Isometry3d prev_worldvicon_to_body_vicon_;
  int64_t prev_vicon_utime_;
  
  
  // To locally integrate - to denoise
  Eigen::Isometry3d local_accum_;
  bool local_integration_;
  int local_max_count_;
  int local_counter_;
  int64_t local_prev_utime_;
  
};


}
#endif /* RBIS_LEGODO_LIB_UPDATE_HPP_ */

