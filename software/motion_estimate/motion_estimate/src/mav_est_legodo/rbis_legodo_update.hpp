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
#include <drc_utils/joint_utils.hpp>

#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/drc/atlas_state_t.hpp>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <model-client/model-client.hpp>

namespace MavStateEst {
  
// Equivalent to bot_core_pose contents
struct PoseT { 
  int64_t utime;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector4d orientation;
  Eigen::Vector3d rotation_rate;
  Eigen::Vector3d accel;
};  

class LegOdoHandler {
public:

  LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames);

  RBISUpdateInterface * processMessage(const drc::atlas_state_t *msg);

  // Calculates the Pelvis Translation between measurements:
  leg_odometry* leg_odo_;
  
  // Converts the Pelvis Translation into a RBIS measurement
  // which is then passed to the estimator
  LegOdoCommon* leg_odo_common_;
  

  void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);  
  void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);  
  
  void republishHandler (const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel);
  
  lcm::LCM* lcm_pub;
  lcm::LCM* lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_recv_boost;
  boost::shared_ptr<lcm::LCM> lcm_pub_boost;
  boost::shared_ptr<ModelClient> model_boost;
  
  BotFrames* frames;
  bot::frames* frames_cpp;
  std::vector<std::string> joint_names_;
  // To republish certain incoming messages, so as to produced output logs:
  bool republish_incoming_poses_;
  // Publish Debug Data e.g. kinematic velocities and foot contacts
  bool publish_diagnostics_;
  int verbose_; 
  
  
  // Vicon state:
  Eigen::Isometry3d prev_worldvicon_to_body_vicon_;
  int64_t prev_vicon_utime_;

  // Pose BDI:
  PoseT world_to_body_bdi_full_;  
  Eigen::Isometry3d world_to_body_bdi_;
  int64_t prev_bdi_utime_;
  int64_t body_bdi_init_;  
  
  // To locally integrate - to denoise
  Eigen::Isometry3d local_accum_;
  bool local_integration_;
  int local_max_count_;
  int local_counter_;
  int64_t local_prev_utime_;
  

};


}
#endif /* RBIS_LEGODO_LIB_UPDATE_HPP_ */

