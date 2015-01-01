#include "voestimator.hpp"

using namespace Eigen;

VoEstimator::VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_,
  std::string channel_extension_):
  lcm_(lcm_), botframes_(botframes_), channel_extension_(channel_extension_),
  pose_initialized_(false), vo_initialized_(false){
  local_to_head_.setIdentity();

  // Assume head to camera is rigid:
  // TODO: remove bot frames dependency by providing this transform in constructor:
  botframes_cpp_->get_trans_with_utime( botframes_ ,  "head", "CAMERA_LEFT", 0, camera_to_head_);
  
  // Vis Config:
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose Body",5,1) );
}


void VoEstimator::updatePosition(int64_t utime, int64_t utime_prev, Eigen::Isometry3d delta_camera){

  // 1. Update the Position of the head frame:
  Eigen::Isometry3d delta_head = camera_to_head_.inverse()*delta_camera*camera_to_head_;
  local_to_head_ = local_to_head_*delta_head;

  // 2. Evaluate Rates:
  double delta_time =  ( (double) utime - utime_prev)/1E6;
  if(utime_prev==0){
    std::cout << "utime_prev is zero [at init]\n";
    vo_initialized_ = false; // reconfirming what is set above
  }else{
    vo_initialized_ = true;
    
    //Eigen::Isometry3d delta_head =  local_to_head_prev_.inverse() * local_to_head_;
    head_lin_rate_ = delta_head.translation()/delta_time;
    Eigen::Vector3d delta_head_rpy;
    quat_to_euler(  Eigen::Quaterniond(delta_head.rotation()) , delta_head_rpy(0), delta_head_rpy(1), delta_head_rpy(2));
    head_rot_rate_ = delta_head_rpy/delta_time; // rotation rate
  }    
  
  // 3. Maintain a smoothed version:
  double alpha = 0.8;
  head_lin_rate_alpha_ =  alpha*head_lin_rate_alpha_ + (1-alpha)*head_lin_rate_;
  head_rot_rate_alpha_ =  alpha*head_rot_rate_alpha_ + (1-alpha)*head_rot_rate_;

  // 4. Output the head position update simply by VO only
  publishUpdate(utime_prev, local_to_head_, "POSE_HEAD_FOVIS", true);
  local_to_head_prev_ = local_to_head_;
  delta_head_prev_ = delta_head;
}
  
void VoEstimator::publishUpdate(int64_t utime,
                                Eigen::Isometry3d local_to_head, std::string channel, bool output_alpha_filter){
  if ((!pose_initialized_) || (!vo_initialized_)) {
    std::cout << (int) pose_initialized_ << " pose\n";
    std::cout << (int) vo_initialized_ << " vo\n";
    std::cout << "pose, vo, zheight not initialized, refusing to publish POSE_HEAD nad POSE_BODY\n";
    return;
  }

  // Send vo pose to collections:
  Isometry3dTime local_to_headT = Isometry3dTime(utime, local_to_head);
  pc_vis_->pose_to_lcm_from_list(60000, local_to_headT);
  // std::cout << head_rot_rate_.transpose() << " head rot rate out\n";
  // std::cout << head_lin_rate_.transpose() << " head lin rate out\n";

  // publish local to head pose
  if (output_alpha_filter){
    publishPose(utime, channel + channel_extension_, local_to_head, head_lin_rate_alpha_, head_rot_rate_alpha_);
  }else{
    publishPose(utime, channel + channel_extension_, local_to_head, head_lin_rate_, head_rot_rate_);
  }
}


void VoEstimator::publishPose(int64_t utime, std::string channel, Eigen::Isometry3d pose,
  Eigen::Vector3d vel_lin, Eigen::Vector3d vel_ang){
  Eigen::Quaterniond r(pose.rotation());
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();
  pose_msg.orientation[0] =  r.w();
  pose_msg.orientation[1] =  r.x();
  pose_msg.orientation[2] =  r.y();
  pose_msg.orientation[3] =  r.z();
  pose_msg.vel[0] = vel_lin(0);
  pose_msg.vel[1] = vel_lin(1);
  pose_msg.vel[2] = vel_lin(2);
  pose_msg.rotation_rate[0] = vel_ang(0);
  pose_msg.rotation_rate[1] = vel_ang(1);
  pose_msg.rotation_rate[2] = vel_ang(2);
  pose_msg.accel[0]=0; // not estimated or filled in
  pose_msg.accel[1]=0;
  pose_msg.accel[2]=0;
  lcm_->publish( channel, &pose_msg);
}