#include "voestimator.hpp"


VoEstimator::VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_,
  std::string pose_head_channel_):
  lcm_(lcm_), botframes_(botframes_), pose_head_channel_(pose_head_channel_)  {
  local_to_head_.setIdentity();

  // Assume head to camera is rigid:
  botframes_cpp_->get_trans_with_utime( botframes_ ,  "head", "CAMERA", 0, camera_to_head_);
  head_to_camera_= camera_to_head_.inverse();

  if(!lcm_->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose Head",5,1) );
}

// TODO: remove fovis dependency entirely:
void VoEstimator::voUpdate(int64_t utime, Eigen::Isometry3d delta_camera){

  Eigen::Isometry3d delta_head = head_to_camera_*delta_camera*camera_to_head_;
  local_to_head_ = local_to_head_*delta_head;

  /*
  std::stringstream ss;
  ss << "VO: ";     print_Isometry3d(delta_camera,ss); 
  ss << "\nC2H ";   print_Isometry3d(camera_to_head_,ss); 
  ss << "\nDC: ";   print_Isometry3d(delta_camera,ss); 
  ss << "\nL2H ";   print_Isometry3d(local_to_head_,ss); 
  std::cout << ss.str() << "\n";
  */
  publishUpdate(utime);
}
  
void VoEstimator::publishUpdate(int64_t utime){
  // publish local to head pose
  Eigen::Quaterniond l2h_rot(local_to_head_.rotation());
  Eigen::Vector3d l2h_trans = local_to_head_.translation();
  bot_core::pose_t l2h_pose_msg;
  l2h_pose_msg.utime = utime;
  l2h_pose_msg.pos[0] = l2h_trans[0];
  l2h_pose_msg.pos[1] = l2h_trans[1];
  l2h_pose_msg.pos[2] = l2h_trans[2];
  l2h_pose_msg.orientation[0] = l2h_rot.w();
  l2h_pose_msg.orientation[1] = l2h_rot.x();
  l2h_pose_msg.orientation[2] = l2h_rot.y();
  l2h_pose_msg.orientation[3] = l2h_rot.z();
  lcm_->publish(pose_head_channel_, &l2h_pose_msg);

  // Send vo pose:
  Isometry3dTime local_to_headT = Isometry3dTime(utime, local_to_head_);
  pc_vis_->pose_to_lcm_from_list(60000, local_to_headT);

}
