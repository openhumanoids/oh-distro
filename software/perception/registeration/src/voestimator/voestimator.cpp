#include "voestimator.hpp"


VoEstimator::VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_):
  lcm_(lcm_), utime_(0), botframes_(botframes_)  {
  local_to_head_.setIdentity();

  // Assume head to camera is rigid:
  botframes_cpp_->get_trans_with_utime( botframes_ ,  "head", "CAMERA", utime_, camera_to_head_);
  head_to_camera_= camera_to_head_.inverse();

  if(!lcm_->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose Head",5,0) );
}

// TODO: remove fovis dependency entirely:
void VoEstimator::voUpdate(int64_t utime, Eigen::Isometry3d delta_camera){
  utime_ = utime;

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
  // publish local to head pose
  Eigen::Quaterniond l2h_rot(local_to_head_.rotation());
  Eigen::Vector3d l2h_trans = local_to_head_.translation();
  bot_core::pose_t l2h_pose_msg;
  l2h_pose_msg.utime = utime_;
  l2h_pose_msg.pos[0] = l2h_trans[0];
  l2h_pose_msg.pos[1] = l2h_trans[1];
  l2h_pose_msg.pos[2] = l2h_trans[2];
  l2h_pose_msg.orientation[0] = l2h_rot.w();
  l2h_pose_msg.orientation[1] = l2h_rot.x();
  l2h_pose_msg.orientation[2] = l2h_rot.y();
  l2h_pose_msg.orientation[3] = l2h_rot.z();
  lcm_->publish("POSE_HEAD", &l2h_pose_msg);

  // Send vo pose:
  Isometry3dTime local_to_headT = Isometry3dTime(utime_, local_to_head_);
  pc_vis_->pose_to_lcm_from_list(60000, local_to_headT);

}
