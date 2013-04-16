#include "voestimator.hpp"


VoEstimator::VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_,
  std::string pose_head_channel_):
  lcm_(lcm_), botframes_(botframes_), pose_head_channel_(pose_head_channel_),
  prev_utime_(0){
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


void VoEstimator::convertDeltaToVelocity(Eigen::Isometry3d delta_head,
                      int64_t utime, int64_t prev_utime){
  double elapsed_time =  ( (double) utime - prev_utime)/1E6;
  
  double xyz_vel[3];
  xyz_vel[0] = delta_head.translation().x()/elapsed_time;
  xyz_vel[1] = delta_head.translation().y()/elapsed_time;
  xyz_vel[2] = delta_head.translation().z()/elapsed_time;
  
  double d_ypr[3], ypr_vel[3];
  quat_to_euler(  Eigen::Quaterniond(delta_head.rotation()) , d_ypr[0], d_ypr[1], d_ypr[2]);
  ypr_vel[0] = d_ypr[0]/elapsed_time;
  ypr_vel[1] = d_ypr[1]/elapsed_time;
  ypr_vel[2] = d_ypr[2]/elapsed_time;  // ypr_vel are now the angular rates in camera frame
  
  cout << "\nElapsed Time: " << elapsed_time << "\n";
  std::stringstream ss;
  ss << "DeltaH: ";     print_Isometry3d(delta_head,ss); 
  std::cout << ss.str() << "\n";
  std::cout << "YPR: " << d_ypr[0] << ", "<<d_ypr[1] << ", "<<d_ypr[2] <<" rads [delta]\n";
  std::cout << "YPR: " << ypr_vel[0] << ", "<<ypr_vel[1] << ", "<<ypr_vel[2] <<" rad/s | ang velocity\n";
  std::cout << "YPR: " << ypr_vel[0]*180/M_PI << ", "<<ypr_vel[1]*180/M_PI << ", "<<ypr_vel[2]*180/M_PI <<" deg/s | ang velocity\n";
  std::cout << "XYZ: " << xyz_vel[0] << ", "<<xyz_vel[1] << ", "<<xyz_vel[2] <<" m/s | lin velocity\n";
  
}
  

// TODO: remove fovis dependency entirely:
void VoEstimator::voUpdate(int64_t utime, Eigen::Isometry3d delta_camera){

  Eigen::Isometry3d delta_head = head_to_camera_*delta_camera*camera_to_head_;
  local_to_head_ = local_to_head_*delta_head;

  if(prev_utime_!=0){
    convertDeltaToVelocity(delta_head, utime, prev_utime_);
  }else{
    std::cout << "prev_utime_ is zero [at init]\n"; 
  }
  
  /*
  std::stringstream ss;
  ss << "VO: ";     print_Isometry3d(delta_camera,ss); 
  ss << "\nC2H ";   print_Isometry3d(camera_to_head_,ss); 
  ss << "\nDC: ";   print_Isometry3d(delta_camera,ss); 
  ss << "\nL2H ";   print_Isometry3d(local_to_head_,ss); 
  std::cout << ss.str() << "\n";
  */
  publishUpdate(utime);
  prev_utime_ = utime;
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
