#include "voestimator.hpp"

using namespace Eigen;

VoEstimator::VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_,
  std::string channel_extension_):
  lcm_(lcm_), botframes_(botframes_), channel_extension_(channel_extension_),
  utime_prev_(0), pose_initialized_(false), vo_initialized_(false), zheight_initialized_(false){
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

  /////////////// Update the body estimate ///////////
  Eigen::Isometry3d head_to_body_cur;
  int status = botframes_cpp_->get_trans_with_utime( botframes_ ,  "body", "head", utime, head_to_body_cur);
  local_to_body_ = local_to_head_ * head_to_body_cur;  
  if (head_to_body_cur.translation().z()==0 ){
    cout << "head to body is zero - this shouldnt happen=======================\n";  
  }
  
  // Evaluate Rates:
  double elapsed_time =  ( (double) utime - utime_prev_)/1E6;
  if(utime_prev_==0){
    std::cout << "utime_prev_ is zero [at init]\n"; 
    vo_initialized_ = false; // reconfirming what is set above
    
  }else{
    vo_initialized_ = true;
    
    Eigen::Vector3d delta_head_to_local = local_to_head_.translation() - local_to_head_prev_.translation();
    local_to_head_lin_rate_ = delta_head_to_local/elapsed_time; // velocity (linear rate)
    Eigen::Vector3d head_ypr,head_ypr_prev, delta_local_to_head_rot;
    quat_to_euler(  Eigen::Quaterniond(local_to_head_.rotation()) , head_ypr(0), head_ypr(1), head_ypr(2));
    quat_to_euler(  Eigen::Quaterniond(local_to_head_prev_.rotation()) , head_ypr_prev(0), head_ypr_prev(1), head_ypr_prev(2));
    delta_local_to_head_rot = head_ypr - head_ypr_prev ;
    local_to_head_rot_rate_ = delta_local_to_head_rot/elapsed_time; // rotation rate

    Eigen::Vector3d delta_body_to_local = local_to_body_.translation() - local_to_body_prev_.translation();
    local_to_body_lin_rate_ = delta_body_to_local/elapsed_time; // velocity (linear rate)
    
    bool use_vo_rot_rates=false;
    if (use_vo_rot_rates){
      Eigen::Vector3d body_ypr,body_ypr_prev, delta_local_to_body_rot;
      quat_to_euler(  Eigen::Quaterniond(local_to_body_.rotation()) , body_ypr(0), body_ypr(1), body_ypr(2));
      quat_to_euler(  Eigen::Quaterniond(local_to_body_prev_.rotation()) , body_ypr_prev(0), body_ypr_prev(1), body_ypr_prev(2));
      delta_local_to_body_rot = body_ypr - body_ypr_prev ; 
      local_to_body_rot_rate_ = delta_local_to_body_rot/elapsed_time; // rotation rate
    }else{
      // Convert from ypr to rpy:
      Eigen::Vector3d temp_body_rpy_rate(body_rot_rate_imu_(2), body_rot_rate_imu_(1), body_rot_rate_imu_(0) );
      // do calculation in RPY - this linear() was Dehann's suggestion
      Eigen::Vector3d temp_local_rpy_rate = local_to_body_.linear() * ( temp_body_rpy_rate ); 
      // Convert back to ypr holder:
      local_to_body_rot_rate_ = Eigen::Vector3d( temp_local_rpy_rate(2), temp_local_rpy_rate(1), temp_local_rpy_rate(0) );
      //std::cout << local_to_body_rot_rate_.transpose() <<" test1\n";
    }
    
    /*
    cout << delta_body_to_local << "\n";
    cout << body_ypr <<" body_ypr\n";
    cout << body_ypr_prev << " body_ypr_prev\n";
    cout << delta_local_to_body_rot << " delta_local_to_body_rot\n";
    cout << local_to_body_rot_rate_ << " local_to_body_rot_rate_\n";
    cout << local_to_body_rot_rate_*180/M_PI << " local_to_body_rot_rate_ deg\n";
  
    /*  {  
    std::stringstream ss3;
    ss3 << "head_to_body_cur : ";     print_Isometry3d(head_to_body_cur,ss3); 
    std::cout << ss3.str() << "\n";
    }  */
    
    /*
    std::stringstream ss2;
    ss2 << "local_to_body_prev: ";     print_Isometry3d(local_to_body_prev_,ss2); 
    std::cout << ss2.str() << "\n";
    std::stringstream ss3;
    ss3 << "local_to_body_    : ";     print_Isometry3d(local_to_body_,ss3); 
    std::cout << ss3.str() << "\n";
    
    {
    Eigen::Isometry3d delta_body_trans = local_to_body_prev_.inverse() * local_to_body_ ;
    std::stringstream ss3;
    ss3 << "delta_body_trans    : ";     print_Isometry3d(delta_body_trans,ss3); 
    std::cout << ss3.str() << "\n";
      
    }    
    */
    
  /*
  Stuff for converting into rates in sensor frame:
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
  */       
    
  }    
  
  
  /*
  std::stringstream ss;
  ss << "VO: ";     print_Isometry3d(delta_camera,ss); 
  ss << "\nC2H ";   print_Isometry3d(camera_to_head_,ss); 
  ss << "\nDC: ";   print_Isometry3d(delta_camera,ss); 
  ss << "\nL2H ";   print_Isometry3d(local_to_head_,ss); 
  std::cout << ss.str() << "\n";
  */
  
  local_to_head_prev_ = local_to_head_;
  local_to_body_prev_ = local_to_body_;  
  utime_prev_ = utime; 
  elapsed_time_prev_ = elapsed_time;
  delta_head_prev_ = delta_head;
  // Publish this to see vo-only estimates
  // however it always has delay. so will result in jerks to the POSES published
  publishUpdate(utime_prev_, local_to_head_prev_ , local_to_body_prev_);
}
  
void VoEstimator::publishUpdate(int64_t utime, Eigen::Isometry3d local_to_head, Eigen::Isometry3d local_to_body){
  if (((!pose_initialized_) || (!vo_initialized_))  || (!zheight_initialized_)) {
    std::cout << "pose, vo, zheight not initialized, refusing to publish POSE_HEAD nad POSE_BODY\n";
    return;
  }

  
  // publish local to head pose
  Eigen::Quaterniond l2head_rot(local_to_head.rotation());
  bot_core::pose_t l2head_msg;
  l2head_msg.utime = utime;
  l2head_msg.pos[0] = local_to_head.translation().x();
  l2head_msg.pos[1] = local_to_head.translation().y();
  l2head_msg.pos[2] = local_to_head.translation().z();
  l2head_msg.orientation[0] = l2head_rot.w();
  l2head_msg.orientation[1] = l2head_rot.x();
  l2head_msg.orientation[2] = l2head_rot.y();
  l2head_msg.orientation[3] = l2head_rot.z();
  l2head_msg.vel[0]=local_to_head_lin_rate_(0);
  l2head_msg.vel[1]=local_to_head_lin_rate_(1);
  l2head_msg.vel[2]=local_to_head_lin_rate_(2);
  l2head_msg.rotation_rate[0]=local_to_head_rot_rate_(2);
  l2head_msg.rotation_rate[1]=local_to_head_rot_rate_(1);
  l2head_msg.rotation_rate[2]=local_to_head_rot_rate_(0);
  l2head_msg.accel[0]=0; // not estimated
  l2head_msg.accel[1]=0;
  l2head_msg.accel[2]=0;  
  lcm_->publish("POSE_HEAD" + channel_extension_, &l2head_msg);  

  // Send vo pose to collections:
  Isometry3dTime local_to_headT = Isometry3dTime(utime, local_to_head);
  pc_vis_->pose_to_lcm_from_list(60000, local_to_headT);

  // publish local to body pose
  Eigen::Quaterniond l2body_rot(local_to_body.rotation());
  bot_core::pose_t l2body_msg;
  l2body_msg.utime = utime;
  l2body_msg.pos[0] = local_to_body.translation().x();
  l2body_msg.pos[1] = local_to_body.translation().y();
  l2body_msg.pos[2] = local_to_body.translation().z();
  l2body_msg.orientation[0] = l2body_rot.w();
  l2body_msg.orientation[1] = l2body_rot.x();
  l2body_msg.orientation[2] = l2body_rot.y();
  l2body_msg.orientation[3] = l2body_rot.z();
  l2body_msg.vel[0]=local_to_body_lin_rate_(0);
  l2body_msg.vel[1]=local_to_body_lin_rate_(1);
  l2body_msg.vel[2]=local_to_body_lin_rate_(2);
  l2body_msg.rotation_rate[0]=local_to_body_rot_rate_(2);
  l2body_msg.rotation_rate[1]=local_to_body_rot_rate_(1);
  l2body_msg.rotation_rate[2]=local_to_body_rot_rate_(0);
  l2body_msg.accel[0]=0; // not estimated
  l2body_msg.accel[1]=0;
  l2body_msg.accel[2]=0;  
  lcm_->publish("POSE_BODY" + channel_extension_, &l2body_msg);  
}



Eigen::Isometry3d VoEstimator::extrapolateHeadRates(float d_time){
  //std::cout << "age of pose: " << (TRUE_state_msg->utime - utime_prev_) << "\n";

  if ((d_time > 0.1 )){
    // This would only happen if the frequency is less than 10Hz
    std::cout << "[ERROR] extrapolation time is " << d_time << "sec. capping at 0.1\n";
    d_time =0.1;
  }else if(d_time < 0){
    // This would only happen if we received out of order messages    
    std::cout << "[ERROR] extrapolation time is " << d_time << "sec. capping at 0sec\n";
    d_time =0.0;
  }
  
  Eigen::Vector3d d_linear = local_to_head_lin_rate_*d_time ;
  Eigen::Vector3d d_rotation = local_to_head_rot_rate_*d_time ;
  
  /*
  std::cout << "age of pose: " << d_time  << " sec\n";
  std::cout << "linear  rate: " << local_to_head_lin_rate_.transpose()  << " \n";
  std::cout << "rotationrate: " << 180*(1/M_PI)*local_to_head_rot_rate_.transpose()  << " \n";
  std::cout << "linear delta: " << d_linear.transpose()  << " \n";
  std::cout << "rotationdelt: " << 180*(1/M_PI)*d_rotation.transpose()  << " \n";
  */
  Eigen::Isometry3d extrapol =  Eigen::Isometry3d::Identity();
  Matrix3d m;
  m = AngleAxisd ( d_rotation(0) , Vector3d::UnitZ ())
                  * AngleAxisd (d_rotation(1) , Vector3d::UnitY ())
                  * AngleAxisd ( d_rotation(2) , Vector3d::UnitX ());  
  extrapol *= m;  
  extrapol.translation()  << d_linear;    
  return extrapol;
}

void VoEstimator::publishUpdateRobotState(const drc::robot_state_t * TRUE_state_msg){
  if (((!pose_initialized_) || (!vo_initialized_))  || (!zheight_initialized_)) {
    std::cout << "pose or vo or zheight not initialized, refusing to publish EST_ROBOT_STATE\n";
    return;
  }
  // Infer the Robot's head position from the ground truth root world pose
  // TODO: do this somewhere else
  bot_core::pose_t pose_msg;
  pose_msg.utime = TRUE_state_msg->utime;
  pose_msg.pos[0] = TRUE_state_msg->origin_position.translation.x;
  pose_msg.pos[1] = TRUE_state_msg->origin_position.translation.y;
  pose_msg.pos[2] = TRUE_state_msg->origin_position.translation.z;
  pose_msg.orientation[0] = TRUE_state_msg->origin_position.rotation.w;
  pose_msg.orientation[1] = TRUE_state_msg->origin_position.rotation.x;
  pose_msg.orientation[2] = TRUE_state_msg->origin_position.rotation.y;
  pose_msg.orientation[3] = TRUE_state_msg->origin_position.rotation.z;
  lcm_->publish("POSE_BODY_TRUE", &pose_msg);    
  
  // 1. Extrapolate:
  // EXTRAPOLATION IS VERY WEAK BASED OFF OF THE VO - LOTS OF JUMPING AROUND 
  // AS ITS BASED ON IMAGES UP TO 2 FRAMES OLD
  //float d_time = ((float) (TRUE_state_msg->utime - utime_prev_)*1E-6); // diff in time since last VO estimate
  //Eigen::Isometry3d extrapol = extrapolateHeadRates(d_time);
  //Eigen::Isometry3d current_local_to_head = local_to_head_ * extrapol;
  Eigen::Isometry3d current_local_to_head = local_to_head_; // NO EXTRAPOLATION
  
  // 2. Update the body-to-head estimate
  Eigen::Isometry3d head_to_body_cur;
  int status = botframes_cpp_->get_trans_with_utime( botframes_ ,  "body", "head", TRUE_state_msg->utime , head_to_body_cur);
  Eigen::Isometry3d current_local_to_body = current_local_to_head* head_to_body_cur;  
  if (head_to_body_cur.translation().z()==0 ){
    cout << "head to body is zero - this shouldnt happen=======================\n";  
  }  
  
  // Publish to Bot Frames:
  //publishUpdate(utime_prev_, current_local_to_head , current_local_to_body);

  // Publish EST_ROBOT_STATE
  Eigen::Quaterniond l2body_rot(current_local_to_body.rotation());
  drc::position_3d_t origin;
  origin.translation.x = current_local_to_body.translation().x();
  origin.translation.y = current_local_to_body.translation().y();
  origin.translation.z = current_local_to_body.translation().z();
  origin.rotation.w = l2body_rot.w();
  origin.rotation.x = l2body_rot.x();
  origin.rotation.y = l2body_rot.y();
  origin.rotation.z = l2body_rot.z();  
  
  drc::twist_t twist;
  twist.linear_velocity.x = local_to_body_lin_rate_(0);
  twist.linear_velocity.y = local_to_body_lin_rate_(1);
  twist.linear_velocity.z = local_to_body_lin_rate_(2);
  twist.angular_velocity.x = local_to_body_rot_rate_(2);
  twist.angular_velocity.y = local_to_body_rot_rate_(1);
  twist.angular_velocity.z = local_to_body_rot_rate_(0);
  
  // EST is TRUE with sensor estimated position
  drc::robot_state_t msgout;
  msgout = *TRUE_state_msg;
  msgout.origin_position = origin;
  msgout.origin_twist = twist;
  lcm_->publish("EST_ROBOT_STATE" + channel_extension_, &msgout); 
}