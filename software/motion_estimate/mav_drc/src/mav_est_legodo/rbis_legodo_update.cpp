#include "rbis_legodo_update.hpp"
#include <path_util/path_util.h>
#include <string>


using namespace std;

namespace MavStateEst {


LegOdoHandler::LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model, BotFrames * frames): 
      lcm_recv(lcm_recv), lcm_pub(lcm_pub), frames(frames)
{
  std::cout << "LegOdo will compute directly, in thread\n";
  verbose_ =2; // 3 lots, 2 some, 1 v.important
  
  frames_cpp = new bot::frames(frames);
  lcm_recv_boost = boost::shared_ptr<lcm::LCM>(lcm_recv);
  lcm_pub_boost = boost::shared_ptr<lcm::LCM>(lcm_pub);
  model_boost = boost::shared_ptr<ModelClient>(model);
  
  leg_odo_ = new leg_odometry(lcm_pub_boost, param, model_boost );
  leg_odo_common_ = new LegOdoCommon(lcm_recv, lcm_pub, param);

  lcm_recv->subscribe("POSE_BDI",&LegOdoHandler::poseBDIHandler,this);
  
  publish_diagnostics_ = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.publish_diagnostics");  
  
  republish_incoming_poses_ = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.republish_incoming_poses");  
  if (lcm_pub != lcm_recv && republish_incoming_poses_) { 
    republish_incoming_poses_ = true; // Only republish if the lcm objects are different (same logic as for main app)
  }else{
    republish_incoming_poses_ = false;
  }
  if (republish_incoming_poses_){
    std::cout << "Will republish a variety of channels\n";
    lcm_recv->subscribe("VICON_BODY|VICON_FRONTPLATE",&LegOdoHandler::viconHandler,this);
  }else{
    std::cout << "Will not republish other data\n";
  }
  
  // Arbitrary Subscriptions:
  bool republish_cameras = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.republish_cameras");    
  if (lcm_pub != lcm_recv && republish_cameras) {
    std::cout << "Will republish camera data\n";
    lcm_recv->subscribe("WEBCAM",&LegOdoHandler::republishHandler,this);  
  }
 
  
  prev_worldvicon_to_body_vicon_.setIdentity();
  prev_vicon_utime_ = -1;
  
  
  local_integration_ = false;
  local_max_count_ = 10;
  local_counter_ = 0;
  local_prev_utime_=0;
  
  // The most recent estimate of pose-bdi. Assumed to be always current. only used for gravity slaved leg odom:
  // TODO: query our own leg odom orientation for this:
  world_to_body_bdi_.setIdentity();
  prev_bdi_utime_ = -1;
  body_bdi_init_ = false;
  
  JointUtils* joint_utils = new JointUtils();
  joint_names_ = joint_utils->atlas_joint_names;
  //std::cout << joint_names_.size() << " joint angles\n";
}

void LegOdoHandler::poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  world_to_body_bdi_full_.utime = msg->utime;
  world_to_body_bdi_full_.pos = Eigen::Vector3d( msg->pos[0],  msg->pos[1],  msg->pos[2] );
  world_to_body_bdi_full_.vel = Eigen::Vector3d( msg->vel[0],  msg->vel[1],  msg->vel[2] );
  world_to_body_bdi_full_.orientation = Eigen::Vector4d( msg->orientation[0],  msg->orientation[1],  msg->orientation[2],  msg->orientation[3] );
  world_to_body_bdi_full_.rotation_rate = Eigen::Vector3d( msg->rotation_rate[0],  msg->rotation_rate[1],  msg->rotation_rate[2] );
  world_to_body_bdi_full_.accel = Eigen::Vector3d( msg->accel[0],  msg->accel[1],  msg->accel[2] );    
  
  
  world_to_body_bdi_.setIdentity();
  world_to_body_bdi_.translation()  << msg->pos[0], msg->pos[1] , msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], 
                                               msg->orientation[2], msg->orientation[3]);
  world_to_body_bdi_.rotate(quat);
  
  prev_bdi_utime_ = msg->utime;
  body_bdi_init_ = true;
}


BotTrans getPoseAsBotTrans(Eigen::Isometry3d delta_odo){
  BotTrans msgT;
  memset(&msgT, 0, sizeof(msgT));
  Eigen::Vector3d motion_T = delta_odo.translation();
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(delta_odo.rotation());
  msgT.trans_vec[0] = motion_T(0);
  msgT.trans_vec[1] = motion_T(1);
  msgT.trans_vec[2] = motion_T(2);
  msgT.rot_quat[0] = motion_R.w();
  msgT.rot_quat[1] = motion_R.x();
  msgT.rot_quat[2] = motion_R.y();
  msgT.rot_quat[3] = motion_R.z();
  
  return msgT;
}


// NOTE: this inserts the BotTrans trans_vec 
// as the velocity components in the pose (for visualization in signal scope)
bot_core::pose_t getBotTransAsBotPoseVelocity(BotTrans bt, int64_t utime ){
  bot_core::pose_t pose;
  pose.utime = utime;
  pose.pos[0] = 0;
  pose.pos[1] = 0;
  pose.pos[2] = 0;
  pose.orientation[0] = 0;
  pose.orientation[1] = 0;
  pose.orientation[2] = 0;
  pose.orientation[3] = 0;
  pose.vel[0] = bt.trans_vec[0];
  pose.vel[1] = bt.trans_vec[1];
  pose.vel[2] = bt.trans_vec[2];
  pose.rotation_rate[0] = 0;//bt.rot_quat[0];
  pose.rotation_rate[1] = 0;//bt.rot_quat[1];
  pose.rotation_rate[2] = 0;//bt.rot_quat[2];
  return pose;
}


// Convert the delta position into a velocity 
// as a bot_pose message for visualization with signal scope:
void LegOdoHandler::sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel){
  BotTrans msgT_vel = leg_odo_common_->getTransAsVelocityTrans(msgT, utime, prev_utime);
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish(channel, &vel_pose );
}


RBISUpdateInterface * LegOdoHandler::processMessage(const drc::atlas_state_t *msg)
{
  
  if (!body_bdi_init_){
    std::cout << "POSE_BDI not received yet, not integrating leg odometry =========================\n";
    return NULL;    
  }
  
  // Disabling below will turn off all input of POSE_BDI to leg odom:
  // which is unused if using the "basic_mode"
  if (1==1){
    leg_odo_->setPoseBDI( world_to_body_bdi_ );
  }
  if (republish_incoming_poses_){ // Don't publish when working live:
    bot_core::pose_t bdipose = getPoseAsBotPose(world_to_body_bdi_, prev_bdi_utime_);
    lcm_pub->publish("POSE_BDI", &bdipose);
  }
  
  // 1. Do the Leg Odometry Integration
  leg_odo_->setFootForces(msg->force_torque.l_foot_force_z,msg->force_torque.r_foot_force_z);
  float odometry_status = leg_odo_->updateOdometry(joint_names_, msg->joint_position, msg->utime);
  if (odometry_status<0){
    if (verbose_ >= 3) std::cout << "Leg Odometry is not valid not integrating =========================\n";
    
    if(publish_diagnostics_){
      Eigen::Isometry3d delta_odo;
      int64_t utime, prev_utime;
      leg_odo_->getDeltaLegOdometry(delta_odo, utime, prev_utime);
      BotTrans msgT = getPoseAsBotTrans(delta_odo);
      sendTransAsVelocityPose( msgT, utime, prev_utime, "POSE_BODY_LEGODO_VELOCITY_FAIL");
    }
    
    return NULL;
  }
  
  // 2. If successful make a RBIS Measurement
  Eigen::Isometry3d delta_odo;
  int64_t utime, prev_utime;
  leg_odo_->getDeltaLegOdometry(delta_odo, utime, prev_utime);
  
  if (!local_integration_){ // typical case...
    BotTrans msgT = getPoseAsBotTrans(delta_odo);
    if (publish_diagnostics_) sendTransAsVelocityPose(msgT, utime, prev_utime, "POSE_BODY_LEGODO_VELOCITY");    
    return leg_odo_common_->createMeasurement(msgT, utime, prev_utime, odometry_status); // 
    
  }else{    

    local_accum_ =  local_accum_*delta_odo;
    local_counter_++;
    
    std::cout << local_counter_ << " counter" << "\n";
    // If the counter is max, then integrate it, else return null pointer
    if (local_counter_ > local_max_count_){
      std::cout << local_counter_ << " integrate" << "\n";
      delta_odo = local_accum_;
      local_accum_.setIdentity();

      int64_t temp_prev_utime= local_prev_utime_; // local copy to pass
      local_counter_=0;
      local_prev_utime_ = utime;
      
      if (temp_prev_utime ==0){ // skip the first iteration
        return NULL; 
      }
      
      BotTrans msgT = getPoseAsBotTrans(delta_odo);
      if (publish_diagnostics_) sendTransAsVelocityPose(msgT, utime, temp_prev_utime, "POSE_BODY_LEGODO_VELOCITY");
      // TODO: currently this only incorporates only ***most recent*** odometry_status, should support an average:
      return leg_odo_common_->createMeasurement(msgT, utime, temp_prev_utime, odometry_status); 
    }else{
      std::cout << local_counter_ << " skip" << "\n";
      return NULL; 
    }
  }  

}


void LegOdoHandler::republishHandler (const lcm::ReceiveBuffer* rbuf, const std::string& channel){
  lcm_pub->publish(channel, rbuf->data, rbuf->data_size);
}



void LegOdoHandler::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){

  Eigen::Isometry3d worldvicon_to_frontplate_vicon;
  worldvicon_to_frontplate_vicon.setIdentity();
  worldvicon_to_frontplate_vicon.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                               msg->quat[2], msg->quat[3]);
  worldvicon_to_frontplate_vicon.rotate(quat); 
  
  // Apply the body to frontplate transform
  Eigen::Isometry3d frontplate_vicon_to_body_vicon;
  frames_cpp->get_trans_with_utime( "body_vicon" , "frontplate_vicon", msg->utime, frontplate_vicon_to_body_vicon);    
  Eigen::Isometry3d worldvicon_to_body_vicon = worldvicon_to_frontplate_vicon* frontplate_vicon_to_body_vicon;
  bot_core::pose_t pose_msg = getPoseAsBotPose(worldvicon_to_body_vicon, msg->utime);
  
  // determine the vicon body frame velocity:
  if (prev_vicon_utime_ > 0){
    // the delta transform between the previous and current 
    Eigen::Isometry3d delta_vicon = prev_worldvicon_to_body_vicon_.inverse() * worldvicon_to_body_vicon;
    double elapsed_time = ( (double) msg->utime -  prev_vicon_utime_)/1000000;
    pose_msg.vel[0] = delta_vicon.translation().x() / elapsed_time;
    pose_msg.vel[1] = delta_vicon.translation().y() / elapsed_time;
    pose_msg.vel[2] = delta_vicon.translation().z() / elapsed_time;
  }
  
  lcm_pub->publish("POSE_VICON", &pose_msg );    
  
  prev_worldvicon_to_body_vicon_ = worldvicon_to_body_vicon;
  prev_vicon_utime_ = msg->utime;
}



} // end of namespace
