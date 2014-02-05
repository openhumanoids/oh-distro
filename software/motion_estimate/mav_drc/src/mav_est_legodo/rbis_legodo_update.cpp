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
  frames_cpp = new bot::frames(frames);

  boost::shared_ptr<lcm::LCM> lcm_recv_boost = boost::shared_ptr<lcm::LCM>(lcm_recv);
  boost::shared_ptr<lcm::LCM> lcm_pub_boost = boost::shared_ptr<lcm::LCM>(lcm_pub);
  boost::shared_ptr<ModelClient> model_boost = boost::shared_ptr<ModelClient>(model);
  leg_odo_ = new leg_odometry(  lcm_recv_boost , lcm_pub_boost ,
                               param, model_boost );

  leg_odo_common_ = new LegOdoCommon(lcm_recv, lcm_pub, param);
  
  
  // 
  // Only republish if the lcm objects are different (same logic as for main app)
  republish_incoming_ = bot_param_get_boolean_or_fail(param, "state_estimator.legodo.republish_incoming");  
  if (lcm_pub != lcm_recv && republish_incoming_) {
    republish_incoming_ = true;
  }else{
    republish_incoming_ = false;
  }
  
  // Arbitrary Subscriptions:
  if (republish_incoming_){
    std::cout << "Will republish a variety of channels\n";
    //lcm_recv->subscribe("WEBCAM",&LegOdoHandler::republishHandler,this);  
    
    lcm_recv->subscribe("VICON_BODY|VICON_FRONTPLATE",&LegOdoHandler::viconHandler,this);
  }else{
    std::cout << "Will not republish other data\n";
  }
 
  
  prev_worldvicon_to_body_vicon_.setIdentity();
  prev_vicon_utime_ = -1;
  
  
  local_integration_ = false;
  local_max_count_ = 10;
  local_counter_ = 0;
  local_prev_utime_=0;
}

RBISUpdateInterface * LegOdoHandler::processMessage(const drc::robot_state_t *msg)
{
  /// ... insert handling and special cases here.
  bool verbose = false;
  if (verbose) std::cout << "LegOdoHandler: received EST_ROBOT_STATE msg\n";

  Eigen::Isometry3d delta_odo;
  int64_t utime, prev_utime;
  
  // The changes below will eventually allow us
  // to read directly from ATLAS_STATE and to publish EST_ROBOT_STATE
  // This is unused if using the "basic mode":
  if (1==1){
    Eigen::Isometry3d world_to_body_bdi;
    world_to_body_bdi.setIdentity();
    world_to_body_bdi.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
    Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
    world_to_body_bdi.rotate(quat);   
    leg_odo_->setPoseBDI( world_to_body_bdi );
  }
  
  if (republish_incoming_){
    // Don't publish then working live:
    bot_core::pose_t bdipose = getRobotStatePoseAsBotPose(msg);
    lcm_pub->publish("POSE_BDI", &bdipose);
  }
  
  
  ///////////////////////////////////////////////
  
  
  leg_odo_->setFootForces(msg->force_torque.l_foot_force_z,msg->force_torque.r_foot_force_z);
    
  if (leg_odo_->updateOdometry(msg->joint_name, msg->joint_position,
                           msg->joint_velocity, msg->joint_effort, msg->utime)){
    leg_odo_->getDeltaLegOdometry(delta_odo, utime, prev_utime);
  } else {
    std::cout << "Leg Odometry is not valid ==============================\n";
    return NULL;
  }
  
  
  if (local_integration_){
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
      
      if (temp_prev_utime ==0){
        // skip the first iteration
        return NULL; 
      }
      
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

      return leg_odo_common_->createMeasurement(msgT, utime, temp_prev_utime);
    }else{
      std::cout << local_counter_ << " skip" << "\n";
      return NULL; 
    }
  }else{
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

    return leg_odo_common_->createMeasurement(msgT, utime, prev_utime);
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
