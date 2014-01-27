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

  leg_odo_common_ = new LegOdoCommon(param);
  
  
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

  BotTrans msgT;
  memset(&msgT, 0, sizeof(msgT));
//  memcpy(msgT.trans_vec, msg->translation, 3 * sizeof(double));
//  memcpy(msgT.rot_quat,  msg->rotation   , 4 * sizeof(double));

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
  lcm_pub->publish("POSE_VICON", &pose_msg );    
}



} // end of namespace
