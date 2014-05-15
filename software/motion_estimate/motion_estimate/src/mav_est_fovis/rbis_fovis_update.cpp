#include "rbis_fovis_update.hpp"
#include <path_util/path_util.h>

namespace MavStateEst {

FovisHandler::FovisHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub,
                           BotParam * param): lcm_recv(lcm_recv), lcm_pub(lcm_pub){
  verbose_ = true;
  publish_diagnostics_ = true;
  
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.fovis.mode");

  if (strcmp(mode_str, "lin_rot_rate") == 0) {
    mode = FovisHandler::MODE_LIN_AND_ROT_RATE;
    std::cout << "FOVIS will provide velocity and rotation rates." << std::endl;
  }
  else if (strcmp(mode_str, "lin_rate") == 0){
    mode = FovisHandler::MODE_LIN_RATE;
    std::cout << "FOVIS will provide velocity rates." << std::endl;
  }
  else{
    // ... incomplete...
  }
  free(mode_str);
  
  Eigen::VectorXd R_fovis;
  if (mode == MODE_LIN_AND_ROT_RATE) {
    z_indices.resize(6);
    R_fovis.resize(6);
  }
  else if (mode == MODE_LIN_RATE){
    z_indices.resize(3);
    R_fovis.resize(3);
  }
  else{
    // ... incomplete...
  }

  // Initialize covariance matrix based on mode.
  if (mode == MODE_LIN_AND_ROT_RATE) {
    double R_fovis_vxyz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_vxyz");
    double R_fovis_vang = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_vang");
    R_fovis(0) = bot_sq(R_fovis_vxyz);
    R_fovis(1) = bot_sq(R_fovis_vxyz);
    R_fovis(2) = bot_sq(R_fovis_vxyz);
    R_fovis(3) = bot_sq(R_fovis_vang);
    R_fovis(4) = bot_sq(R_fovis_vang);
    R_fovis(5) = bot_sq(R_fovis_vang);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
    z_indices.tail<3>() = eigen_utils::RigidBodyState::angularVelocityInds();


  }else if (mode == MODE_LIN_RATE){
    double R_fovis_vxyz = bot_param_get_double_or_fail(param, "state_estimator.fovis.r_vxyz");
    R_fovis(0) = bot_sq(R_fovis_vxyz);
    R_fovis(1) = bot_sq(R_fovis_vxyz);
    R_fovis(2) = bot_sq(R_fovis_vxyz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }else{
    // ..incomplete
  }

  cov_fovis = R_fovis.asDiagonal();
}



// NOTE: this inserts the BotTrans trans_vec 
// as the velocity components in the pose 
// [duplicated in rbis_legodo_common.cpp]
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

RBISUpdateInterface * FovisHandler::processMessage(const fovis::update_t * msg){
  
  if (msg->estimate_status == fovis::update_t::ESTIMATE_VALID){
    std::cout << "FovisHandler: FOVIS success\n";
  }else{
    std::cout << "FovisHandler: FOVIS failure, quitting\n";
    return NULL;
  }
  
  // TODO: explore why this is allowed to be published upstream
  if ( isnan( msg->translation[0]) ){
    std::cout << "FovisHandler: FOVIS produced NaN. x="<< msg->translation[0] << ", quitting\n";
    return NULL;
  }
  
  BotTrans odo_deltaT;
  memset(&odo_deltaT, 0, sizeof(odo_deltaT));
  memcpy(odo_deltaT.trans_vec, msg->translation, 3 * sizeof(double));
  memcpy(odo_deltaT.rot_quat,  msg->rotation   , 4 * sizeof(double));

  BotTrans odo_velT = getTransAsVelocityTrans(odo_deltaT, msg->timestamp, msg->prev_timestamp);
  
  if (publish_diagnostics_){
    // Get the velocity as a pose message:
    bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(odo_velT, msg->timestamp)  ;
    lcm_pub->publish("POSE_BODY_FOVIS_VELOCITY", &vel_pose );      
  }
  
  if (mode == MODE_LIN_AND_ROT_RATE) { // Working on this:
    Eigen::VectorXd z_meas(6);
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, odo_velT.rot_quat);
    z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(odo_velT.trans_vec);
    //  eigen_utils::botDoubleToQuaternion(quat, msg->quat);
    //  z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->trans);
    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_fovis, quat, RBISUpdateInterface::fovis,
            msg->timestamp);
  }else if (mode == MODE_LIN_RATE) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>( odo_velT.trans_vec ), cov_fovis, RBISUpdateInterface::fovis,
        msg->timestamp);
    //RBISIndexedMeasurement(RBIS::velocity_inds(),
    //VO_velocity_measurement_body_coords,VO_measurement_cov_body_coords,
    //vo_sensor_id <you'll need to add this>, utime);

  }else{
    std::cout << "FovisHandler Mode not supported, exiting\n";
    return NULL;
  }
  
}


/// Everything below is duplicated in rbis_legodo_common.cpp
void printTrans(BotTrans bt, std::string message){
  std::cout << message << ": "
      << bt.trans_vec[0] << ", " << bt.trans_vec[1] << ", " << bt.trans_vec[2] << " | "
      << bt.rot_quat[0] << ", " << bt.rot_quat[1] << ", " << bt.rot_quat[2] << ", " << bt.rot_quat[3] << "\n";
}




// Difference the transform and scale by elapsed time:

BotTrans FovisHandler::getTransAsVelocityTrans(BotTrans msgT, int64_t utime, int64_t prev_utime){
  BotTrans msgT_vel;
  memset(&msgT_vel, 0, sizeof(msgT_vel));
  
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(msgT.rot_quat,rpy);
  double elapsed_time = ( (double) utime -  prev_utime)/1000000;
  double rpy_rate[3];
  rpy_rate[0] = rpy[0]/elapsed_time;
  rpy_rate[1] = rpy[1]/elapsed_time;
  rpy_rate[2] = rpy[2]/elapsed_time;
  
  if (verbose_){
    std::stringstream ss;
    ss << utime << " msgT: ";
    printTrans(msgT, ss.str() );  
    std::cout << "Elapsed Time: " << elapsed_time  << " sec\n";
    std::cout << "RPY: " << rpy[0] << ", "<<rpy[1] << ", "<<rpy[2] <<" rad\n";
    std::cout << "RPY: " << rpy[0]*180/M_PI << ", "<<rpy[1]*180/M_PI << ", "<<rpy[2]*180/M_PI <<" deg\n";
    std::cout << "RPY: " << rpy_rate[0] << ", "<<rpy_rate[1] << ", "<<rpy_rate[2] <<" rad/s | velocity scaled\n";
    std::cout << "RPY: " << rpy_rate[0]*180/M_PI << ", "<<rpy_rate[1]*180/M_PI << ", "<<rpy_rate[2]*180/M_PI <<" deg/s | velocity scaled\n";
    std::cout << "XYZ: " << msgT.trans_vec[0] << ", "  << msgT.trans_vec[1] << ", "  << msgT.trans_vec[2] << "\n";
  }
  
  msgT_vel.trans_vec[0] = msgT.trans_vec[0]/elapsed_time;
  msgT_vel.trans_vec[1] = msgT.trans_vec[1]/elapsed_time;
  msgT_vel.trans_vec[2] = msgT.trans_vec[2]/elapsed_time;
  bot_roll_pitch_yaw_to_quat (rpy_rate, msgT_vel.rot_quat);
  
  if (verbose_){
    std::stringstream ss2;
    ss2 << " msgT_vel: ";
    printTrans(msgT_vel, ss2.str() );
    std::cout << "\n\n";
  }  
  
  return msgT_vel;
}


/// Publishing Functions 
// Convert the delta position into a velocity 
// as a bot_pose message for visualization with signal scope:
// [duplicated in rbis_legodo_common.cpp]
void FovisHandler::sendTransAsVelocityPose(BotTrans msgT, int64_t utime, int64_t prev_utime, std::string channel){
  BotTrans msgT_vel = getTransAsVelocityTrans(msgT, utime, prev_utime);
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish(channel, &vel_pose );
}

} // end of namespace
