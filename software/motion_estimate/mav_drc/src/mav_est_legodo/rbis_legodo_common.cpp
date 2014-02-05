#include "rbis_legodo_common.hpp"

namespace MavStateEst {

LegOdoCommon::LegOdoCommon(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, BotParam * param):
  lcm_recv(lcm_recv), lcm_pub(lcm_pub){
  verbose = false;
  
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.legodo.mode");

  if (strcmp(mode_str, "lin_rot_rate") == 0) {
    mode = MODE_LIN_AND_ROT_RATE;
    std::cout << "LegOdo will provide velocity and rotation rates." << std::endl;
  }
  else if (strcmp(mode_str, "lin_rate") == 0){
    mode = MODE_LIN_RATE;
    std::cout << "LegOdo will provide linear velocity rates but no rotation rates." << std::endl;
  }
  else{
    // ... incomplete...
  }

  /*
  if (strcmp(mode_str, "position") == 0) {
    mode = MODE_POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_yaw") == 0) {
    mode = MODE_POSITION_YAW;
    std::cout << "Scan matcher will provide position and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity") == 0) {
    mode = MODE_VELOCITY;
    std::cout << "Scan matcher will provide velocity measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity_yaw") == 0) {
    mode = MODE_VELOCITY_YAW;
    std::cout << "Scan matcher will provide velocity and yaw measurements." << std::endl;
  }
  else {
    mode = MODE_VELOCITY;
    std::cout << "Unrecognized scan matcher mode. Using velocity mode by default." << std::endl;
  }
  */

  free(mode_str);
  Eigen::VectorXd R_legodo;

  if (mode == MODE_LIN_AND_ROT_RATE) {
    z_indices.resize(6);
    R_legodo.resize(6);
  }
  else if (mode == MODE_LIN_RATE){
    z_indices.resize(3);
    R_legodo.resize(3);
  }
  else{
    // ... incomplete...
  }


  // Initialize covariance matrix based on mode.
  if (mode == MODE_LIN_AND_ROT_RATE) {
    double R_legodo_vxyz = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vxyz");
    double R_legodo_vang = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vang");
    R_legodo(0) = bot_sq(R_legodo_vxyz);
    R_legodo(1) = bot_sq(R_legodo_vxyz);
    R_legodo(2) = bot_sq(R_legodo_vxyz);
    R_legodo(3) = bot_sq(R_legodo_vang);
    R_legodo(4) = bot_sq(R_legodo_vang);
    R_legodo(5) = bot_sq(R_legodo_vang);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
    z_indices.tail<3>() = eigen_utils::RigidBodyState::angularVelocityInds();


  }else if (mode == MODE_LIN_RATE){
    double R_legodo_vxyz = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vxyz");
    R_legodo(0) = bot_sq(R_legodo_vxyz);
    R_legodo(1) = bot_sq(R_legodo_vxyz);
    R_legodo(2) = bot_sq(R_legodo_vxyz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }else{
    // ..incomplete
  }


  /*
  if (mode == MODE_POSITION || mode == MODE_POSITION_YAW) {
    double r_scan_match_pxy = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_pxy");
    double r_scan_match_pz = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_pz");
    R_legodo(0) = bot_sq(r_scan_match_pxy); // Cleaner way?
    R_legodo(1) = bot_sq(r_scan_match_pxy);
    R_legodo(2) = bot_sq(r_scan_match_pz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
  }
  else {
    double r_scan_match_vxy = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vxy");
    double r_scan_match_vz = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vz");
    R_legodo(0) = bot_sq(r_scan_match_vxy); // Cleaner way?
    R_legodo(1) = bot_sq(r_scan_match_vxy);
    R_legodo(2) = bot_sq(r_scan_match_vz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }

  if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_yaw");
    R_legodo(3) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }
  */  
  
  cov_legodo = R_legodo.asDiagonal();
}


void printTrans(BotTrans bt, std::string message){
  std::cout << message << ": "
      << bt.trans_vec[0] << ", " << bt.trans_vec[1] << ", " << bt.trans_vec[2] << " | "
      << bt.rot_quat[0] << ", " << bt.rot_quat[1] << ", " << bt.rot_quat[2] << ", " << bt.rot_quat[3] << "\n";
}



// NOTE: this inserts the BotTrans trans_vec 
// as the velocity components in the pose 
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

RBISUpdateInterface * LegOdoCommon::createMeasurement(BotTrans &msgT, int64_t utime, int64_t prev_utime){
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(msgT.rot_quat,rpy);
  double elapsed_time = ( (double) utime -  prev_utime)/1000000;
  double rpy_rate[3];
  rpy_rate[0] = rpy[0]/elapsed_time;
  rpy_rate[1] = rpy[1]/elapsed_time;
  rpy_rate[2] = rpy[2]/elapsed_time;
  
  if (verbose){
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

  
  
  BotTrans msgT_vel;
  memset(&msgT_vel, 0, sizeof(msgT_vel));
  
  msgT_vel.trans_vec[0] = msgT.trans_vec[0]/elapsed_time;
  msgT_vel.trans_vec[1] = msgT.trans_vec[1]/elapsed_time;
  msgT_vel.trans_vec[2] = msgT.trans_vec[2]/elapsed_time;
  bot_roll_pitch_yaw_to_quat (rpy_rate, msgT_vel.rot_quat);
  
  
  if (verbose){
    std::stringstream ss2;
    ss2 << " msgT_vel: ";
    printTrans(msgT_vel, ss2.str() );
    std::cout << "\n\n";
  }
  
  // Get the velocity as a pose message for visualization with signal scope:
  bot_core::pose_t vel_pose = getBotTransAsBotPoseVelocity(msgT_vel, utime)  ;
  lcm_pub->publish("POSE_BODY_LEGODO_VELOCITY", &vel_pose );   

  if (mode == MODE_LIN_AND_ROT_RATE) {
    // Working on this:

    Eigen::VectorXd z_meas(6);
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msgT_vel.rot_quat);
    z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msgT_vel.trans_vec);

    //  eigen_utils::botDoubleToQuaternion(quat, msg->quat);
    //  z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->trans);

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_legodo, quat, RBISUpdateInterface::legodo,
            utime);

  }else if (mode == MODE_LIN_RATE) {

    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>( msgT_vel.trans_vec ), cov_legodo, RBISUpdateInterface::legodo,
        utime);
    //RBISIndexedMeasurement(RBIS::velocity_inds(),
    //VO_velocity_measurement_body_coords,VO_measurement_cov_body_coords,
    //vo_sensor_id <you'll need to add this>, utime);

  }else{
    std::cout << "LegOdometry Mode not supported, exiting\n";
    return NULL;
  }


    /*
    if (mode == MODE_POSITION) {
      return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
          Eigen::Map<const Eigen::Vector3d>(msg->pos), cov_scan_match, RBISUpdateInterface::scan_matcher,
          utime);
    }
  else if (mode == MODE_VELOCITY) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->vel), cov_legodo, RBISUpdateInterface::scan_matcher,
        utime);
  }
  else if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    Eigen::Vector4d z_meas;
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msg->orientation);

    if (mode == MODE_POSITION_YAW) {
      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->pos);
    }
    else {
      z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->vel);
    }

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_legodo, quat,
        RBISUpdateInterface::scan_matcher, utime);
  }
  */
}

}