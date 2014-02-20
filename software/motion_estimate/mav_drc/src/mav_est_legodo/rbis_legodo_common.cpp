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

  free(mode_str);
  Eigen::VectorXd R_legodo;
  Eigen::VectorXd R_legodo_uncertain;

  if (mode == MODE_LIN_AND_ROT_RATE) {
    z_indices.resize(6);
    R_legodo.resize(6);
    R_legodo_uncertain.resize(6);
  }
  else if (mode == MODE_LIN_RATE){
    z_indices.resize(3);
    R_legodo.resize(3);
    R_legodo_uncertain.resize(3);
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
    
    double R_legodo_vxyz_uncertain = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vxyz_uncertain");
    double R_legodo_vang_uncertain = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vang_uncertain");
    R_legodo_uncertain(0) = bot_sq(R_legodo_vxyz_uncertain);
    R_legodo_uncertain(1) = bot_sq(R_legodo_vxyz_uncertain);
    R_legodo_uncertain(2) = bot_sq(R_legodo_vxyz_uncertain);
    R_legodo_uncertain(3) = bot_sq(R_legodo_vang_uncertain);
    R_legodo_uncertain(4) = bot_sq(R_legodo_vang_uncertain);
    R_legodo_uncertain(5) = bot_sq(R_legodo_vang_uncertain);
    
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
    z_indices.tail<3>() = eigen_utils::RigidBodyState::angularVelocityInds();
  }else if (mode == MODE_LIN_RATE){
    double R_legodo_vxyz = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vxyz");
    R_legodo(0) = bot_sq(R_legodo_vxyz);
    R_legodo(1) = bot_sq(R_legodo_vxyz);
    R_legodo(2) = bot_sq(R_legodo_vxyz);
    
    double R_legodo_vxyz_uncertain = bot_param_get_double_or_fail(param, "state_estimator.legodo.r_vxyz_uncertain");
    R_legodo_uncertain(0) = bot_sq(R_legodo_vxyz_uncertain);
    R_legodo_uncertain(1) = bot_sq(R_legodo_vxyz_uncertain);
    R_legodo_uncertain(2) = bot_sq(R_legodo_vxyz_uncertain);    
    
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }else{
    // ..incomplete
  }
  
  cov_legodo = R_legodo.asDiagonal();
  cov_legodo_uncertain = R_legodo_uncertain.asDiagonal();
}


void printTrans(BotTrans bt, std::string message){
  std::cout << message << ": "
      << bt.trans_vec[0] << ", " << bt.trans_vec[1] << ", " << bt.trans_vec[2] << " | "
      << bt.rot_quat[0] << ", " << bt.rot_quat[1] << ", " << bt.rot_quat[2] << ", " << bt.rot_quat[3] << "\n";
}


// Difference the transform and scale by elapsed time:
BotTrans LegOdoCommon::getTransAsVelocityTrans(BotTrans msgT, int64_t utime, int64_t prev_utime){
  BotTrans msgT_vel;
  memset(&msgT_vel, 0, sizeof(msgT_vel));
  
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
  
  return msgT_vel;
}


RBISUpdateInterface * LegOdoCommon::createMeasurement(BotTrans &msgT, 
                                                      int64_t utime, int64_t prev_utime, 
                                                      float odometry_status){
  BotTrans msgT_vel = getTransAsVelocityTrans(msgT, utime, prev_utime);
  
  Eigen::MatrixXd cov_legodo_use;
  if (odometry_status < 0.5){ // low variance, high reliable
    cov_legodo_use = cov_legodo;
  }else{ // high variance, low reliable e.g. breaking contact
    cov_legodo_use = cov_legodo_uncertain;
  }
  

  if (mode == MODE_LIN_AND_ROT_RATE) {
    // Working on this:

    Eigen::VectorXd z_meas(6);
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msgT_vel.rot_quat);
    z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msgT_vel.trans_vec);

    //  eigen_utils::botDoubleToQuaternion(quat, msg->quat);
    //  z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->trans);

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_legodo_use, quat, RBISUpdateInterface::legodo,
            utime);

  }else if (mode == MODE_LIN_RATE) {

    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>( msgT_vel.trans_vec ), cov_legodo_use, RBISUpdateInterface::legodo,
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