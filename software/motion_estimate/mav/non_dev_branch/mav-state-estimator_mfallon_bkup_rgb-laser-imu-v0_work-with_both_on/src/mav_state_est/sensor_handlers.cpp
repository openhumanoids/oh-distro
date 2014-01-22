#include "sensor_handlers.hpp"

SensorHandler::SensorHandler(BotParam * param, const std::string & sensor_prefix)
{
  std::string param_prefix = "state_estimator." + sensor_prefix;
  downsample_factor = bot_param_get_int_or_fail(param, (param_prefix + ".downsample_factor").c_str());
  char * chan = bot_param_get_str_or_fail(param, (param_prefix + ".channel").c_str());
  channel = chan;
  free(chan);
  utime_delay = bot_param_get_int_or_fail(param, (param_prefix + ".utime_offset").c_str());
  counter = 0;
}

InsHandler::InsHandler(BotParam * param, BotFrames * frames) :
    SensorHandler(param, "ins")
{
  cov_gyro = bot_param_get_double_or_fail(param, "state_estimator.ins.q_gyro");
  cov_gyro = bot_sq(bot_to_radians(cov_gyro));
  cov_accel = bot_param_get_double_or_fail(param, "state_estimator.ins.q_accel");
  cov_accel = bot_sq(cov_accel);
  cov_gyro_bias = bot_param_get_double_or_fail(param, "state_estimator.ins.q_gyro_bias");
  cov_gyro_bias = bot_sq(bot_to_radians(cov_gyro_bias));
  cov_accel_bias = bot_param_get_double_or_fail(param, "state_estimator.ins.q_accel_bias");
  cov_accel_bias = bot_sq(cov_accel_bias);

  dt = bot_param_get_double_or_fail(param, "state_estimator.ins.timestep_dt"); // nominally dt = 0.01 for 100 Hz IMU messages
  dt = dt / (double) downsample_factor;

  bot_frames_get_trans(frames, "microstrain", "body", &ins_to_body);

}

RBISUpdateInterface * InsHandler::processMessage(const mav_ins_t * msg)
{
  if (counter++ % downsample_factor != 0)
    return NULL;

  //    get everything into the right frame
  double body_accel[3];
  bot_trans_apply_vec(&ins_to_body, msg->accel, body_accel);
  Eigen::Map<Eigen::Vector3d> accelerometer(body_accel);

  double body_gyro[3];
  bot_trans_apply_vec(&ins_to_body, msg->gyro, body_gyro);
  Eigen::Map<Eigen::Vector3d> gyro(body_gyro);

  //    double body_mag[3]; //not used
  //    bot_trans_apply_vec(&ins_to_body, msg->mag, body_mag);
  //    Eigen::Map<Eigen::Vector3d> mag(body_mag);

  return new RBISIMUProcessStep(gyro, accelerometer, cov_gyro, cov_accel, cov_gyro_bias, cov_accel_bias, dt,
      msg->utime - utime_delay);
}

ViconHandler::ViconHandler(BotParam * param) :
    SensorHandler(param, "vicon")
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.vicon.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ViconHandler::MODE_POSITION;
    std::cout << "Vicon will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_orient") == 0) {
    mode = ViconHandler::MODE_POSITION_ORIENT;
    std::cout << "Vicon will provide position and orientation measurements." << std::endl;
  }
  else {
    mode = ViconHandler::MODE_POSITION;
    std::cout << "Unrecognized Vicon mode. Using position mode by default." << std::endl;
  }

  free(mode_str);
  init(param);
}

ViconHandler::ViconHandler(BotParam * param, ViconMode vicon_mode) :
    SensorHandler(param, "vicon"), mode(vicon_mode)
{
  init(param);
}

void ViconHandler::init(BotParam * param)
{
  // Build full covariance matrix - we may only use part of it.
  double r_vicon_xyz = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_xyz");
  double r_vicon_chi = bot_param_get_double_or_fail(param, "state_estimator.vicon.r_chi");

  cov_vicon = Eigen::MatrixXd::Zero(6, 6);
  cov_vicon.topLeftCorner<3, 3>() = pow(r_vicon_xyz, 2) * Eigen::Matrix3d::Identity();
  cov_vicon.bottomRightCorner<3, 3>() = pow(bot_to_radians(r_vicon_chi), 2) * Eigen::Matrix3d::Identity();

  if (mode == MODE_POSITION) {
    z_indices = RBIS::positionInds();
  }
  else {
    z_indices.resize(6);
    z_indices.head<3>() = RBIS::positionInds();
    z_indices.tail<3>() = RBIS::chiInds();
  }
}

RBISUpdateInterface * ViconHandler::processMessage(const bot_core_rigid_transform_t * msg)
{
  if (counter++ % downsample_factor != 0)
    return NULL;

  if ((Eigen::Map<const Eigen::Array3d>(msg->trans).abs() < 1e-5).all())
    return NULL;

  if (mode == MODE_POSITION) {
    return new RBISIndexedMeasurement(z_indices, Eigen::Map<const Eigen::Vector3d>(msg->trans),
        cov_vicon.block<3, 3>(0, 0), RBISUpdateInterface::vicon,
        msg->utime - utime_delay);
  }
  else {
    Eigen::VectorXd z_meas(6);
    Eigen::Quaterniond quat;
    eigen_utils::botDoubleToQuaternion(quat, msg->quat);

    z_meas.head<3>() = Eigen::Map<const Eigen::Vector3d>(msg->trans);

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_vicon, quat, RBISUpdateInterface::vicon,
        msg->utime - utime_delay);
  }
}

GpsHandler::GpsHandler(BotParam * param) :
    SensorHandler(param, "gps")
{
  double r_gps_xy = bot_param_get_double_or_fail(param, "state_estimator.gps.r_xy");
  double r_gps_z = bot_param_get_double_or_fail(param, "state_estimator.gps.r_z");
  Eigen::Vector3d R_gps_diagonal = Eigen::Array3d(r_gps_xy, r_gps_xy, r_gps_z).pow(2);
  cov_xyz = R_gps_diagonal.asDiagonal();
}

RBISUpdateInterface * GpsHandler::processMessage(const mav_gps_data_t * msg)
{
  if (counter++ % downsample_factor != 0)
    return NULL;

  return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
      Eigen::Map<const Eigen::Vector3d>(msg->xyz_pos), cov_xyz, RBISUpdateInterface::gps, msg->utime - utime_delay);
}

IndexedMeasurementHandler::IndexedMeasurementHandler(BotParam * param, const std::string & sensor_name) :
    SensorHandler(param, sensor_name)
{
}

RBISUpdateInterface * IndexedMeasurementHandler::processMessage(const mav_indexed_measurement_t * msg)
{
  if (counter++ % downsample_factor != 0)
    return NULL;

  return new RBISIndexedMeasurement(Eigen::Map<const Eigen::VectorXi>(msg->z_indices, msg->measured_dim),
      Eigen::Map<const Eigen::VectorXd>(msg->z_effective, msg->measured_dim),
      Eigen::Map<const Eigen::MatrixXd>(msg->R_effective, msg->measured_dim, msg->measured_dim),
      RBISUpdateInterface::laser_gpf, msg->utime - utime_delay);
}

ScanMatcherHandler::ScanMatcherHandler(BotParam * param) :
    SensorHandler(param, "scan_matcher")
{
  char* mode_str = bot_param_get_str_or_fail(param, "state_estimator.scan_matcher.mode");

  if (strcmp(mode_str, "position") == 0) {
    mode = ScanMatcherHandler::MODE_POSITION;
    std::cout << "Scan matcher will provide position measurements." << std::endl;
  }
  else if (strcmp(mode_str, "position_yaw") == 0) {
    mode = ScanMatcherHandler::MODE_POSITION_YAW;
    std::cout << "Scan matcher will provide position and yaw measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity") == 0) {
    mode = ScanMatcherHandler::MODE_VELOCITY;
    std::cout << "Scan matcher will provide velocity measurements." << std::endl;
  }
  else if (strcmp(mode_str, "velocity_yaw") == 0) {
    mode = ScanMatcherHandler::MODE_VELOCITY_YAW;
    std::cout << "Scan matcher will provide velocity and yaw measurements." << std::endl;
  }
  else {
    mode = ScanMatcherHandler::MODE_VELOCITY;
    std::cout << "Unrecognized scan matcher mode. Using velocity mode by default." << std::endl;
  }

  free(mode_str);
  Eigen::VectorXd R_scan_match;

  if (mode == MODE_POSITION || mode == MODE_VELOCITY) {
    z_indices.resize(3);
    R_scan_match.resize(3);
  }
  else {
    z_indices.resize(4); // Use yaw measurements too.
    R_scan_match.resize(4);
  }

  // Initialize covariance matrix based on mode.
  if (mode == MODE_POSITION || mode == MODE_POSITION_YAW) {
    double r_scan_match_pxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pxy");
    double r_scan_match_pz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_pz");
    R_scan_match(0) = bot_sq(r_scan_match_pxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_pxy);
    R_scan_match(2) = bot_sq(r_scan_match_pz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::positionInds();
  }
  else {
    double r_scan_match_vxy = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_vxy");
    double r_scan_match_vz = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_vz");
    R_scan_match(0) = bot_sq(r_scan_match_vxy); // Cleaner way?
    R_scan_match(1) = bot_sq(r_scan_match_vxy);
    R_scan_match(2) = bot_sq(r_scan_match_vz);
    z_indices.head<3>() = eigen_utils::RigidBodyState::velocityInds();
  }

  if (mode == MODE_POSITION_YAW || mode == MODE_VELOCITY_YAW) {
    double r_scan_match_yaw = bot_param_get_double_or_fail(param, "state_estimator.scan_matcher.r_yaw");
    R_scan_match(3) = bot_sq(bot_to_radians(r_scan_match_yaw));
    z_indices(3) = RBIS::chi_ind + 2; // z component only
  }

  cov_scan_match = R_scan_match.asDiagonal();
}

RBISUpdateInterface * ScanMatcherHandler::processMessage(const bot_core_pose_t * msg)
{
  if (counter++ % downsample_factor != 0)
    return NULL;

  if (mode == MODE_POSITION) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::positionInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->pos), cov_scan_match, RBISUpdateInterface::scan_matcher,
        msg->utime - utime_delay);
  }
  else if (mode == MODE_VELOCITY) {
    return new RBISIndexedMeasurement(eigen_utils::RigidBodyState::velocityInds(),
        Eigen::Map<const Eigen::Vector3d>(msg->vel), cov_scan_match, RBISUpdateInterface::scan_matcher,
        msg->utime - utime_delay);
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

    return new RBISIndexedPlusOrientationMeasurement(z_indices, z_meas, cov_scan_match, quat,
        RBISUpdateInterface::scan_matcher, msg->utime - utime_delay);
  }
}

OpticalFlowHandler::OpticalFlowHandler(BotParam * param, BotFrames * frames) :
    SensorHandler(param, "optical_flow")
{
  bot_frames_get_trans(frames, "body", "camera", &body_to_cam);
  bot_trans_get_trans_vec(&body_to_cam, body_to_cam_trans.data());
  bot_trans_get_rot_mat_3x3(&body_to_cam, body_to_cam_rot.data());
//  cam_to_body_trans = cam_to_body_rot * cam_to_body_trans;

  Eigen::Vector4d R_optical_flow_xyrs;

  double r_optical_flow_x = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_ux");
  double r_optical_flow_y = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_uy");
  double r_optical_flow_r = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_r");
  double r_optical_flow_s = bot_param_get_double_or_fail(param, "state_estimator.optical_flow.r_s");

  R_optical_flow_xyrs
      << Eigen::Array4d(r_optical_flow_x, r_optical_flow_y, r_optical_flow_r, r_optical_flow_s).square();
  cov_xyrs = R_optical_flow_xyrs.asDiagonal();
}

RBISUpdateInterface * OpticalFlowHandler::processMessage(const mav_optical_flow_t * msg)
{
  if (counter++ % downsample_factor != 0)
    return NULL;

// Camera frame to body frame transform.
  z_xyrs(0) = msg->ux;
  z_xyrs(1) = msg->uy;
  z_xyrs(2) = msg->theta;
  z_xyrs(3) = msg->scale;

  double alpha1 = msg->alpha1;
  double alpha2 = msg->alpha2;
  double gamma = msg->gamma;

//  eigen_dump(z_xyrs);
//  eigen_dump(cov_xyrs);
//  eigen_dump(cam_to_body_trans);
//  eigen_dump(cam_to_body_rot);
  return new RBISOpticalFlowMeasurement(z_xyrs, cov_xyrs, body_to_cam_trans, body_to_cam_rot, alpha1, alpha2, gamma,
      RBISUpdateInterface::optical_flow, msg->utime - utime_delay);
}

