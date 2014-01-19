#include "Initializer.hpp"

using namespace std;
using namespace Eigen;

void Initializer::init(lcm_t * _lcm, BotParam * _param, BotFrames * _frames)
{
  lcm = _lcm;
  param = _param;
  frames = _frames;
  done = false;
  insDone = false;
  havePos = false;
  overrideHeading = false;
  ins_init_samples_counter = 0;

  loadDefaultSigmas();

  //-------------ins initialization variables -------------
  g_vec_sum = Vector3d::Zero();
  mag_vec_sum = Vector3d::Zero();
  gyro_bias_sum = Vector3d::Zero();

  //subscribe to INS messages
  num_ins_to_init = 100; //TODO: pass in/param?
  ins_sensor_handler = new InsHandler(param, _frames);
  ins_sub = mav_ins_t_subscribe(lcm, ins_sensor_handler->channel.c_str(), ins_message_handler_static, (void *) this);
  gps_sensor_handler = new GpsHandler(param);
  gps_sub = NULL;
  vicon_sensor_handler = new ViconHandler(param, ViconHandler::MODE_POSITION);
  vicon_sub = NULL;
}

Initializer::Initializer(lcm_t * _lcm, BotParam * _param, BotFrames * _frames,
    init_mode_t _init_mode, bool _no_publish)
{
  init(_lcm, _param, _frames);
  init_mode = _init_mode;
  no_publish = _no_publish;
  init_msg = NULL;

  switch (init_mode) {
  case origin_init:
    setPosition(Vector3d::Zero());
    setHeading(0);
    fprintf(stderr, "initializing at the origin\n");
    break;
  case gps_init:
    gps_sub = mav_gps_data_t_subscribe(lcm, gps_sensor_handler->channel.c_str(), gps_message_handler_static, this);
    fprintf(stderr, "initializing from GPS\n");
    break;
  case vicon_init:
    vicon_sub = bot_core_rigid_transform_t_subscribe(lcm, vicon_sensor_handler->channel.c_str(),
        vicon_message_handler_static,
        this);
    fprintf(stderr, "initializing from vicon\n");
    break;
  case external_init:
    break;
  default:
    fprintf(stderr, "ERROR: invalid initialization mode: %d\n", init_mode);
    exit(1);
  }

}

Initializer::Initializer(lcm_t * _lcm, BotParam * _param, BotFrames * _frames,
    const Eigen::Vector3d & _init_xyz, const Eigen::Matrix3d & _init_xyz_cov)
{
  init(_lcm, _param, _frames);
  init_mode = external_init;
  setPosition(_init_xyz, _init_xyz_cov);
}

Initializer::Initializer(lcm_t * _lcm, BotParam * _param, BotFrames * _frames,
    const Eigen::Vector3d & _init_xyz, const Eigen::Matrix3d & _init_xyz_cov,
    double _init_heading, double _init_heading_cov)
{
  init(_lcm, _param, _frames);
  init_mode = external_init;
  setPosition(_init_xyz, _init_xyz_cov);
  setHeading(_init_heading, _init_heading_cov);
}

void Initializer::setPosition(const Eigen::Vector3d & _init_xyz, const Eigen::Matrix3d & _init_xyz_cov)
{
  init_xyz = _init_xyz;

  //covariance isn't negative, so don't use default
  if (_init_xyz_cov(0, 0) >= 0) {
    init_xyz_cov = _init_xyz_cov;
  }

  havePos = true;
}

void Initializer::setHeading(double _init_heading, double _init_heading_cov)
{
  init_heading = _init_heading;
  if (_init_heading_cov>0){
    init_heading_cov = _init_heading_cov;
    init_chi_cov(2, 2) = init_heading_cov;
  }
  overrideHeading = true;


}

void Initializer::loadDefaultSigmas()
{
  sigma_Delta_xy_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_xy");
  sigma_Delta_z_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.Delta_z");
  sigma_chi_xy_init = bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_xy"));
  sigma_chi_z_init = bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.chi_z"));
  sigma_vb_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.vb");
  sigma_gyro_bias_init = bot_to_radians(bot_param_get_double_or_fail(param, "state_estimator.sigma0.gyro_bias"));
  sigma_accelerometer_bias_init = bot_param_get_double_or_fail(param, "state_estimator.sigma0.accel_bias");

  Vector3d xyz_cov_diag = Eigen::Array3d(sigma_Delta_xy_init, sigma_Delta_xy_init, sigma_Delta_z_init).square();
  init_xyz_cov = xyz_cov_diag.asDiagonal();

  Vector3d init_chi_cov_diag = Eigen::Array3d(sigma_chi_xy_init, sigma_chi_xy_init, sigma_chi_z_init).square();
  init_chi_cov = init_chi_cov_diag.asDiagonal();
}

Initializer::~Initializer()
{
  if (ins_sub != NULL)
    mav_ins_t_unsubscribe(lcm, ins_sub);
  if (vicon_sub != NULL)
    bot_core_rigid_transform_t_unsubscribe(lcm, vicon_sub);
  if (gps_sub != NULL)
    mav_gps_data_t_unsubscribe(lcm, gps_sub);

  if (init_msg != NULL)
    mav_filter_state_t_destroy(init_msg);

  delete ins_sensor_handler;
  delete gps_sensor_handler;
  delete vicon_sensor_handler;
}

void Initializer::ins_message_handler(const mav_ins_t * msg)
{
  RBISIMUProcessStep * update = dynamic_cast<RBISIMUProcessStep *>(ins_sensor_handler->processMessage(msg));
  if (update == NULL)
    return;
  //need to compute the body mag seperately cuz it's not used in the ins_process update
  Eigen::Vector3d mag;
  bot_trans_apply_vec(&ins_sensor_handler->ins_to_body, msg->mag, mag.data());

  //add teh values to the accumulators
  g_vec_sum += -update->accelerometer;
  mag_vec_sum += mag;
  gyro_bias_sum += update->gyro;
  ins_init_samples_counter++;
  delete update;

  if (ins_init_samples_counter >= num_ins_to_init) {
    printf("filled, about to send init mfallon\n");

    insDone = true;
    mav_ins_t_unsubscribe(lcm, ins_sub);
    ins_sub = NULL;
    publish_initialize(msg->utime);
  }
}

void Initializer::vicon_message_handler(const bot_core_rigid_transform_t * msg)
{

  Vector3d vicon_xyz = Map<const Vector3d>(msg->trans);
  Quaterniond vicon_quat(msg->quat[0], msg->quat[1], msg->quat[2], msg->quat[3]);

  double rpy[3];
  bot_quat_to_roll_pitch_yaw(msg->quat, rpy);
  double vicon_heading = rpy[2];

  setPosition(vicon_xyz, vicon_sensor_handler->cov_vicon.block<3, 3>(0, 0));
  setHeading(vicon_heading, vicon_sensor_handler->cov_vicon(5, 5));

  bot_core_rigid_transform_t_unsubscribe(lcm, vicon_sub);
  vicon_sub = NULL;
  publish_initialize(msg->utime);

}

void Initializer::gps_message_handler(const mav_gps_data_t * msg)
{

  Vector3d gps_xyz = Map<const Vector3d>(msg->xyz_pos);

  setPosition(gps_xyz, gps_sensor_handler->cov_xyz);

  mav_gps_data_t_unsubscribe(lcm, gps_sub);
  gps_sub = NULL;
  publish_initialize(msg->utime);
}

void Initializer::computeINSInitialEstimates()
{

  if (overrideHeading) {
    //set the mag_vec to the heading vector clicked out so it will align with the x axis
    mag_vec_sum.head(2) = eigen_utils::angleToVec(init_heading + M_PI / 2);
    mag_vec_sum(2) = 0;
  }

  Vector3d ins_mag_vec_est;
  Vector3d ins_g_vec_est;
  ins_gyro_bias_est;

  ins_g_vec_est = g_vec_sum / (double) ins_init_samples_counter;
  ins_mag_vec_est = mag_vec_sum / (double) ins_init_samples_counter;
  ins_gyro_bias_est = gyro_bias_sum / (double) ins_init_samples_counter;
  ins_mag_vec_est(2) = 0; //make sure we're only trying to align in the xy plane

//set orientation
  Quaterniond quat_g_vec, quat_mag;
  quat_mag.setFromTwoVectors(ins_mag_vec_est, Vector3d::UnitY()); //in ENU, the magnetic vector should be aligned with Y axis
  quat_g_vec.setFromTwoVectors(ins_g_vec_est, -Vector3d::UnitZ()); //the gravity vector points in the negative z axis

  eigen_dump(ins_mag_vec_est);
  eigen_dump(ins_g_vec_est);
  eigen_dump(quat_mag.toRotationMatrix().eulerAngles(2, 1, 0) * 180 / M_PI);
  eigen_dump(quat_g_vec.toRotationMatrix().eulerAngles(2, 1, 0) * 180 / M_PI);
  ins_quat_est = quat_mag * quat_g_vec;

}

void Initializer::publish_initialize(int64_t utime)
{
  if (!havePos || !insDone)
    return;

  printf("p1\n");
  computeINSInitialEstimates();
  printf("p2\n");

  //setting the bias covs to zeros forces the values to be static which is what we want for now
  init_cov.setZero();
  //  init_cov = RBIM::Identity() * bot_sq(0);

  printf("p3\n");


  //set all the sub-blocks of the covariance matrix
  init_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = bot_sq(sigma_vb_init) * Matrix3d::Identity();
  init_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = init_chi_cov;
  init_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = init_xyz_cov;
  init_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Matrix3d::Identity()
      * bot_sq(sigma_gyro_bias_init);
  init_cov.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Matrix3d::Identity()
      * bot_sq(sigma_accelerometer_bias_init);

  printf("p4\n");

  // angularVelocity and acceleration aren't true filter states so they can be 0
  init_state.vec.setZero();
  init_state.quat = ins_quat_est;
  init_state.position() = init_xyz;
  init_state.gyroBias() = ins_gyro_bias_est;

  printf("p5\n");

  //create the initialization message
  if (init_msg != NULL) {
    //mav_filter_state_t_destroy(init_msg);
    // mfallon disabled
  }
  printf("p6\n");
  init_msg = rbisCreateFilterStateMessage(init_state, init_cov);
  init_msg->utime = utime;

  printf("p7\n");

  if (!no_publish)
    mav_filter_state_t_publish(lcm, "MAV_STATE_EST_INITIALIZER", init_msg);

  printf("p8\n");

  //print out
  Vector3d eulers = ins_quat_est.toRotationMatrix().eulerAngles(2, 1, 0);
  eulers *= 180 / M_PI;

  eigen_dump(init_state);
  eigen_dump(init_cov);

  std::cerr << "initialized position to: (x,y,z): " << init_xyz.transpose() << std::endl;
  std::cerr << "initialized orientation to: (phi,theta,psi): " << eulers.reverse().transpose() << std::endl;
  std::cerr << "initialized gyro bias at:\n " << bot_to_degrees(ins_gyro_bias_est).transpose() << " degrees/second\n\n";

  done = true;
  init_utime = utime;

}

