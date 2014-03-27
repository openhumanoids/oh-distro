#ifndef INITIALIZER_HPP_
#define INITIALIZER_HPP_
#include <Eigen/Dense>
#include <bot_frames/bot_frames.h>
#include <mav_state_est/sensor_handlers.hpp>
#include <mav_state_est/rbis.hpp>

class Initializer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef enum {
    origin_init = 1,
    gps_init = 2,
    vicon_init = 3,
    external_init = 4
  } init_mode_t;

  init_mode_t init_mode;
  int num_ins_to_init;
  int ins_init_samples_counter;
  bool insDone;

  //handlers
  InsHandler * ins_sensor_handler;
  mav_ins_t_subscription_t * ins_sub;

  ViconHandler * vicon_sensor_handler;
  bot_core_rigid_transform_t_subscription_t * vicon_sub;

  GpsHandler * gps_sensor_handler;
  mav_gps_data_t_subscription_t * gps_sub;

  lcm_t * lcm;
  BotParam * param;
  BotFrames * frames;

  bool havePos;
  Eigen::Vector3d init_xyz;
  Eigen::Matrix3d init_xyz_cov;
  Eigen::Matrix3d init_chi_cov;

  bool overrideHeading;
  double init_heading;
  double init_heading_cov;

  //final initialization values
  bool done;
  bool no_publish;
  int64_t init_utime;
  RBIS init_state;
  RBIM init_cov;
  mav_filter_state_t * init_msg;

  //for averaging IMU
  Eigen::Vector3d g_vec_sum;
  Eigen::Vector3d mag_vec_sum;
  Eigen::Vector3d gyro_bias_sum;

  //estimated values from INS
  Eigen::Quaterniond ins_quat_est;
  Eigen::Vector3d ins_gyro_bias_est;

  //default Sigma values
  double sigma_Delta_xy_init;
  double sigma_Delta_z_init;
  double sigma_chi_xy_init;
  double sigma_chi_z_init;
  double sigma_vb_init;
  double sigma_gyro_bias_init;
  double sigma_accelerometer_bias_init;

  void init(lcm_t * _lcm, BotParam * _param, BotFrames * _frames);
  Initializer(lcm_t * _lcm, BotParam * _param, BotFrames * _frames,
      init_mode_t init_mode, bool _no_publish = false);


  Initializer(lcm_t * lcm, BotParam * param, BotFrames * frames,
      const Eigen::Vector3d & init_xyz, const Eigen::Matrix3d &init_xyz_cov);
  Initializer(lcm_t * lcm, BotParam * param, BotFrames * frames,
      const Eigen::Vector3d & init_xyz, const Eigen::Matrix3d &init_xyz_cov,
      double init_heading, double init_heading_cov);

  ~Initializer();

  void loadDefaultSigmas();

  void setPosition(const Eigen::Vector3d & _init_xyz,
      const Eigen::Matrix3d & _init_xyz_cov = Eigen::Matrix3d::Constant(-1));
  void setHeading(double _init_heading, double _init_heading_cov = -1);

  void computeINSInitialEstimates();
  void publish_initialize(int64_t utime);

  void ins_message_handler(const mav_ins_t * msg);
  void vicon_message_handler(const bot_core_rigid_transform_t * msg);
  void gps_message_handler(const mav_gps_data_t * msg);

  static void ins_message_handler_static(const lcm_recv_buf_t *rbuf, const char * channel, const mav_ins_t * msg,
      void * user)
  {
    Initializer * self = (Initializer*) user;
    self->ins_message_handler(msg);
  }
  static void vicon_message_handler_static(const lcm_recv_buf_t *rbuf, const char * channel,
      const bot_core_rigid_transform_t * msg,
      void * user)
  {
    Initializer * self = (Initializer*) user;
    self->vicon_message_handler(msg);
  }
  static void gps_message_handler_static(const lcm_recv_buf_t *rbuf, const char * channel, const mav_gps_data_t * msg,
      void * user)
  {
    Initializer * self = (Initializer*) user;
    self->gps_message_handler(msg);
  }
};

#endif /* INITIALIZER_HPP_ */
