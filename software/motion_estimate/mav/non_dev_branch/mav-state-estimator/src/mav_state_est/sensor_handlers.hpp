#ifndef __SENSOR_HANDLERS_HPP__
#define __SENSOR_HANDLERS_HPP__

#include <stdint.h>
#include <string>
#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <eigen_utils/eigen_utils.hpp>
#include "rbis_update_interface.hpp"
#include <lcmtypes/mav_ins_t.h>
#include <lcmtypes/mav_gps_data_t.h>
#include <lcmtypes/mav_optical_flow_t.h>

#define VICON_CHANNEL_DEFAULT "VICON_fixie"
#define INS_CHANNEL_DEFAULT "MICROSTRAIN_INS"
#define SCAN_MATCHER_CHANNEL_DEFAULT "SCAN_MATCH_POS"
#define OPTICAL_FLOW_CHANNEL_DEFAULT "OPTICAL_FLOW"

//base class for getting minimal params for a sensor handler
class SensorHandler {
public:
  SensorHandler(BotParam * param, const std::string & sensor_prefix);

  std::string channel;
  int64_t utime_delay;

  int downsample_factor;
  int counter;
};

class InsHandler: public SensorHandler {
public:
  InsHandler(BotParam * param, BotFrames * frames);
  RBISUpdateInterface * processMessage(const mav_ins_t * msg);

  BotTrans ins_to_body;

  double cov_accel;
  double cov_gyro;
  double cov_accel_bias;
  double cov_gyro_bias;

  double dt;
};

class ViconHandler: public SensorHandler {
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_ORIENT
  } ViconMode;

  ViconHandler(BotParam * param);
  ViconHandler(BotParam * param, ViconMode vicon_mode); // get mode directly
  void init(BotParam * param); // make private?
  RBISUpdateInterface * processMessage(const bot_core_rigid_transform_t * msg);

  ViconMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_vicon;
};

class GpsHandler: public SensorHandler {
public:
  GpsHandler(BotParam * param);
  RBISUpdateInterface * processMessage(const mav_gps_data_t * msg);

  Eigen::Matrix3d cov_xyz;
};

class IndexedMeasurementHandler: public SensorHandler {
public:
  IndexedMeasurementHandler(BotParam * param, const std::string & sensor_name);
  RBISUpdateInterface * processMessage(const mav_indexed_measurement_t * msg);

};

class ScanMatcherHandler: public SensorHandler {
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_YAW, MODE_VELOCITY, MODE_VELOCITY_YAW
  } ScanMatchingMode;

  ScanMatcherHandler(BotParam * param);
  RBISUpdateInterface * processMessage(const bot_core_pose_t * msg);

  ScanMatchingMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_scan_match;
};

class OpticalFlowHandler: public SensorHandler {
public:
  OpticalFlowHandler(BotParam * param, BotFrames * frames);
  RBISUpdateInterface * processMessage(const mav_optical_flow_t * msg);

  BotTrans body_to_cam;
  Eigen::Vector3d body_to_cam_trans; // In body frame, not camera frame
  Eigen::Matrix3d body_to_cam_rot;

  Eigen::Vector4d z_xyrs; // data storage vector
  Eigen::Matrix4d cov_xyrs; // x, y, rot, scale
};

#endif /* __SENSOR_HANDLERS_HPP__ */
