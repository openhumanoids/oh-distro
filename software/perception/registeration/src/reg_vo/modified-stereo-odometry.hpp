#ifndef __fovis_bot2_stereo_odometry_hpp__
#define __fovis_bot2_stereo_odometry_hpp__

#include <iostream>
#include <fstream>
#include <sstream>

#include <signal.h>
#include <getopt.h>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <lcm/lcm.h>
#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/microstrain_ins_t.h>

#include <lcmtypes/reg_feature_t.h>
#include <lcmtypes/reg_features_t.h>

#ifdef USE_LCMGL
#include <bot_lcmgl_client/lcmgl.h>
#endif

// mfallon:
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <fovis/fovis.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

struct ImageFeature
{
  int track_id;
  Eigen::Vector2d uv; ///< unrectified, distorted, orig. coords
  Eigen::Vector2d base_uv; ///< unrectified, distorted, base level
  Eigen::Vector3d uvd; ///< rectified, undistorted, base level
  Eigen::Vector3d xyz;
  Eigen::Vector4d xyzw;
  uint8_t color[3];

  // @todo what more is needed?
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace fovis
{




class Visualization;

class StereoOdometry {
public:
  StereoOdometry();

  int initialize(int argc, char **argv);

  ~StereoOdometry();

  void go() {
    while((lcm_handle(_subscribe_lcm)==0) && !_shutdown_flag) {
      // nothing
    }
    if (_publish_tictoc) {
      publish_tictoc_stats();
    }

    fprintf(stderr, "Done.\n");
  }

private:
  StereoOdometry (const StereoOdometry& other);
  StereoOdometry& operator=(const StereoOdometry& other);

  static void sig_handler_aux(int signal, siginfo_t *s, void *data) {
    fprintf(stderr, "Shutting down\n");
    _shutdown_flag = 1;
  }

  static void image_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const bot_core_image_t* msg,
                                void* user_data) {
    ((StereoOdometry *) user_data)->image_handler(msg);
  }


  static void imu_handler_aux(const lcm_recv_buf_t* rbuf,
                                const char* channel,
                                const microstrain_ins_t* msg,
                                void* user_data) {
    ((StereoOdometry *) user_data)->imu_handler(msg);
  }

  void image_handler(const bot_core_image_t *msg);
  void imu_handler(const microstrain_ins_t *msg);

  static VisualOdometryOptions getDefaultOptions();

  int init_lcm();
  int init_calibration(const char * key_prefix);

  void decode_image(const bot_core_image_t *msg);
  void publish_motion_estimation();
  void publish_tictoc_stats();

  void usage(const char *prog_name);
  int parse_command_line_options(int argc, char **argv);

  // data members
  static sig_atomic_t _shutdown_flag;

  lcm_t* _subscribe_lcm;
  lcm_t* _publish_lcm;
  lcm_t* _bot_param_lcm;

  std::string _odom_channel;
  std::string _pose_channel;
  std::string _frame_update_channel;
  std::string _stats_channel;
  std::string _tictoc_channel;

  BotParam* _bot_param;
  BotFrames* _bot_frames; // added mfallon
  bot::frames* _bot_frames_cpp; // added mfallon
    
  VisualOdometry* _odom;
  StereoDepth* _depth_producer;
  StereoCalibrationParameters _stereo_params;

  std::string _input_log_fname;
  std::string _output_log_fname;

  std::string _vo_parameters_fname;
  std::string _camera_parameters_fname;
  std::string _camera_block; // mfallon

  bool _publish_pose;
  bool _publish_odometry;
  bool _publish_frame_update;
  bool _publish_stats;
  bool _publish_tictoc;

  int64_t _utime_cur;
  int64_t _utime_prev;

  uint8_t* _image_left_buf;
  uint8_t* _image_right_buf;
  uint8_t* _images_buf;
  size_t _buf_size;

  // IMU variables:
  int _imu_attitude; // 0 no-imu | 1 only for init | 2 reuse imu occasionally to reinit pitch and roll
  bool _imu_init;
  Eigen::Isometry3d _local_to_camera; // main pose estimated by the VO system
  int _imu_counter;

  const fovis::OdometryFrame* ref_odom_frame_;
  const fovis::OdometryFrame* target_odom_frame_;
  int64_t _ref_utime;
  Eigen::Isometry3d _ref_pose;

  // Copy of images to write upon the next frame:
  uint8_t* _image_left_buf_copy;
  uint8_t* _image_right_buf_copy;
  int _output_counter;


  pointcloud_vis* pc_vis_;
  void write_images();
  void write_ref_images();
  void writeFeatures(std::vector<ImageFeature> features);
  void sendFeatures(std::vector<ImageFeature> features);
  void sendFeaturesAsCollection(std::vector<ImageFeature> features, int vs_id);

#ifdef USE_LCMGL
  bool _draw_lcmgl;
  Visualization* _visualization;
#endif

};

}

#endif /* end of include guard: __fovis_bot2_stereo_odometry_hpp__ */
