#include "modified-stereo-odometry.hpp"

#include <bot_core/bot_core.h>

#include <fovis/fovis.hpp>
#include <fovis/tictoc.hpp>
#include <lcmtypes/fovis_update_t.h>
#include <lcmtypes/fovis_tictoc_t.h>
#include <lcmtypes/fovis_param_t.h>
#include <lcmtypes/fovis_stats_t.h>

#include "jpeg-utils.h"
#ifdef USE_LCMGL
#include "visualization.hpp"
#endif

namespace fovis
{

sig_atomic_t StereoOdometry::_shutdown_flag = 0;

int
StereoOdometry::init_calibration(const char * key_prefix)
{
  std::string key;
  std::string key_prefix_str;
  CameraIntrinsicsParameters * params;
  for (int i=0; i < 2; ++i) {
    if (i == 0) {
      key_prefix_str = std::string(key_prefix) + ".left";
      params = &(_stereo_params.left_parameters);
    } else {
      key_prefix_str = std::string(key_prefix) + ".right";

      params = &(_stereo_params.right_parameters);
    }
    params->width = bot_param_get_int_or_fail(_bot_param, (key_prefix_str+".width").c_str());
    params->height = bot_param_get_int_or_fail(_bot_param,(key_prefix_str+".height").c_str());
    params->fx = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".fx").c_str());
    params->fy = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".fy").c_str());
    params->cx = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".cx").c_str());
    params->cy = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".cy").c_str());
    params->k1 = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".k1").c_str());
    params->k2 = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".k2").c_str());
    params->k3 = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".k3").c_str());
    params->p1 = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".p1").c_str());
    params->p2 = bot_param_get_double_or_fail(_bot_param, (key_prefix_str+".p2").c_str());
  }

  // We assume rotation is a rotation matrix
  double rotation[9], translation[3];
  bot_param_get_double_array_or_fail(_bot_param,
                                     (key_prefix_str+".rotation").c_str(),
                                     &rotation[0],
                                     9);
  bot_param_get_double_array_or_fail(_bot_param,
                                     (key_prefix_str+".translation").c_str(),
                                     &translation[0],
                                     3);

  bot_matrix_to_quat(rotation, _stereo_params.right_to_left_rotation);
  std::copy(translation, translation+3, _stereo_params.right_to_left_translation);

  return 0;
}

void
StereoOdometry::publish_tictoc_stats()
{
  std::vector<tictoc_t> tictoc_stats;
  tictoc_get_stats(&tictoc_stats);
  // publish all tictoc entries as lcm messages
  for (std::vector<tictoc_t>::iterator itr = tictoc_stats.begin();
       itr != tictoc_stats.end();
       ++itr) {
    fovis_tictoc_t tictoc_msg;
    tictoc_msg.t = itr->t;
    tictoc_msg.totalT = itr->totalT;
    tictoc_msg.ema = itr->ema;
    tictoc_msg.min = itr->min;
    tictoc_msg.max = itr->max;
    tictoc_msg.numCalls = itr->numCalls;
    tictoc_msg.description = new char[itr->description.size()+1];
    std::copy(itr->description.begin(), itr->description.end(), &tictoc_msg.description[0]);
    tictoc_msg.description[itr->description.size()] = '\0';
    //std::cerr << "tictoc_msg.description = " << tictoc_msg.description << std::endl;
    fovis_tictoc_t_publish(_publish_lcm, _tictoc_channel.c_str(), &tictoc_msg);
    delete tictoc_msg.description;
  }
}

void
StereoOdometry::decode_image(const bot_core_image_t * msg)
{
  // extract image data
  // TODO add support for raw RGB
  switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      // in stereo the two images are simply one above the other, so the pointer for
      // the right image is at the half of the images_buf
      std::copy(msg->data             , msg->data+msg->size/2 , _image_left_buf);
      std::copy(msg->data+msg->size/2 , msg->data+msg->size   , _image_right_buf);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // for some reason msg->row_stride is 0, so we use msg->width instead.
      jpeg_decompress_8u_gray(msg->data,
                              msg->size,
                              _images_buf,
                              msg->width,
                              msg->height,
                              msg->width);
      std::copy(_images_buf           , _images_buf+_buf_size   , _image_left_buf);
      std::copy(_images_buf+_buf_size , _images_buf+2*_buf_size , _image_right_buf);
      break;
    default:
      fprintf(stderr, "Unrecognized image format\n");
      break;
  }
}

void
StereoOdometry::publish_motion_estimation()
{
  Eigen::Isometry3d cam_to_local = _odom->getPose();

  // rotate coordinate frame so that look vector is +X, and up is +Z
  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;
  cam_to_local = M * cam_to_local;
  Eigen::Vector3d translation(cam_to_local.translation());
  Eigen::Quaterniond rotation(cam_to_local.rotation());
  rotation = rotation * M.transpose();

  Eigen::Isometry3d motion_estimate = _odom->getMotionEstimate();
  fovis_update_t update_msg;
  update_msg.timestamp = _utime_cur;
  update_msg.prev_timestamp = _utime_prev;
  Eigen::Vector3d motion_T = motion_estimate.translation();
  update_msg.translation[0] = motion_T(0);
  update_msg.translation[1] = motion_T(1);
  update_msg.translation[2] = motion_T(2);
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(motion_estimate.rotation());
  update_msg.rotation[0] = motion_R.w();
  update_msg.rotation[1] = motion_R.x();
  update_msg.rotation[2] = motion_R.y();
  update_msg.rotation[3] = motion_R.z();

  //zero out matrix
  const Eigen::MatrixXd & motion_cov = _odom->getMotionEstimateCov();
  for (int i=0;i<6;i++)
    for (int j=0;j<6;j++)
      update_msg.covariance[i][j] = motion_cov(i,j);

  const MotionEstimator * me = _odom->getMotionEstimator();
  MotionEstimateStatusCode estim_status = _odom->getMotionEstimateStatus();
  switch(estim_status) {
    case NO_DATA:
      break;
    case SUCCESS:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_VALID;
      printf("Inliers: %4d  Rep. fail: %4d Matches: %4d Feats: %4d Mean err: %5.2f\n",
          me->getNumInliers(),
          me->getNumReprojectionFailures(),
          me->getNumMatches(),
          _odom->getTargetFrame()->getNumKeypoints(),
          me->getMeanInlierReprojectionError());
      break;
    case INSUFFICIENT_INLIERS:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_INSUFFICIENT_FEATURES;
      printf("Insufficient inliers\n");
      break;
    case OPTIMIZATION_FAILURE:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_DEGENERATE;
      printf("Unable to solve for rigid body transform\n");
      break;
    case REPROJECTION_ERROR:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_REPROJECTION_ERROR;
      printf("Excessive reprojection error (%f).\n", me->getMeanInlierReprojectionError());
      break;
    default:
      printf("Unknown error (this should never happen)\n");
      break;
  }

  if (estim_status != NO_DATA) {
    // TODO why does this get published to odom channel?
    fovis_update_t_publish(_publish_lcm,
                           _odom_channel.c_str(),
                           &update_msg);
  }

  if (_publish_stats) {
    fovis_stats_t stats_msg;
    stats_msg.timestamp = update_msg.timestamp;
    stats_msg.num_matches = me->getNumMatches();
    stats_msg.num_inliers = me->getNumInliers();
    stats_msg.mean_reprojection_error = me->getMeanInlierReprojectionError();
    stats_msg.num_reprojection_failures = me->getNumReprojectionFailures();

    const OdometryFrame * tf(_odom->getTargetFrame());
    stats_msg.num_detected_keypoints = tf->getNumDetectedKeypoints();
    stats_msg.num_keypoints = tf->getNumKeypoints();
    stats_msg.fast_threshold = _odom->getFastThreshold();

    fovis_stats_t_publish(_publish_lcm, _stats_channel.c_str(), &stats_msg);
  }

  if (_publish_pose) {
    // publish current pose
    bot_core_pose_t pose_msg;
    memset(&pose_msg, 0, sizeof(pose_msg));
    pose_msg.utime = _utime_cur;
    pose_msg.pos[0] = translation[0];
    pose_msg.pos[1] = translation[1];
    pose_msg.pos[2] = translation[2];
    pose_msg.orientation[0] = rotation.w();
    pose_msg.orientation[1] = rotation.x();
    pose_msg.orientation[2] = rotation.y();
    pose_msg.orientation[3] = rotation.z();
    bot_core_pose_t_publish(_publish_lcm, _pose_channel.c_str(),
                            &pose_msg);
    //  printf("[%6.2f %6.2f %6.2f]\n", translation[0], translation[1], translation[2]);
  }

  if (_publish_frame_update) {
    //publish the frame update message as well
    bot_core_rigid_transform_t iso_msg;
    iso_msg.utime = _utime_cur;
    for (int i = 0; i < 3; i++) {
      iso_msg.trans[i] = translation[i];
    }
    iso_msg.quat[0] = rotation.w();
    iso_msg.quat[1] = rotation.x();
    iso_msg.quat[2] = rotation.y();
    iso_msg.quat[3] = rotation.z();
    // TODO why does this get published to frame update channel?
    bot_core_rigid_transform_t_publish(_publish_lcm, _frame_update_channel.c_str(),
                                       &iso_msg);
  }
}

void
StereoOdometry::image_handler(const bot_core_image_t *msg)
{
  tictoc("image_handler");
  _utime_prev = _utime_cur;
  _utime_cur = msg->utime;

  tictoc("decode_image");
  decode_image(msg);
  tictoc("decode_image");

  tictoc("processFrame");
  _depth_producer->setRightImage(_image_right_buf);
  _odom->processFrame(_image_left_buf, _depth_producer);
  tictoc("processFrame");

  publish_motion_estimation();

  tictoc("image_handler");
#ifdef USE_LCMGL
  if(_draw_lcmgl) { _visualization->draw(_odom); }
#endif
}

void
StereoOdometry::usage(const char* progname)
{
  fprintf(stderr, "Usage: %s [options]\n"
      "\n"
      "Options:\n"
      "  -p, --pose [CHAN]             Publish pose messages on specified channel.\n"
      "  -u, --update-channel [CHAN]   Publish frame updates on specified channel.\n"
      "                                  Default channel is BODY_TO_LOCAL.\n"
      "  -o, --odometry-channel [CHAN] Publish relative odometry messages on specified channel.\n"
      "                                  Default channel is STEREO_REL_ODOMETRY  \n"
      "  -s, --stats-channel [CHAN]    Publish extra statistics on specified channel.\n"
      "                                  Default channel is FOVIS_STATS.\n"
      "  -t, --tictoc-channel [CHAN]   Publish timing info on specified channel.\n"
      "                                  Default channel is FOVIS_TICTOC.\n"
      "  -i, --input-log FILE          Process a log file.\n"
      "  -w, --output-log FILE         Write output directly to a log file.\n"
      "  -c, --camera-file FILE        File with camera calibration information in bot_param format.\n"
      "                                  By default this information is read from bot_param server.\n"
      "  -b, --camera-block [STRING]   Which camera block in the cfg file to use.\n"
      "  -a, --param-file FILE         File with VO parameters in bot_param format. May be same file\n"
      "                                  as camera file.\n"
#ifdef USE_LCMGL
      "  -l, --lcmgl                   Render debugging information with LCMGL\n"
#endif
      "  -h, --help                    Shows this help text and exits\n",
      progname);
}

int
StereoOdometry::parse_command_line_options(int argc, char **argv) {
  // TODO parse options
  const char *optstring = "hp::u::s::t::o::i:w:c:b:a:l";
  int c;
  struct option long_opts[] = {
    { "help", no_argument, 0, 'h' },
    { "pose", optional_argument, 0, 'p' },
    { "update-channel", optional_argument, 0, 'u' },
    { "odometry-channel", required_argument, 0, 'o' },
    { "stats-channel", required_argument, 0, 's' },
    { "tictoc-channel", required_argument, 0, 't' },
    { "input-log", required_argument, 0, 'i' },
    { "output-log", required_argument, 0, 'w'},
    { "camera-file", required_argument, 0, 'c' },
    { "camera-block", required_argument, 0, 'b' },
    { "param-file", required_argument, 0, 'a'},
    { "lcmgl", no_argument, 0, 'l' },
    {0, 0, 0, 0}
  };

  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch(c) {
      case 'p':
        _publish_pose = true;
        if(optarg)
          _pose_channel = optarg;
        break;
      case 'u':
        _publish_frame_update = true;
        if (optarg)
          _frame_update_channel = optarg;
        break;
      case 'o':
        _publish_odometry = true;
        if (optarg)
          _odom_channel = optarg;
        break;
      case 'i':
        _input_log_fname = optarg;
        break;
      case 'w':
        _output_log_fname = optarg;
        break;
      case 'c':
        _camera_parameters_fname = optarg;
        break;
      case 'b':
        _camera_block = optarg; // mfallon
        break;
      case 'a':
        _vo_parameters_fname = optarg;
        break;
      case 's':
        _publish_stats = true;
        if (optarg)
          _stats_channel = optarg;
        break;
      case 't':
        _publish_tictoc = true;
        if (optarg)
          _tictoc_channel = optarg;
        break;
#if USE_LCMGL
      case 'l':
        _draw_lcmgl = true;
        break;
#endif
      case 'h':
      default:
        usage(argv[0]);
        return 1;
    }
  }
  return 0;
}

StereoOdometry::StereoOdometry() :
    _publish_lcm(NULL),
    _subscribe_lcm(NULL),
    _bot_param_lcm(NULL),
    _odom_channel("STEREO_REL_ODOMETRY"),
    _pose_channel("POSE"),
    _frame_update_channel("BODY_TO_LOCAL"),
    _stats_channel("FOVIS_STATS"),
    _tictoc_channel("FOVIS_TICTOC"),
    _bot_param(NULL),
    _odom(NULL),
    _depth_producer(NULL),
    _publish_pose(false),
    _publish_odometry(false),
    _publish_frame_update(false),
    _publish_stats(false),
    _publish_tictoc(false),
    _utime_cur(0),
    _utime_prev(0),
    _image_left_buf(NULL),
    _image_right_buf(NULL),
    _images_buf(NULL),
    _buf_size(0)
{
#ifdef USE_LCMGL
  _draw_lcmgl = false;
  _visualization = NULL;
#endif
}

int
StereoOdometry::init_lcm()
{
  // four cases on whether we are using logfiles for input and output.
  // TODO better error checking
  std::string in_url = std::string("file://") + _input_log_fname + "?speed=0";
  std::string out_url = std::string("file://") + _output_log_fname + "?mode=w";
  if (_input_log_fname.size() &&
      _output_log_fname.size()) {
    // both.
    // avoid getting bot_param from lcm handles on logfiles.
    _subscribe_lcm = lcm_create(in_url.c_str());
    _publish_lcm = lcm_create(out_url.c_str());
    _bot_param_lcm = lcm_create(NULL);
  } else if (_input_log_fname.size()) {
    // only input.
    _subscribe_lcm = lcm_create(in_url.c_str());
    _publish_lcm = lcm_create(NULL);
    _bot_param_lcm = _publish_lcm;
  } else if (_output_log_fname.size()) {
    // only output.
    _subscribe_lcm = lcm_create(NULL);
    _publish_lcm = lcm_create(out_url.c_str());
    _bot_param_lcm = _subscribe_lcm;
  } else {
    // neither.
    _subscribe_lcm = lcm_create(NULL);
    _publish_lcm = _subscribe_lcm;
    _bot_param_lcm = _subscribe_lcm;
  }
  if (_subscribe_lcm==NULL ||
      _publish_lcm==NULL ||
      _bot_param_lcm==NULL) {
    fprintf(stderr, "LCM initialization failed.\n");
    return 1;
  }
  return 0;
}

int
StereoOdometry::initialize(int argc, char **argv)
{
  if (parse_command_line_options(argc, argv)) { return 1; }

  VisualOdometryOptions vo_opts = StereoOdometry::getDefaultOptions();

  if (init_lcm()) { return 1; }

  if (_camera_parameters_fname.size()) {
    _bot_param = bot_param_new_from_file(_camera_parameters_fname.c_str());
    if (_bot_param == NULL) {
      fprintf(stderr, "Couldn't get bot param from file %s\n",
              _camera_parameters_fname.c_str());
      return 1;
    }
  } else {
    _bot_param = bot_param_new_from_server(_bot_param_lcm, 0);
    if (_bot_param == NULL) {
      fprintf(stderr, "Couldn't get bot param from server.\n");
      return 1;
    }
  }

  std::string calib_block = "cameras." + _camera_block; //added mfallon
  if (init_calibration(calib_block.c_str())) { return 1; }

  // TODO at some point publish camera calib on lcm
  // TODO integrate camera parameter reading and bot param reading in a sane
  // way.
  if (_vo_parameters_fname.size()) {
    BotParam * vo_bot_param = bot_param_new_from_file(_vo_parameters_fname.c_str());
    if (vo_bot_param == NULL) {
      fprintf(stderr, "Couldn't get bot param from file %s",
          _camera_parameters_fname.c_str());
      return 1;
    }

    fprintf(stderr, "VO options from file:\n");
    for (VisualOdometryOptions::iterator itr = vo_opts.begin();
         itr != vo_opts.end();
         ++itr) {
      std::string key(itr->first);
      std::replace(key.begin(), key.end(), '-', '_');
      char * new_val = bot_param_get_str_or_fail(vo_bot_param,
                                                 ("VisualOdometryOptions."+key).c_str());
      itr->second = new_val;
      free(new_val);
      fprintf(stderr, "%s : %s\n", itr->first.c_str(), itr->second.c_str());
    }
    fprintf(stderr, "\n");
  }

  // Note VisualOdometry is borrowing the rectification pointer
  StereoCalibration *calib = new StereoCalibration(_stereo_params);
  _odom = new VisualOdometry(calib->getLeftRectification(), vo_opts);
  _depth_producer = new StereoDepth(calib, vo_opts);

#ifdef USE_LCMGL
  // use bot_param_lcm because it doesn't come from logfile
  bot_lcmgl_t* lcmgl = bot_lcmgl_init(_bot_param_lcm, "stereo-odometry");
  _visualization = new Visualization(lcmgl, calib);
#endif

  // init buffers
  _buf_size = _stereo_params.left_parameters.width*_stereo_params.left_parameters.height;
  _image_left_buf = new uint8_t[_buf_size];
  _image_right_buf = new uint8_t[_buf_size];
  _images_buf = new uint8_t[2*_buf_size];

  // catch signals
  struct sigaction new_action;
  new_action.sa_sigaction = sig_handler_aux;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;
  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);

  // subscribe to image events
  //std::string image_channel(bot_param_get_camera_lcm_channel(_bot_param, "stereo")); 
  std::string image_channel(bot_param_get_camera_lcm_channel(_bot_param, _camera_block.c_str())); // added mfallon
  bot_core_image_t_subscribe(_subscribe_lcm, image_channel.c_str(),
                             StereoOdometry::image_handler_aux, this);


  return 0;
}

StereoOdometry::~StereoOdometry()
{
  delete[] _image_left_buf;
  delete[] _image_right_buf;
  delete[] _images_buf;

  delete _depth_producer;
  delete _odom;
  delete _visualization;

  if (_subscribe_lcm) {
    lcm_destroy(_subscribe_lcm);
  }
  if (_publish_lcm && (_publish_lcm != _subscribe_lcm)) {
    lcm_destroy(_publish_lcm);
  }
  // avoid double destruction. A case for smart pointers?
  if (_bot_param_lcm &&
      (_bot_param_lcm != _subscribe_lcm) &&
      (_bot_param_lcm != _publish_lcm)) {
    lcm_destroy(_bot_param_lcm);
  }
  // TODO delete bot param if it is from file
}

VisualOdometryOptions
StereoOdometry::getDefaultOptions()
{
  VisualOdometryOptions vo_opts = VisualOdometry::getDefaultOptions();

  // change to stereo 'defaults'
  vo_opts["feature-window-size"] = "9";
  vo_opts["max-pyramid-level"] = "3";
  vo_opts["min-pyramid-level"] = "0";
  vo_opts["target-pixels-per-feature"] = "250";
  vo_opts["fast-threshold"] = "10";
  vo_opts["fast-threshold-adaptive-gain"] = "0.002";
  vo_opts["use-adaptive-threshold"] = "true";
  vo_opts["use-homography-initialization"] = "true";
  vo_opts["ref-frame-change-threshold"] = "150";

  // OdometryFrame
  vo_opts["use-bucketing"] = "true";
  vo_opts["bucket-width"] = "50";
  vo_opts["bucket-height"] = "50";
  vo_opts["max-keypoints-per-bucket"] = "10";
  vo_opts["use-image-normalization"] = "true";

  // MotionEstimator
  vo_opts["inlier-max-reprojection-error"] = "2.0";
  vo_opts["clique-inlier-threshold"] = "0.1";
  vo_opts["min-features-for-estimate"] = "10";
  vo_opts["max-mean-reprojection-error"] = "8.0";
  vo_opts["use-subpixel-refinement"] = "true";
  vo_opts["feature-search-window"] = "25";
  vo_opts["update-target-features-with-refined"] = "false";

  // StereoDepth
  vo_opts["stereo-require-mutual-match"] = "true";
  vo_opts["stereo-max-dist-epipolar-line"] = "2.0";
  vo_opts["stereo-max-refinement-displacement"] = "2.0";
  vo_opts["stereo-max-disparity"] = "128";

  return vo_opts;
}

}
