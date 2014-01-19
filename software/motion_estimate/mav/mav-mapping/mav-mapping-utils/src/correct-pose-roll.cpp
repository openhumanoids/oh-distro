#include <stdio.h>
#include <lcmtypes/isam_slam.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/mav/ins_t.hpp>
#include <lcm_utils/lcm_utils.hpp>
#include <ConciseArgs>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <eigen_utils/eigen_utils.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <laser_utils/laser_util.h>
#include <lcmgl_utils/LcmglStore.hpp>

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

/**
 * fits the laser to the floor by doing ransac on the left and right half of the
 * laser scan, considering points below a threshold. the final fit is performed by
 * using the ransac-selected points from left and right with each half's points
 * weighted by the inverse of their fit's variance
 */
class LaserFloorFit {
public:
  Laser_projector * projector;
  laser_projected_scan * proj_scan;
  lcmgl_utils::LcmglStore lcmgls;
  bool lcmgl_vis;

  double ground_thresh;
  int min_num_points;
  double include_threshold;
  int num_iterations;
  int vis_usleep;
  int beam_skip;
  double spatial_decimation_min;
  double spatial_decimation_max;

  LaserFloorFit(lcm_t * lcm, BotParam * param, BotFrames * frames, std::string laser_name)
  {
    projector = laser_projector_new(param, frames, laser_name.c_str(), 1);
    proj_scan = NULL;

    lcmgls = lcmgl_utils::LcmglStore(lcm, "LaserFloorFit");

    ground_thresh = bot_param_get_double_or_fail(param, "pose_correct.laser_ransac.ground_thresh");
    min_num_points = bot_param_get_int_or_fail(param, "pose_correct.laser_ransac.min_num_points");
    include_threshold = bot_param_get_double_or_fail(param, "pose_correct.laser_ransac.include_threshold");
    num_iterations = bot_param_get_int_or_fail(param, "pose_correct.laser_ransac.num_iterations");
    lcmgl_vis = bot_param_get_boolean_or_fail(param, "pose_correct.laser_ransac.lcmgl_vis");
    vis_usleep = bot_param_get_int_or_fail(param, "pose_correct.laser_ransac.vis_usleep");

    beam_skip = bot_param_get_int_or_fail(param, "pose_correct.laser_ransac.beam_skip");
    spatial_decimation_min = bot_param_get_double_or_fail(param, "pose_correct.laser_ransac.spatial_decimation_min");
    spatial_decimation_max = bot_param_get_double_or_fail(param, "pose_correct.laser_ransac.spatial_decimation_max");
  }

  void fitScan(const bot_core_planar_lidar_t * msg, double * height, double * roll)
  {

    proj_scan = laser_create_projected_scan_from_planar_lidar(projector, msg, "body");
    int beam_skip = 1;
    double spatial_decimation_min = .07;
    double spatial_decimation_max = .07;

    laser_decimate_projected_scan(proj_scan, beam_skip, spatial_decimation_min, spatial_decimation_max);

    MatrixXd left_data(2, proj_scan->numValidPoints);
    MatrixXd right_data(2, proj_scan->numValidPoints);

    if (lcmgl_vis) {
      bot_lcmgl_color3fv(lcmgls["laserPoints"], bot_color_util_blue);
      bot_lcmgl_point_size(lcmgls["laserPoints"], 1);
      bot_lcmgl_begin(lcmgls["laserPoints"], LCMGL_POINTS);

      bot_lcmgl_color3fv(lcmgls["consideredPoints"], bot_color_util_green);
      bot_lcmgl_point_size(lcmgls["consideredPoints"], 3);
      bot_lcmgl_begin(lcmgls["consideredPoints"], LCMGL_POINTS);
    }

    int num_considered_left = 0;
    int num_considered_right = 0;
    for (int i = 0; i < this->proj_scan->npoints; i++) {
      if (this->proj_scan->point_status[i] != laser_surround)
        continue;

      if (proj_scan->points[i].z < ground_thresh) {
        bot_lcmgl_vertex2d(lcmgls["consideredPoints"], proj_scan->points[i].y, proj_scan->points[i].z);
        if (proj_scan->points[i].y > 0) {
          left_data.col(num_considered_left) = Vector2d(proj_scan->points[i].y, proj_scan->points[i].z);
          num_considered_left++;
        }
        else {
          right_data.col(num_considered_right) = Vector2d(proj_scan->points[i].y, proj_scan->points[i].z);
          num_considered_right++;
        }
      }

      if (lcmgl_vis) {
        bot_lcmgl_vertex2d(lcmgls["laserPoints"], proj_scan->points[i].y, proj_scan->points[i].z);
      }
    }

    if (lcmgl_vis) {
      bot_lcmgl_end(lcmgls["laserPoints"]);
      bot_lcmgl_switch_buffer(lcmgls["laserPoints"]);

      bot_lcmgl_end(lcmgls["consideredPoints"]);
      bot_lcmgl_switch_buffer(lcmgls["consideredPoints"]);
    }

    left_data = left_data.leftCols(num_considered_left);
    right_data = right_data.leftCols(num_considered_right);

    VectorXi consensus_set_left, consensus_set_right;
    VectorXd result_left, result_right;

    double left_stddev = fitHyperPlaneRANSAC(left_data, num_iterations, include_threshold, min_num_points,
        result_left,
        consensus_set_left);
    double right_stddev = fitHyperPlaneRANSAC(right_data, num_iterations, include_threshold, min_num_points,
        result_right,
        consensus_set_right);

    MatrixXd total_data(2, left_data.cols() + right_data.cols());
    VectorXd total_weights(left_data.cols() + right_data.cols());

    total_data.leftCols(left_data.cols()) = left_data;
    total_data.rightCols(right_data.cols()) = right_data;

    total_weights.head(left_data.cols()) = consensus_set_left.cast<double>() / bot_sq(left_stddev);
    total_weights.tail(right_data.cols()) = consensus_set_right.cast<double>() / bot_sq(right_stddev);

    VectorXd result;
    fitHyperplaneLeastSquares(total_data, total_weights, result);

    laser_destroy_projected_scan(proj_scan);

    *height = result.norm();
    *roll = atan2(-result(0), -result(1));

    if (lcmgl_vis) {
      bot_lcmgl_line_width(lcmgls["fitVec"], 3);
      bot_lcmgl_color3fv(lcmgls["fitVec"], bot_color_util_magenta);
      bot_lcmgl_begin(lcmgls["fitVec"], LCMGL_LINES);
      bot_lcmgl_vertex2d(lcmgls["fitVec"], 0, 0);
      bot_lcmgl_vertex2d(lcmgls["fitVec"], result_left(0), result_left(1));
      bot_lcmgl_end(lcmgls["fitVec"]);

      bot_lcmgl_line_width(lcmgls["fitVec"], 3);
      bot_lcmgl_color3fv(lcmgls["fitVec"], bot_color_util_cyan);
      bot_lcmgl_begin(lcmgls["fitVec"], LCMGL_LINES);
      bot_lcmgl_vertex2d(lcmgls["fitVec"], 0, 0);
      bot_lcmgl_vertex2d(lcmgls["fitVec"], result_right(0), result_right(1));
      bot_lcmgl_end(lcmgls["fitVec"]);

      bot_lcmgl_line_width(lcmgls["fitVec"], 5);
      bot_lcmgl_color3fv(lcmgls["fitVec"], bot_color_util_black);
      bot_lcmgl_begin(lcmgls["fitVec"], LCMGL_LINES);
      bot_lcmgl_vertex2d(lcmgls["fitVec"], 0, 0);
      bot_lcmgl_vertex2d(lcmgls["fitVec"], result(0), result(1));
      bot_lcmgl_end(lcmgls["fitVec"]);

      bot_lcmgl_switch_buffer(lcmgls["fitVec"]);

      bot_lcmgl_color3fv(lcmgls["includedPoints"], bot_color_util_red);
      bot_lcmgl_point_size(lcmgls["includedPoints"], 5);
      bot_lcmgl_begin(lcmgls["includedPoints"], LCMGL_POINTS);
      for (int ii = 0; ii < left_data.cols(); ii++) {
        if (consensus_set_left(ii))
          bot_lcmgl_vertex2d(lcmgls["includedPoints"], left_data(0, ii), left_data(1, ii));
      }

      for (int ii = 0; ii < right_data.cols(); ii++) {
        if (consensus_set_right(ii))
          bot_lcmgl_vertex2d(lcmgls["includedPoints"], right_data(0, ii), right_data(1, ii));
      }
      bot_lcmgl_end(lcmgls["includedPoints"]);
      bot_lcmgl_switch_buffer(lcmgls["includedPoints"]);

      usleep(vis_usleep);

    }
  }

  ~LaserFloorFit()
  {
    laser_projector_destroy(projector);
  }
};

int main(int argc, char** argv)
{
  string logfileName;
  string outFileName;

  string insChannelName = "MICROSTRAIN_INS";
  string insFrameName = "microstrain";
  string paramFileName = "";
  string poseChannelName = "STATE_ESTIMATOR_POSE";

  char mode = 'p';

  ConciseArgs opt =
      ConciseArgs(
          argc,
          argv,
          "logFname outFname",
          "corrects the roll of the tripod setup by default with the vertical laser fitting to a flat floor using ransac, switch to ins or other pose with optional args");
  opt.add(paramFileName, "P", "param",
      "param file (otherwise tries to get over lcm)\n");
  opt.add(insChannelName, "i", "ins",
      "ins channel name to use ins to get pitch and roll, assumes param is available with -frame");
  opt.add(insFrameName, "f", "frame",
      "ins frame name in config, used in conjuction with -ins to get ins orientation\n");
  opt.add(poseChannelName, "p", "pose",
      "pose channel name to get pitch and roll, assumes param is available with -frame");
  opt.add(mode, "m", "mode", "'p' uses orientation from pose set rpy and sets height to default from config\n"
      "'h' just sets the height to default\n"
      "'i' uses ins orientation to set rpy\n"
      "'r' uses ransac to set rpy\n");

  opt.parse(logfileName, outFileName);


  printf("correcting poses in log %s\n", logfileName.c_str());
  printf("and writing it to %s\n", outFileName.c_str());

  lcm_t * lcm_live = lcm_create(NULL);
  BotParam * param;
  if (opt.wasParsed("param")) {
    param = bot_param_new_from_file(paramFileName.c_str());
  }
  else {
    param = bot_param_get_global(lcm_live, false);
  }
  BotFrames * frames = bot_frames_new(lcm_live, param);
  LaserFloorFit * floor_fitter;
  double height = bot_param_get_double_or_fail(param, "pose_correct.default_height");

  Quaterniond body_to_ins_quat = Quaterniond::Identity();

  switch (mode) {
  case 'p':
    fprintf(stderr, "using pose on channel %s to correct rpy and setting height to %f\n", poseChannelName.c_str(),
        height);
    if (poseChannelName == "POSE") {
      fprintf(stderr, "can't correct using channel POSE\n");
      exit(1);
    }
    break;
  case 'h':
    fprintf(stderr, "setting height to default from config %f\n", height);
  case 'i':
    fprintf(stderr, "using imu to correct pitch and roll on channel %s with frame name %s\n", insChannelName.c_str(),
        insFrameName.c_str());

    BotTrans body_to_ins_trans;
    bot_frames_get_trans(frames, "body", insFrameName.c_str(), &body_to_ins_trans);

    botDoubleToQuaternion(body_to_ins_quat, body_to_ins_trans.rot_quat);
    break;
  case 'r':
    fprintf(stderr, "using ransac on flat floor to correct rolls\n");
    floor_fitter = new LaserFloorFit(lcm_live, param, frames, "laser_vert");
    break;
  default:
    opt.usage(true);
  }

  Quaterniond body_to_level_quat = Quaterniond::Identity();

  // Open the log file.
  lcm::LogFile log(logfileName, "r");
  if (!log.good()) {
    perror("LogFile");
    std::cerr << "couldn't open log file: " << logfileName << std::endl;
    return 1;
  }

  lcm::LogFile out_log(outFileName, "w");

  bool set_correction = false;
  if (mode == 'h')
    set_correction = true;

  int num_poses = 0;

  while (1) {
// Read a log event.
    const lcm::LogEvent *original_event = log.readNextEvent();
    if (!original_event)
      break;

    lcm::LogEvent event = *original_event; //copy event structure so that event num can be written to
    if (event.channel == "POSE") { //
      if (!set_correction) {
        fprintf(stderr, "got pose before correction, discarding\n");
        continue;
      }

      num_poses++;
      if (num_poses % 500 == 0) {
        fprintf(stderr, "corrected %d poses\n", num_poses);
      }

      bot_core::pose_t pmsg;
      pmsg.decode(event.data, 0, event.datalen);

      Quaterniond yaw_slam_quat;
      botDoubleToQuaternion(yaw_slam_quat, pmsg.orientation);
      yaw_slam_quat = yaw_slam_quat * body_to_level_quat;
      quaternionToBotDouble(pmsg.orientation, yaw_slam_quat);

      pmsg.pos[2] = height;

      pmsg.encode(event.data, 0, event.datalen);
    }
    else if (mode == 'i' && event.channel == insChannelName) {
      mav::ins_t ins_msg;
      ins_msg.decode(event.data, 0, event.datalen);
      Quaterniond ins_to_local_quat;
      eigen_utils::botDoubleToQuaternion(ins_to_local_quat, ins_msg.quat);
      Quaterniond insbody_to_local_quat = ins_to_local_quat * body_to_ins_quat;
      Vector3d eulers = getEulerAngles(insbody_to_local_quat);
      eulers(2) = 0; //zero out the yaw so we just use pitch and roll
      body_to_level_quat = setQuatEulerAngles(eulers);

      set_correction = true;

//      Vector3d ins_eulers_deg = bot_to_degrees(getEulerAngles(ins_to_local_quat));
//      fprintf(stderr, "raw ins rpy %f,%f,%f ... trans rp %f, %f\n",
//          ins_eulers_deg(0), ins_eulers_deg(1), ins_eulers_deg(2),
//          bot_to_degrees(eulers(0)), bot_to_degrees(eulers(1)));

    }
    else if (mode == 'p' && event.channel == poseChannelName) {
      bot_core::pose_t pose_msg;
      pose_msg.decode(event.data, 0, event.datalen);
      Quaterniond pose_to_local_quat;
      eigen_utils::botDoubleToQuaternion(pose_to_local_quat, pose_msg.orientation);
      Vector3d eulers = getEulerAngles(pose_to_local_quat);
      eulers(2) = 0; //zero out the yaw so we just use pitch and roll
      body_to_level_quat = setQuatEulerAngles(eulers);

      set_correction = true;

      //      Vector3d ins_eulers_deg = bot_to_degrees(getEulerAngles(ins_to_local_quat));
      //      fprintf(stderr, "raw ins rpy %f,%f,%f ... trans rp %f, %f\n",
      //          ins_eulers_deg(0), ins_eulers_deg(1), ins_eulers_deg(2),
      //          bot_to_degrees(eulers(0)), bot_to_degrees(eulers(1)));

    }
    else if (mode == 'r' && event.channel == "LASER_VERT") {
      bot_core_planar_lidar_t lmsg_c;
      bot_core_planar_lidar_t_decode(event.data, 0, event.datalen, &lmsg_c);
      double roll;
      floor_fitter->fitScan(&lmsg_c, &height, &roll);

      Vector3d eulers(roll, 0, 0);
      body_to_level_quat = setQuatEulerAngles(eulers);
      bot_core_planar_lidar_t_decode_cleanup(&lmsg_c);

      set_correction = true;
    }

//    out_log.publish(event.channel, event.data, event.datalen);

    out_log.writeEvent(&event);

  }

  bot_frames_destroy(frames);
  bot_param_destroy(param);
  lcm_destroy(lcm_live);

  return 0;
}
