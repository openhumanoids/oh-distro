/*
 * bot_loop_closer.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: abachrac
 */
//local stuff
#include "IsamSlam.hpp"

#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <opencv/highgui.h>
//#ifndef _GNU_SOURCE
//#define _GNU_SOURCE
//#endif
#include <getopt.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>
#include <scanmatch/ScanMatcher.hpp>
#include <lcmtypes/sm_rigid_transform_2d_t.h>

using namespace std;
using namespace scanmatch;

//add a new node every this many meters
#define LINEAR_DIST_TO_ADD .5
//and or this many radians
#define ANGULAR_DIST_TO_ADD 0.5

typedef struct {
  char * input_log_fname;
  lcm_t * lcm_recv;
  lcm_t * lcm_pub;
  IsamSlam * isamslam;
  ScanMatcher * sm;
  ScanMatcher * sm_incremental;

  int optimize;
  int scanmatchBeforAdd;
  double odometryConfidence;
  int publish_pose;
  int do_drawing;
  char * chan;
  int beam_skip; //downsample ranges by only taking 1 out of every beam_skip points
  double spatialDecimationThresh; //don't discard a point if its range is more than this many std devs from the mean range (end of hallway)
  double maxRange; //discard beams with reading further than this value
  double maxUsableRange; //only draw map out to this far for MaxRanges...
  float validBeamAngles[2]; //valid part of the field of view of the laser in radians, 0 is the center beam
  sm_rigid_transform_2d_t * prev_odom; //odometry computed outside module
  sm_rigid_transform_2d_t * prev_sm_odom; //internally scan matched odometry
  int verbose;

  bool publishTraj;
  bool publishCompleteTraj;

  char * initialization_log_name;

} app_t;

Scan *
get_maxrange_scan(float * ranges, int numPoints, double thetaStart, double thetaStep, double maxRange,
    double maxUsableRange, double validRangeStart, double validRangeEnd)
{
  Scan * scan = new Scan();
  smPoint * points = (smPoint *) malloc(numPoints * sizeof(smPoint));
  scan->points = points;
  int numMaxRanges = 0;
  double theta = thetaStart;
  Contour * contour = new Contour();
  scan->contours.push_back(contour);
  for (int i = 0; i < numPoints; i++) {
    double r = ranges[i];
    if (r < .1) {
      r = maxRange;
    }
    if (r >= maxRange && theta > validRangeStart && theta < validRangeEnd) {
      r = maxUsableRange;
      //project to body centered coordinates
      points[numMaxRanges].x = r * cos(theta);
      points[numMaxRanges].y = r * sin(theta);
      contour->points.push_back(points[numMaxRanges]);
      numMaxRanges++;

    }
    else if (contour->points.size() > 0) {
      //end that contour
      if (contour->points.size() > 2) {
        //clear out middle points
        contour->points[1] = contour->points.back();
        contour->points.resize(2);
      }
      //      printf("contour is (%f,%f)-(%f,%f)\n",contour->points[0].x,contour->points[0].y,contour->points[1].x,contour->points[1].y);
      contour = new Contour();
      scan->contours.push_back(contour);
    }
    else if (contour->points.size() == 1) {
      //discard single maxrange beams...
      contour->points.clear();
    }
    theta += thetaStep;
  }

  if (contour->points.size() < 2) {
    scan->contours.pop_back();
    delete contour;
  }

  points = (smPoint*) realloc(points, numMaxRanges * sizeof(smPoint));
  scan->numPoints = numMaxRanges;
  scan->ppoints = (smPoint *) malloc(numMaxRanges * sizeof(smPoint));
  memcpy(scan->ppoints, points, numMaxRanges * sizeof(smPoint));
  return scan;
}

////////////////////////////////////////////////////////////////////
//where all the work is done
////////////////////////////////////////////////////////////////////
static void aligned_laser_handler(const bot_core_planar_lidar_t * laser_msg, const sm_rigid_transform_2d_t * odom,
    int64_t utime, double height, double rp[2], app_t * app)
{
  //compute the distance between this scan and the last one that got added.
  double dist[2];
  sm_vector_sub_2d(odom->pos, app->prev_odom->pos, dist);
  double ld = sm_norm(SMPOINT(dist));
  double ad = sm_angle_subtract(odom->theta, app->prev_odom->theta);
  bool addScan = app->isamslam->trajectory.empty();
  if (fabs(ld) > LINEAR_DIST_TO_ADD) {
    //    fprintf(stderr, "Scan is %f meters from last add, adding scan\n", ld);
    addScan = true;
  }
  if (fabs(ad) > ANGULAR_DIST_TO_ADD) {
    //    fprintf(stderr, "Scan is %f degrees from last add, adding scan\n", ad* 180.0/M_PI);
    addScan = true;
  }
  if (!addScan) {
    //    fprintf(stderr, "Scan is %f meters and %f degrees from last add, NOT adding scan\n", ld,ad* 180.0/M_PI);
    if (app->publish_pose || app->publishCompleteTraj) {
      Pose2d curr_pose(odom->pos[0], odom->pos[1], odom->theta);
      Pose2d prev_pose(app->prev_odom->pos[0], app->prev_odom->pos[1], app->prev_odom->theta);
      Pose2d prev_curr_tranf = curr_pose.ominus(prev_pose);
      Pose2d curr_slam_pose = app->isamslam->getCurrentPose();
      Pose2d prop_pose = curr_slam_pose.oplus(prev_curr_tranf);
      bot_core_pose_t pose;
      memset(&pose, 0, sizeof(pose));
      pose.pos[0] = prop_pose.x();
      pose.pos[1] = prop_pose.y();
      double rpy[3] = { 0, 0, prop_pose.t() };
      bot_roll_pitch_yaw_to_quat(rpy, pose.orientation);
      pose.utime = laser_msg->utime;
      if (app->publish_pose)
        bot_core_pose_t_publish(app->lcm_pub, "POSE", &pose);
      if (app->publishCompleteTraj)
        app->isamslam->addIntermediateNode(prev_curr_tranf, laser_msg->utime, height, rp);
    }
    return;
  }
  sm_tictoc("addScanToGraph");

  ////////////////////////////////////////////////////////////////////
  //we want to process and add this scan!
  ////////////////////////////////////////////////////////////////////
  fprintf(stderr, "a");
  //Project ranges into points, and decimate points so we don't have too many
  smPoint * points = (smPoint *) calloc(laser_msg->nranges, sizeof(smPoint));
  int numValidPoints = sm_projectRangesAndDecimate(app->beam_skip, app->spatialDecimationThresh, laser_msg->ranges,
      laser_msg->nranges, laser_msg->rad0, laser_msg->radstep, points, app->maxRange, app->validBeamAngles[0],
      app->validBeamAngles[1]);
  if (numValidPoints < 30) {
    fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n", numValidPoints);
    free(points);
    return;
  }
  else {
    points = (smPoint *) realloc(points, numValidPoints * sizeof(smPoint));  //crop memory down to size
  }

  ScanTransform T;
  //just use the zero transform... it'll get updated the first time its needed
  memset(&T, 0, sizeof(T));
  Scan * scan = new Scan(numValidPoints, points, T, SM_HOKUYO_UTM, utime, true);
  Scan * maxRangeScan = get_maxrange_scan(laser_msg->ranges, laser_msg->nranges, laser_msg->rad0, laser_msg->radstep,
      app->maxRange, app->maxUsableRange, app->validBeamAngles[0], app->validBeamAngles[1]);

  //get the odometry delta
  Pose2d curr_pose(odom->pos[0], odom->pos[1], odom->theta);
  Pose2d prev_pose(app->prev_odom->pos[0], app->prev_odom->pos[1], app->prev_odom->theta);
  Pose2d prev_curr_tranf = curr_pose.ominus(prev_pose);

  if (app->prev_odom != NULL
  )
    sm_rigid_transform_2d_t_destroy(app->prev_odom);
  app->prev_odom = sm_rigid_transform_2d_t_copy(odom);

  //rotate the cov to body frame
  double Rcov[9];
  sm_rotateCov2D(odom->cov, -odom->theta, Rcov);
  Matrix cov(3, 3, Rcov);
  if (!app->scanmatchBeforAdd) {
    sm_tictoc("addNodeToSlam");
    app->isamslam->addNodeToSlam(prev_curr_tranf, cov, scan, maxRangeScan, utime, height, rp);
    sm_tictoc("addNodeToSlam");
  }
  else {
    sm_tictoc("IncrementalSMRefineMent");
    //do incremental scan matching to refine the odometry estimate
    Pose2d prev_sm_pose(app->prev_sm_odom->pos[0], app->prev_sm_odom->pos[1], app->prev_sm_odom->theta);
    Pose2d curr_sm_pose = prev_sm_pose.oplus(prev_curr_tranf);
    ScanTransform sm_prior;
    memset(&sm_prior, 0, sizeof(sm_prior));
    sm_prior.x = curr_sm_pose.x();
    sm_prior.y = curr_sm_pose.y();
    sm_prior.theta = curr_sm_pose.t();
    sm_prior.score = app->odometryConfidence; //wide prior
    ScanTransform r = app->sm->matchSuccessive(points, numValidPoints, SM_HOKUYO_UTM, sm_get_utime(), false, &sm_prior);
    curr_sm_pose.set(r.x, r.y, r.theta);
    sm_rigid_transform_2d_t sm_odom;
    memset(&sm_odom, 0, sizeof(sm_odom));
    sm_odom.utime = laser_msg->utime;
    sm_odom.pos[0] = r.x;
    sm_odom.pos[1] = r.y;
    sm_odom.theta = r.theta;
    memcpy(sm_odom.cov, r.sigma, 9 * sizeof(double));
    if (app->prev_sm_odom != NULL
    )
      sm_rigid_transform_2d_t_destroy(app->prev_sm_odom);
    app->prev_sm_odom = sm_rigid_transform_2d_t_copy(&sm_odom);

    Pose2d prev_curr_sm_tranf = curr_sm_pose.ominus(prev_sm_pose);
    double Rcov_sm[9];
    sm_rotateCov2D(r.sigma, -r.theta, Rcov_sm);
    Matrix cov_sm(3, 3, Rcov_sm);
    cov_sm = 1000 * cov_sm;

    double cov_hardcode[9] = { .05, 0, 0, 0, .05, 0, 0, 0, .01 };
    Matrix cov_hc(3, 3, cov_hardcode);
    sm_tictoc("IncrementalSMRefineMent");
    app->isamslam->addNodeToSlam(prev_curr_sm_tranf, cov_hc, scan, maxRangeScan, utime, height, rp);
  }
  sm_tictoc("addScanToGraph");

  if (app->optimize) {
    sm_tictoc("SlamIncrementalOptim");
    app->isamslam->slam->update();
    sm_tictoc("SlamIncrementalOptim");
  }

  sm_tictoc("publishDraw");

  //publish the SLAM_POS
  sm_tictoc("publishSlamPos");
  app->isamslam->publishSlamPos();
  sm_tictoc("publishSlamPos");

  if (app->publishTraj) {
    //publish the SLAM_Trajectory
    sm_tictoc("publishSlamPos");
    app->isamslam->publishSlamTrajectory();
    sm_tictoc("publishSlamPos");
  }

  if (app->publishCompleteTraj) {
    //publish the complete trajectory
    sm_tictoc("publishCompleteTrajector");
    app->isamslam->publishCompleteTrajectory();
    sm_tictoc("publishCompleteTrajector");
  }

  //render the gridmap
  sm_tictoc("renderGridmap");
  app->isamslam->renderGridmap();
  sm_tictoc("renderGridmap");

  sm_tictoc("publishGridMap");
  app->isamslam->publishGridMap();
  sm_tictoc("publishGridMap");

  //do drawing :-)
  sm_tictoc("drawGraph");
  app->isamslam->drawGraph();
  sm_tictoc("drawGraph");

  //  sm_tictoc("drawMap");
  //  app->isamslam->drawMap();
  //  sm_tictoc("drawMap");

  //  sm_tictoc("draw3DPointCloud");
  //  app->isamslam->draw3DPointCloud();
  //  sm_tictoc("draw3DPointCloud");

  sm_tictoc("publishDraw");

  if (app->publish_pose) {
    Pose2d slam_pose = app->isamslam->getCurrentPose();
    bot_core_pose_t pose;
    memset(&pose, 0, sizeof(pose));
    pose.pos[0] = slam_pose.x();
    pose.pos[1] = slam_pose.y();
    double rpy[3] = { 0, 0, slam_pose.t() };
    bot_roll_pitch_yaw_to_quat(rpy, pose.orientation);
    pose.utime = laser_msg->utime;
    bot_core_pose_t_publish(app->lcm_pub, "POSE", &pose);
  }

}

//static void robot_laser_handler(const lcm_recv_buf_t *rbuf , const char * channel , const carmen3d_robot_laser_t * msg,
//    void * user  )
//{
//  app_t * app = (app_t *) user;
//
//  const carmen3d_planar_lidar_t * laser_msg = &msg->laser;
//  //convert odometry info to bot_core_rigid_transform_2d style... which is what it should be...
//  sm_rigid_transform_2d_t odom;
//  memset(&odom, 0, sizeof(odom));
//  memcpy(odom.pos, msg->pose.pos, 2 * sizeof(double));
//  double rpy[3];
//  bot_quat_to_roll_pitch_yaw(msg->pose.orientation, rpy);
//  odom.theta = rpy[2];
//  double cov[9] = { 0 };
//  cov[0] = msg->cov.x;
//  cov[1] = msg->cov.xy;
//  cov[3] = msg->cov.xy;
//  cov[4] = msg->cov.y;
//  cov[8] = msg->cov.yaw;
//  //cov was published as body frame already... rotate to global for now
//  double Rcov[9];
//  sm_rotateCov2D(cov, odom.theta, odom.cov);
//
//  static int64_t utime_prev = msg->utime;
//  //this is a bad idea if the wifi drops out :-/
//  //let's let scan matcher handle this...
//  //TODO: might make sense to do an extra big scan match job or something though...
//
//  if (fabs(msg->utime - utime_prev) > 30e6 && bot_vector_magnitude_2d(odom.pos) < .1) {
//    fprintf(stderr, "BIG gap in robot_laser timestamps and quad is back at 0, doing a hotstart! %jd %jd\n", msg->utime,
//        utime_prev);
//    if (app->prev_odom != NULL)
//      sm_rigid_transform_2d_t_destroy(app->prev_odom);
//    app->prev_odom = sm_rigid_transform_2d_t_copy(&odom);
//  }
//  utime_prev = msg->utime;
//
//  sm_tictoc("aligned_laser_handler");
//  aligned_laser_handler((bot_core_planar_lidar_t*) laser_msg, &odom, msg->utime, msg->pose.pos[2], rpy, app);
//  sm_tictoc("aligned_laser_handler");
//}

////////////////////////////////////////////////////////////////////
//where all the work is done
////////////////////////////////////////////////////////////////////
static void laser_handler(const lcm_recv_buf_t *rbuf,
    const char * channel, const bot_core_planar_lidar_t * msg,
    void * user)
{
  app_t * app = (app_t *) user;
  sm_tictoc("laser_handler");
  ////////////////////////////////////////////////////////////////////
  //Project ranges into points, and decimate points so we don't have too many
  ////////////////////////////////////////////////////////////////////
  smPoint * points = (smPoint *) calloc(msg->nranges, sizeof(smPoint));
  int numValidPoints = sm_projectRangesAndDecimate(app->beam_skip, app->spatialDecimationThresh, msg->ranges,
      msg->nranges, msg->rad0, msg->radstep, points, app->maxRange, app->validBeamAngles[0], app->validBeamAngles[1]);
  if (numValidPoints < 30) {
    fprintf(stderr, "WARNING! NOT ENOUGH VALID POINTS! numValid=%d\n", numValidPoints);
    return;
  }

  ////////////////////////////////////////////////////////////////////
  //do the matching
  ////////////////////////////////////////////////////////////////////
  static int64_t prev_msg_utime = -1;
  ScanTransform r;
  if (prev_msg_utime > 0 && msg->utime - prev_msg_utime > 10e6) {
    fprintf(stderr, "\n\n%jdsec jump in laser message timestamps, matching with a big window\n\n", (msg->utime
        - prev_msg_utime) / 1000000);
    r = app->sm_incremental->gridMatch(points, numValidPoints, &app->sm_incremental->currentPose, 5, 5, M_PI, NULL,
        NULL, NULL);
    app->sm_incremental->drawGUI(points, numValidPoints, r, NULL, "HotStart");
    app->sm_incremental->prevPose = r;
    app->sm_incremental->currentPose = r;
  }
  else {
    r = app->sm_incremental->matchSuccessive(points, numValidPoints, SM_HOKUYO_UTM, msg->utime, NULL); //don't have a better estimate than prev, so just set prior to NULL
  }
  prev_msg_utime = msg->utime;

  sm_rigid_transform_2d_t odom;
  memset(&odom, 0, sizeof(odom));
  odom.utime = msg->utime;
  odom.pos[0] = r.x;
  odom.pos[1] = r.y;
  odom.theta = r.theta;
  memcpy(odom.cov, r.sigma, 9 * sizeof(double));

  //  app->sm_incremental->drawGUI(points, numValidPoints, r, NULL,"ScanMatcher");

  free(points);
  sm_tictoc("laser_handler");

  double rp[2] = { 0, 0 }; //call the slam/loop closing handler
  aligned_laser_handler(msg, &odom, msg->utime, 0, rp, app);

}

//static void optim_handler(const lcm_recv_buf_t *rbuf , const char * channel , const carmen3d_log_annotate_msg_t * msg,
//    void * user  )
//{
//  app_t * app = (app_t *) user;
//  switch (msg->enumerate) {
//  case 0:
//    app->optimize = 0;
//    break;
//  case 1:
//    app->optimize = 1;
//    break;
//  case 2:
//    fprintf(stderr, "batch_optimization_step\n");
//    app->isamslam->slam->batch_optimization();
//    break;
//  default:
//    fprintf(stderr, "full_optimization\n");
//    app->isamslam->slam->update();
//    break;
//  }
//  //do drawing :-)
//  app->isamslam->publishSlamPos();
//  app->isamslam->publishSlamTrajectory();
//  app->isamslam->renderGridmap();
//  app->isamslam->publishGridMap();
//  app->isamslam->draw3DPointCloud();
//  app->isamslam->drawGraph();
//  app->isamslam->drawMap();
//  double trajlength = app->isamslam->computeTrajectoryLength();
//  fprintf(stderr, "trajectory is %f meters long\n", trajlength);
//}

void app_destroy(app_t *app)
{
  app->isamslam->publishSlamPos();
  app->isamslam->publishSlamTrajectory();
  if (app->publishCompleteTraj) {
    app->isamslam->publishCompleteTrajectory("isam_slam_complete_traj.lcmlog");
  }
  app->isamslam->renderGridmap();
  app->isamslam->publishGridMap();
  app->isamslam->draw3DPointCloud();
  app->isamslam->drawGraph();
  app->isamslam->drawMap();
  double trajlength = app->isamslam->computeTrajectoryLength();
  fprintf(stderr, "Exiting after a trajectory is %f meters long\n", trajlength);

  //TODO: there is a double free mess cuz scan matcher free's scans that are held onto elsewhere :-/
  //LEAK everything for now!!!

  //  if (app) {
  //    if (app->chan)
  //      free(app->chan);
  //
  //    lcm_destroy(app->lcm);
  //    if (app->prev_odom)
  //      sm_rigid_transform_2d_t_destroy(app->prev_odom);
  //
  //    if (app->isamslam)
  //      delete app->isamslam;
  //    fprintf(stderr, "deleted isamSlam object\n");
  //    if (app->sm)
  //      delete app->sm;
  //    if (app->sm_incremental)
  //      delete app->sm_incremental;
  //
  //    free(app);
  //  }

  // dump timing stats
  sm_tictoc(NULL);

}

static void usage(const char *name)
{
  fprintf(
      stderr,
      "usage: %s [options]\n"
          "\n"
          "  -h, --help                      Shows this help text and exits\n"
          "  -a,  --aligned                  Subscribe to aligned_laser (ROBOT_LASER) msgs. Incremental scan matching will no be performed.\n"
          "  -c, --chan <LCM CHANNEL>        Input lcm channel default:\"LASER\" or \"ROBOT_LASER\"\n"
          "  -s, --scanmatch                 Run scan matcher between nodes that get added to map. The incremental scan matcher/odometry is used as a prior\n"
          "  -d, --draw                      Show window with scan matches for loop closures \n"
          "  -f, --from_log <log_file>       Run directly on logged data\n"
          "  -p, --publish_pose              publish POSE messages\n"
          "  -t, --trajectory                publish the SLAM trajectory after every node is added to the map (~1m)\n"
          "  -T, --complete_trajectory       publish the complete optimized trajectory after every node is added to the map (~1m).\n"
          "                                      Poses in between the SLAM nodes are interpolated using the odometry.\n"
          "  -v, --verbose                   Be verbose\n"
          "  -m, --mode  \"HOKUYO_UTM\"|\"SICK\" configures low-level options. default is HOKUYO_UTM\n"
          "\n"
          "Low-level laser options:\n"
          "  -M, --mask <min,max>            Mask min max angles in (radians)\n"
          "  -B, --beamskip <n>              Skip every n beams \n"
          "  -D, --decimation <value>        Spatial decimation threshold (meters?)\n"
          "  -R, --range <range>             Maximum range (meters)\n", name);
}

sig_atomic_t still_groovy = 1;

static void sig_action(int signal, siginfo_t *s, void *user)
{
  still_groovy = 0;
}

int main(int argc, char *argv[])
{
  setlinebuf (stdout);

  app_t *app = (app_t *) calloc(1, sizeof(app_t));
  app->prev_odom = (sm_rigid_transform_2d_t *) calloc(1, sizeof(sm_rigid_transform_2d_t)); //set initial prev_odom to zero...
  app->prev_sm_odom = (sm_rigid_transform_2d_t *) calloc(1, sizeof(sm_rigid_transform_2d_t)); //set initial prev_odom to zero...
  bool alignLaser = false;
  app->input_log_fname = NULL;
  app->chan = NULL;
  app->verbose = 0;
  app->do_drawing = 0;
  app->publish_pose = 0;
  app->scanmatchBeforAdd = 0;
  app->odometryConfidence = LINEAR_DIST_TO_ADD;

  // set to default values
  //parameters for a hokuyo with the helicopters mirror's attached
  app->validBeamAngles[0] = -2.1;
  app->validBeamAngles[1] = 2.1;
  app->beam_skip = 3;
  app->spatialDecimationThresh = .2;
  app->maxRange = 29.7;
  app->maxUsableRange = 8;

  const char *optstring = "hc:davm:R:B:D:M:ps::tTf:I:";
  int c;
  struct option long_opts[] = {
      { "help", no_argument, 0, 'h' },
      { "chan", required_argument, 0, 'c' },
      { "draw", no_argument, 0, 'd' },
      { "mode", required_argument, 0, 'm' },
      { "range", required_argument, 0, 'R' },
      { "beamskip", required_argument, 0, 'B' },
      { "decimation", required_argument, 0, 'D' },
      { "publish_pose", no_argument, 0, 'p' },
      { "mask", required_argument, 0, 'M' },
      { "aligned", no_argument, 0, 'a' },
      { "scanmatch", optional_argument, 0, 's' },
      { "verbose", no_argument, 0, 'v' },
      { "trajectory", no_argument, 0, 't' },
      { "complete_traj", no_argument, 0, 'T' },
      { "from_log", required_argument, 0, 'f' },
      { "initialize", required_argument, 0, 'I' },
      { 0, 0, 0, 0 } };

  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'c':
      free(app->chan);
      app->chan = strdup(optarg);
      break;
    case 'v':
      app->verbose = 1;
      break;
    case 'd':
      app->do_drawing = 1;
      break;
    case 'f':
      app->input_log_fname = strdup(optarg);
      break;
    case 'p':
      app->publish_pose = 1;
      break;
    case 's':
      app->scanmatchBeforAdd = 1;
      if (optarg != NULL) {
        app->odometryConfidence = atof(optarg);
      }
      break;
    case 'm':
      if (optarg && optarg[0] == 'S') {
        // assume we want sick mode:
        app->maxRange = 79.0;
        app->beam_skip = 0;
        // anything else required?
      }
      break;
    case 'R':
      app->maxRange = strtod(optarg, 0);
      break;
    case 'B':
      app->beam_skip = atoi(optarg);
      break;
    case 'D':
      app->spatialDecimationThresh = strtod(optarg, 0);
      break;
    case 'M':
      sscanf(optarg, "%f,%f", &app->validBeamAngles[0], &app->validBeamAngles[1]);
      break;
    case 'a':
      alignLaser = true;
      break;
    case 't':
      app->publishTraj = true;
      break;
    case 'T':
      app->publishCompleteTraj = true;
      break;
    case 'I':
      app->initialization_log_name = strdup(optarg);
    case 'h':
      default:
      usage(argv[0]);
      return 1;
    }
  }

  if (app->chan == NULL) {
    if (alignLaser)
      app->chan = strdup("ROBOT_LASER");
    else
      app->chan = strdup("LASER");
  }

  if (app->verbose) {
    if (alignLaser)
      printf("INFO: Listening for robot_laser msgs on %s\n", app->chan);
    else
      printf("INFO: Listening for laser msgs on %s\n", app->chan);

    printf("INFO: Do Draw:%d\n", app->do_drawing);
    printf("INFO: publish_pose:%d\n", app->publish_pose);
    printf("INFO: Max Range:%lf\n", app->maxRange);
    printf("INFO: SpatialDecimationThresh:%lf\n", app->spatialDecimationThresh);
    printf("INFO: Beam Skip:%d\n", app->beam_skip);
    printf("INFO: validRange:%f,%f\n", app->validBeamAngles[0], app->validBeamAngles[1]);
  }

  /* LCM */
  app->lcm_pub = bot_lcm_get_global(NULL);
  if (app->input_log_fname != NULL) {
    char buf[1024];
    printf("running on LCM log file: %s\n", app->input_log_fname);
    sprintf(buf, "file://%s?speed=0", app->input_log_fname);
    app->lcm_recv = lcm_create(buf);
  }
  else {
    app->lcm_recv = app->lcm_pub;
  }

  //hardcoded loop closing scan matcher params
  double metersPerPixel_lc = .02; //translational resolution for the brute force search
  double thetaResolution_lc = .01; //angular step size for the brute force search
  int useGradientAscentPolish_lc = 1; //use gradient descent to improve estimate after brute force search
  int useMultires_lc = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes
  bool useThreads_lc = true;
//  if (app->input_log_fname)
//    useThreads_lc = false;

  //create the actual scan matcher object
  app->isamslam = new IsamSlam(app->lcm_pub, metersPerPixel_lc, thetaResolution_lc, useGradientAscentPolish_lc,
      useMultires_lc, app->do_drawing, useThreads_lc);

  if (app->isamslam->sm->isUsingIPP())
    fprintf(stderr, "Using IPP\n");
  else
    fprintf(stderr, "NOT using IPP\n");

  //hardcoded scan matcher params
  double metersPerPixel = .02; //translational resolution for the brute force search
  double thetaResolution = .02; //angular step size for the brute force search
  sm_incremental_matching_modes_t matchingMode = SM_GRID_COORD; //use gradient descent to improve estimate after brute force search
  int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes

  double initialSearchRangeXY = .15; //nominal range that will be searched over
  double initialSearchRangeTheta = .1;

  //SHOULD be set greater than the initialSearchRange
  double maxSearchRangeXY = .3; //if a good match isn't found I'll expand and try again up to this size...
  double maxSearchRangeTheta = .2; //if a good match isn't found I'll expand and try again up to this size...

  int maxNumScans = 30; //keep around this many scans in the history
  double addScanHitThresh = .90; //add a new scan to the map when the number of "hits" drops below this

  int useThreads = 1;
//  if (app->input_log_fname)
//    useThreads = false;

  //create the incremental scan matcher object
  app->sm_incremental = new ScanMatcher(metersPerPixel, thetaResolution, useMultires, useThreads, false);
  app->sm_incremental->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY, maxSearchRangeXY,
      initialSearchRangeTheta, maxSearchRangeTheta, matchingMode, addScanHitThresh, false, .3, NULL);

  //create the SLAM scan matcher object
  app->sm = new ScanMatcher(metersPerPixel, thetaResolution, useMultires, useThreads, false);
  app->sm->initSuccessiveMatchingParams(maxNumScans, LINEAR_DIST_TO_ADD, LINEAR_DIST_TO_ADD, ANGULAR_DIST_TO_ADD,
      ANGULAR_DIST_TO_ADD, matchingMode, addScanHitThresh, false, .3, NULL);

  memset(&app->sm->currentPose, 0, sizeof(app->sm->currentPose));
  app->sm->prevPose = app->sm->currentPose;

  //subscribe to messages
  if (!alignLaser)
    bot_core_planar_lidar_t_subscribe(app->lcm_recv, app->chan, laser_handler, app);
  //  else //TODO:suppor this?
  //    carmen3d_robot_laser_t_subscribe(app->lcm_recv, app->chan, robot_laser_handler, app);

  //  carmen3d_log_annotate_msg_t_subscribe(app->lcm_recv, "SLAM_OPTIM", optim_handler, app);

  // setup sigaction();
  struct sigaction new_action;
  new_action.sa_sigaction = sig_action;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;

  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGKILL, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);

  if (app->initialization_log_name != NULL) {
    char buf[1024];
    printf("Initializing from LCM log file: %s\n", app->initialization_log_name);
    sprintf(buf, "file://%s?speed=0", app->initialization_log_name);
    lcm_t * lcmtmp = lcm_create(buf);
    while (lcm_handle(app->lcm_recv) >= 0)
      fprintf(stderr, ".");
    fprintf(stderr, "\nInitialization done!\n");
  }

  /* sit and wait for messages */
  while (still_groovy)
    if (lcm_handle(app->lcm_recv) < 0)
      break; //log done

  still_groovy = true;
  while (true) {
    if (app->isamslam->loopClosedUpTill == app->isamslam->trajectory.size())
      break;
    if (!still_groovy)
      break;
    sleep(1);
  }

  app_destroy(app);

  return 0;
}

