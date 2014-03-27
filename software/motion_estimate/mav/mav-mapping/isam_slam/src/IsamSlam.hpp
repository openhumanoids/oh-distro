/*
 * IsamSlam.h
 *
 *  Created on: Oct 22, 2009
 *      Author: abachrac
 */

#ifndef ISAMSLAM_H_
#define ISAMSLAM_H_
#include <scanmatch/ScanMatcher.hpp>
#include <isam/isam.h>
#include "SlamPose.hpp"
#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <pthread.h>
#include <occ_map/PixelMap.hpp>
#include <vector>

// batch solve with variable reordering and relinearization every MOD_BATCH steps
const int MOD_BATCH = 100;
// for incremental steps, solve by backsubstitution every MOD_SOLVE steps
const int MOD_SOLVE = 10;

class IsamSlam {
public:
  IsamSlam(lcm_t * _lcm, double metersPerPixel_, double thetaResolution_, int useGradientAscentPolish_,
      int useMultires_,
      int doDrawing_, bool useThreads_);
  virtual ~IsamSlam();

  void doLoopClosing(int loop_close_ind);
  void addNodeToSlam(Pose2d &prev_curr_tranf, Matrix &cov, Scan * scan, Scan * maxRangeScan, int64_t utime,
      double height, double rp[2]);

  void addIntermediateNode(Pose2d &prev_curr_tranf, int64_t utime, double height, double rp[2]);

  Pose2d getCurrentPose();

  void publishSlamPos();
  void publishSlamTrajectory();
  void publishCompleteTrajectory(const char * saveFname = NULL);

  double computeTrajectoryLength();

  //gridmap rendering functions
  void checkForUpdates();
  void renderGridmap();
  void publishGridMap();

  void drawGraph();
  void drawMap();
  void draw3DPointCloud();
  void saveMap(const char * fname);

  Pose2d_Node * origin_node;
  scanmatch::ScanMatcher * sm; //scan matcher object for loop closing
  Slam * slam; //isam slam object
  std::vector<SlamPose *> trajectory;
  int loopClosedUpTill;

  occ_map::FloatPixelMap * gridmap;
  double gridmapMetersPerPixel;
  double gridmapMargin;

  int doDrawing;

  lcm_t * lcm;
  bot_lcmgl_t * lcmgl_graph;
  bot_lcmgl_t * lcmgl_map;
  bot_lcmgl_t * lcmgl_point_cloud;

  //threading stuff
  bool useThreads;
  int killThread;
  pthread_t loop_closer;
  static void * loopClose_thread_func(IsamSlam*parent);
  pthread_mutex_t trajectory_mutex;
  pthread_cond_t loop_closer_cond;

};

#endif /* ISAMSLAM_H_ */
