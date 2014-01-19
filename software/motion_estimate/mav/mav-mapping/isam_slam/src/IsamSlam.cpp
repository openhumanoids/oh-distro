/*
 * IsamSlam.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: abachrac
 */

#include "IsamSlam.hpp"
#include <math.h>
#include <unistd.h>
#include <GL/gl.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>
#include <lcmtypes/isam_slam_position_t.h>
#include <lcmtypes/isam_slam_trajectory_t.h>

#define MISS_INC -1.0
#define HIT_INC 1.5
#define CLAMPING_THRES_MIN -2
#define CLAMPING_THRES_MAX 7
inline float logOddsToProb(float l)
{
  return 1. - (1. / (1. + exp(l)));
}
inline float logOddsToML(float l)
{
  return l > 0.1 ? 1. : (l < -0.1 ? 0 : .5);
}

using namespace scanmatch;
IsamSlam::IsamSlam(lcm_t * _lcm, double metersPerPixel_, double thetaResolution_, int useGradientAscentPolish_,
    int useMultires_, int doDrawing_, bool useThreads_) :
    lcm(_lcm), loopClosedUpTill(0), gridmap(NULL), gridmapMetersPerPixel(.1), gridmapMargin(10), doDrawing(doDrawing_),
        useThreads(useThreads_)
{
  //supply dummy values for things we don't care about
  sm = new scanmatch::ScanMatcher(metersPerPixel_, thetaResolution_, useMultires_, false, false);

  slam = new Slam();
  // Create first node at this pose: we add a prior to keep the
  // first pose in place, which is an arbitrary choice.
  Matrix sqrtinf3 = 10. * Matrix::eye(3);
  // create a first pose (a node)
  origin_node = new Pose2d_Node();
  // add it to the graph
  slam->add_node(origin_node);
  // create a prior measurement (a factor)
  Pose2d origin(0., 0., 0.);
  Pose2d_Factor* prior = new Pose2d_Factor(origin_node, origin, sqrtinf3);
  // add it to the graph
  slam->add_factor(prior);

  lcmgl_graph = bot_lcmgl_init(lcm, "isam_graph");
  lcmgl_map = bot_lcmgl_init(lcm, "isam_map");
  lcmgl_point_cloud = bot_lcmgl_init(lcm, "isam_pointcloud");
  if (useThreads) {
    //remember to make sure that sm_tictoc gets initialized
    killThread = 0;

    /* Initialize mutex and condition variable objects */
    pthread_mutex_init(&trajectory_mutex, NULL);
    pthread_cond_init(&loop_closer_cond, NULL);

    //create rebuilder thread
    pthread_create(&loop_closer, 0, (void *(*)(void *)) loopClose_thread_func, (void *) this);
  }

}

IsamSlam::~IsamSlam()
{

  if (useThreads) {
    //kill the loop closer thread
    while (killThread != -1) {
      killThread = 1;
      pthread_cond_broadcast(&loop_closer_cond);
      usleep(10000);
    }
    //aquire all locks so we can destroy them
    pthread_mutex_lock(&trajectory_mutex);
    // destroy mutex and condition variable objects
    pthread_mutex_destroy(&trajectory_mutex);
    pthread_cond_destroy(&loop_closer_cond);
  }
  fprintf(stderr, "Killed loop closing thread");

  delete origin_node;
  delete slam;
  delete sm;

  //clear the trajectory
  for (unsigned i = 0; i < trajectory.size(); i++)
    delete trajectory[i];
  trajectory.clear();

}

void IsamSlam::addNodeToSlam(Pose2d &prev_curr_tranf, Matrix &cov, Scan * scan, Scan * maxRangeScan, int64_t utime,
    double height, double rp[2])
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  SlamPose * slampose = new SlamPose(utime, trajectory.size() + 1, scan, maxRangeScan, height, rp);
  slam->add_node(slampose->pose2d_node);

  Pose2d_Pose2d_Factor* constraint;
  if (!trajectory.empty())
    constraint = new Pose2d_Pose2d_Factor(trajectory.back()->pose2d_node, slampose->pose2d_node, prev_curr_tranf, cov);
  else
    constraint = new Pose2d_Pose2d_Factor(origin_node, slampose->pose2d_node, prev_curr_tranf, cov);

  slampose->constraints.push_back(constraint);
  slam->add_factor(constraint);
  trajectory.push_back(slampose);

  //  if (trajectory.size() % MOD_BATCH == 0) {
  //    // batch solve periodically to avoid fill-in
  //    slam->batch_optimization_step();
  //    //slam.full_optimization();
  //  }
  //  else {
  //    // for efficiency, incrementally update most of the time.
  //    slam->incremental_update();
  //    if (trajectory.size() % MOD_SOLVE == 0) {
  //      slam->solve();
  //    }
  //  }

  if (useThreads) {
    pthread_mutex_unlock(&trajectory_mutex);
    pthread_cond_broadcast(&loop_closer_cond);  //tell the loop closing thread to have at it...
  }
  else
    doLoopClosing(trajectory.size() - 1);
}
void IsamSlam::doLoopClosing(int loop_close_ind)
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  int minLookBack = 60;
  int matchRegionNumScans = 10;
  float maxCheckDist = 10;
  if (loop_close_ind < minLookBack) {
    trajectory[loop_close_ind]->loopClosureChecked = true;
    if (useThreads)
      pthread_mutex_unlock(&trajectory_mutex);
    return; //nothin to loop close against yet
  }
  //see if the end of the trajectory matches something toward the back
  SlamPose * slampose_tocheck = trajectory[loop_close_ind];
  SlamPose * slampose_toverify = trajectory[loop_close_ind - 1];
  slampose_tocheck->loopClosureChecked = true;
  Pose2d tocheck_value = slampose_tocheck->pose2d_node->value();
  Pose2d toverify_value = slampose_toverify->pose2d_node->value();
  smPoint p0 = { tocheck_value.x(), tocheck_value.y() };

  //find closest point in the trajectory to build map from
  int closestInd = -1;
  double closestDist = 1e9;
  for (int i = 0; i < (loop_close_ind - minLookBack); i++) {
    Pose2d value = trajectory[i]->pose2d_node->value();
    smPoint p1 = { value.x(), value.y() };
    double d = sm_dist(&p0, &p1);
    if (d < closestDist) {
      closestInd = i;
      closestDist = d;
    }
  }
  if (closestDist > maxCheckDist) {
    //    fprintf(stderr, "no point is within 6m, closest was %f\n", closestDist);
    if (useThreads)
      pthread_mutex_unlock(&trajectory_mutex);
    return;
  }

  sm_tictoc("update_transforms");
  //lets try to match against the closest
  //TODO: may want to match against multiple trajectory sections
  int rangeStart = sm_imax(0, closestInd - matchRegionNumScans);
  int rangeEnd = closestInd + matchRegionNumScans; //has to be at least 40 from the end
  //  fprintf(stderr, "closest is %f trying to do loop closure on scans %d-%d\n", closestDist,rangeStart,rangeEnd);
  for (int i = rangeStart; i <= rangeEnd; i++) {
    //update the scan match transform with the current SLAM value estimate
    Pose2d value = trajectory[i]->pose2d_node->value();
    ScanTransform newT;
    newT.x = value.x();
    newT.y = value.y();
    newT.theta = value.t();
    trajectory[i]->scan->applyTransform(newT);
    sm->addScan(trajectory[i]->scan, false);
  }
  sm_tictoc("update_transforms");

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

  sm_tictoc("build_map");
  sm->addScan(NULL, true); //actually do the raster rebuild
  sm_tictoc("build_map");

  sm_tictoc("LC_Match");
  slampose_tocheck->scan->T.score = 0;
  ScanTransform lc_r = sm->gridMatch(slampose_tocheck->scan->points, slampose_tocheck->scan->numPoints,
      &slampose_tocheck->scan->T, 6.0, 6.0, M_PI / 6.0);
  sm_tictoc("LC_Match");
  double hitpct = lc_r.hits / (double) slampose_tocheck->scan->numPoints;
  double sxx = lc_r.sigma[0];
  double sxy = lc_r.sigma[1];
  double syy = lc_r.sigma[4];
  double stt = lc_r.sigma[8];

  bool accepted = false;
  if (hitpct > .80) {
    fprintf(stderr, "tentatively ACCEPTED match for node %d had %f%% hits...", slampose_tocheck->node_id, hitpct);
    accepted = true;
  }
  else {
    fprintf(stderr, "REJECTED match for node %d had %f%% hits, sxx=%f,sxy=%f,syy=%f,stt=%f\n",
        slampose_tocheck->node_id, hitpct, lc_r.sigma[0], lc_r.sigma[1], lc_r.sigma[4], lc_r.sigma[8]);
  }

  if (accepted) {
    double sigma[9];
    memcpy(sigma, lc_r.sigma, 9 * sizeof(double));
    double evals[3] = { 0 };
    double evals_sq[9] = { 0 };
    double evecs[9] = { 0 };
    CvMat cv_sigma = cvMat(3, 3, CV_64FC1, sigma);
    CvMat cv_evals = cvMat(3, 1, CV_64FC1, evals);
    CvMat cv_evecs = cvMat(3, 3, CV_64FC1, evecs);
    cvEigenVV(&cv_sigma, &cv_evecs, &cv_evals);
    if (evals[0] < .005) {
      fprintf(stderr, "sxx=%f,sxy=%f,syy=%f,stt=%f, eigs=[%f %f %f]\n", sxx, sxy, syy, stt, evals[0], evals[1],
          evals[2]);
      accepted = true;
    }
    else {
      fprintf(stderr, "REJECTED sxx=%f,sxy=%f,syy=%f,stt=%f, eigs=[%f %f %f]\n", sxx, sxy, syy, stt, evals[0],
          evals[1], evals[2]);
      accepted = false;
    }
  }

  if (accepted) {
    Pose2d closestValue = trajectory[closestInd]->pose2d_node->value();
    Pose2d newToCheckValue(lc_r.x, lc_r.y, lc_r.theta);
    Pose2d delta = closestValue.ominus(newToCheckValue);
    if (sqrt(sm_sq(delta.x()) + sm_sq(delta.y())) > 3) {
      fprintf(stderr, "REJECTED dx=%f, dy=%f, dt = %f\n", delta.x(), delta.y(), bot_to_degrees(delta.t()));
      accepted = false;
    }

  }

  if (accepted) {
    if (doDrawing) {
      sm->drawGUI(slampose_tocheck->scan->points, slampose_tocheck->scan->numPoints, lc_r, NULL, "LC_accept", CV_RGB(0,
          255, 0));
    }
    //perform rigidity check
    Pose2d newToCheckValue(lc_r.x, lc_r.y, lc_r.theta);
    Pose2d oldDelta = toverify_value.ominus(tocheck_value);
    Pose2d newToVerifyValue = newToCheckValue.oplus(oldDelta); //if match is rigid, verify scan should be here...
    Pose2d sanity = tocheck_value.oplus(oldDelta);

    ScanTransform newT;
    memset(&newT, 0, sizeof(newT));
    newT.x = newToVerifyValue.x();
    newT.y = newToVerifyValue.y();
    newT.theta = newToVerifyValue.t();
    ScanTransform verify_r = sm->gridMatch(slampose_toverify->scan->points, slampose_toverify->scan->numPoints, &newT,
        1.0, 1.0, M_PI / 12.0);
    smPoint p0 = { verify_r.x, verify_r.y };
    smPoint p1 = { newT.x, newT.y };
    double dist = sm_dist(&p0, &p1);
    double adist = fabs(sm_angle_subtract(verify_r.theta, newT.theta));
    if (dist < .025 && adist < .01) {
      fprintf(stderr, "Match PASSED rigidity check... dist=%f, adist=%f\n", dist, adist);
      if (doDrawing) {
        sm->drawGUI(slampose_toverify->scan->points, slampose_toverify->scan->numPoints, verify_r, NULL, "LC_verify",
            CV_RGB(0, 255, 0));
      }
    }
    else {
      fprintf(stderr, "Match FAILED rigidity check... dist=%f, adist=%f\n", dist, adist);
      if (doDrawing) {
        sm->drawGUI(slampose_toverify->scan->points, slampose_toverify->scan->numPoints, verify_r, NULL,
            "LC_REJECTED_verify", CV_RGB(255, 0, 0));
      }
      accepted = false;
    }

  }

  if (accepted) {
    sm_tictoc("loopCloseOptimize");
    if (useThreads)
      pthread_mutex_lock(&trajectory_mutex);

    //add the edge to isam
    Pose2d matchedPos(lc_r.x, lc_r.y, lc_r.theta);
    Pose2d transf = matchedPos.ominus(trajectory[closestInd]->pose2d_node->value()); //get the delta between that pose and the current

    //TODO: probably want to scale the covariance...
    //rotate the cov to body frame
    double Rcov[9];
    sm_rotateCov2D(lc_r.sigma, -lc_r.theta, Rcov);
    Matrix cov(3, 3, Rcov);
    cov = 10000.0 * cov;
    //    printf("LC cov:\n");
    //    cov.print();
    //    printf("\n");
    double cov_hardcode[9] = { .5, 0, 0, 0, .5, 0, 0, 0, .2 };
    Matrix cov_hc(3, 3, cov_hardcode);

    Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(trajectory[closestInd]->pose2d_node,
        slampose_tocheck->pose2d_node, transf, cov_hc);
    slampose_tocheck->constraint_ids.push_back(closestInd);
    slampose_tocheck->constraints.push_back(constraint);
    slam->add_factor(constraint);

    //optimize the graph
    sm_tictoc("batch_optimization");
    slam->batch_optimization();
    sm_tictoc("batch_optimization");

    //    sm_tictoc("update");
    //    slam->update();
    //    sm_tictoc("update");

    if (useThreads)
      pthread_mutex_unlock(&trajectory_mutex);
    sm_tictoc("loopCloseOptimize");
  }
  else if (doDrawing) {
    sm->drawGUI(slampose_tocheck->scan->points, slampose_tocheck->scan->numPoints, lc_r, NULL, "LC_reject", CV_RGB(255,
        0, 0));
  }
  sm->clearScans(false);    //clear out scans to get ready for next time
}

void * IsamSlam::loopClose_thread_func(IsamSlam*parent)
{
  fprintf(stderr, "Loop closure thread started\n");

  pthread_mutex_lock(&parent->trajectory_mutex);
  while (!parent->killThread) {
    int ind_to_check = -1;
    if (!parent->trajectory.empty()) {
      if (!parent->trajectory.back()->loopClosureChecked) {
        ind_to_check = parent->trajectory.size() - 1;
      }
      else {
        int randInd = bot_randf_in_range(parent->loopClosedUpTill, parent->trajectory.size() - 1);
        if (!parent->trajectory[randInd]->loopClosureChecked) {
          ind_to_check = randInd;
        }
        else {
          //work backwords
          for (int i = parent->trajectory.size() - 1; i >= parent->loopClosedUpTill; i--) {
            if (!parent->trajectory[i]->loopClosureChecked) {
              ind_to_check = i;
              break;
            }
          }
        }
      }
    }
    if (ind_to_check < 0) {
      //all nodes have been checked already... so lets go to sleep
      parent->loopClosedUpTill = parent->trajectory.size();
      pthread_cond_wait(&parent->loop_closer_cond, &parent->trajectory_mutex);
      continue;
    }
    else {
      //need to check this node
      pthread_mutex_unlock(&parent->trajectory_mutex); //unlock since lock is reacquired in doLoopClosing
      parent->doLoopClosing(ind_to_check);
      pthread_mutex_lock(&parent->trajectory_mutex); //lock to go back around the loop
    }

  }
  pthread_mutex_unlock(&parent->trajectory_mutex);
  parent->killThread = -1;
  fprintf(stderr, "Loop closure thread stopped\n");
  return NULL;
}

void IsamSlam::checkForUpdates()
{
  double minxy[2] = { DBL_MAX, DBL_MAX };
  double maxxy[2] = { -DBL_MAX, -DBL_MAX };
  // Compute the bounds of the current map.
  bool needToDelete = false;

  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);
  for (unsigned t = 0; t < trajectory.size(); t++) {
    int updated = trajectory[t]->updateScan(); //update the scan to match the current slam pos
    if (updated && trajectory[t]->rendered)
      needToDelete = true;
    for (unsigned j = 0; j < trajectory[t]->allScans.size(); j++) {
      Scan * s = trajectory[t]->allScans[j];

      for (unsigned cidx = 0; cidx < s->contours.size(); cidx++) {
        for (unsigned i = 0; i < s->contours[cidx]->points.size(); i++) {
          smPoint p = s->contours[cidx]->points[i];
          minxy[0] = fmin(minxy[0], p.x);
          maxxy[0] = fmax(maxxy[0], p.x);
          minxy[1] = fmin(minxy[1], p.y);
          maxxy[1] = fmax(maxxy[1], p.y);
        }
      }
    }
  }
  if (gridmap != NULL) {
    if (minxy[0] < gridmap->xy0[0] || maxxy[0] > gridmap->xy1[0] || minxy[1] < gridmap->xy0[1] || maxxy[1]
        > gridmap->xy1[1])
      needToDelete = true;
    if (needToDelete) {
      delete gridmap;
      gridmap = NULL;
      //mark all poses as unrendered
      for (unsigned t = 0; t < trajectory.size(); t++)
        trajectory[t]->rendered = false;
    }
  }
  if (gridmap == NULL) {
    //    printf("creating new gridmap\n");
    minxy[0] -= gridmapMargin;
    minxy[1] -= gridmapMargin;
    maxxy[0] += gridmapMargin;
    maxxy[1] += gridmapMargin;
    gridmap = new occ_map::FloatPixelMap(minxy, maxxy, gridmapMetersPerPixel, 0);
  }
  //  else {
  //    printf("reusing old gridmap\n");
  //  }
  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

}

void IsamSlam::publishSlamPos()
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  SlamPose * slampose = trajectory.back();
  Pose2d value = slampose->pose2d_node->value();

  isam_slam_position_t slam_pos;
  memset(&slam_pos, 0, sizeof(slam_pos));
  slam_pos.utime = slampose->utime;
  slam_pos.pos[0] = value.x();
  slam_pos.pos[1] = value.y();
  slam_pos.pos[2] = slampose->height;
  double rpy[3] = { 0, 0, value.t() };
  bot_roll_pitch_yaw_to_quat(rpy, slam_pos.orientation);
  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);
  isam_slam_position_t_publish(lcm, "SLAM_POSITION", &slam_pos);

}

void IsamSlam::publishSlamTrajectory()
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);
  isam_slam_trajectory_t pose_list;
  memset(&pose_list, 0, sizeof(pose_list));
  pose_list.num_poses = trajectory.size();
  pose_list.poses = (isam_slam_position_t*) calloc(pose_list.num_poses, sizeof(isam_slam_position_t));

  for (unsigned int i = 0; i < trajectory.size(); i++) {
    SlamPose * slampose = trajectory[i];
    Pose2d value = slampose->pose2d_node->value();

    pose_list.poses[i].utime = slampose->utime;
    pose_list.poses[i].pos[0] = value.x();
    pose_list.poses[i].pos[1] = value.y();
    pose_list.poses[i].pos[2] = slampose->height;
    double rpy[3] = { slampose->rp[0], slampose->rp[1], value.t() };
    bot_roll_pitch_yaw_to_quat(rpy, pose_list.poses[i].orientation);
  }
  pose_list.utime = trajectory.back()->utime;
  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);
  isam_slam_trajectory_t_publish(lcm, "SLAM_TRAJECTORY", &pose_list);

}

void IsamSlam::publishCompleteTrajectory(const char * saveFname)
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  //compute total number of poses
  int numIntermediateNodes = 0;
  for (unsigned int i = 0; i < trajectory.size(); i++) {
    numIntermediateNodes += trajectory[i]->intermediate_nodes.size();
  }

  isam_slam_trajectory_t pose_list;
  memset(&pose_list, 0, sizeof(pose_list));
  pose_list.num_poses = trajectory.size() + numIntermediateNodes;
  pose_list.poses = (isam_slam_position_t*) calloc(pose_list.num_poses, sizeof(isam_slam_position_t));

  int count = 0;
  for (unsigned int i = 0; i < trajectory.size(); i++) {
    SlamPose * slampose = trajectory[i];
    Pose2d slam_value = slampose->pose2d_node->value();

    pose_list.poses[count].utime = slampose->utime;
    pose_list.poses[count].pos[0] = slam_value.x();
    pose_list.poses[count].pos[1] = slam_value.y();
    pose_list.poses[count].pos[2] = slampose->height;
    double rpy[3] = { slampose->rp[0], slampose->rp[1], slam_value.t() };
    bot_roll_pitch_yaw_to_quat(rpy, pose_list.poses[count].orientation);
    count++;
    for (int j = 0; j < slampose->intermediate_nodes.size(); j++) {
      SlamPose * interPose = slampose->intermediate_nodes[j];
      Pose2d value = slam_value.oplus(interPose->pose2d_node->value());

      pose_list.poses[count].utime = interPose->utime;
      pose_list.poses[count].pos[0] = value.x();
      pose_list.poses[count].pos[1] = value.y();
      pose_list.poses[count].pos[2] = interPose->height;
      double rpy[3] = { interPose->rp[0], interPose->rp[1], value.t() };
      bot_roll_pitch_yaw_to_quat(rpy, pose_list.poses[count].orientation);
      count++;

    }
  }
  pose_list.utime = trajectory.back()->utime;

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);
  isam_slam_trajectory_t_publish(lcm, "SLAM_COMPLETE_TRAJECTORY", &pose_list);

  if (saveFname != NULL) {
    lcm_t * out_log = lcm_create(string("file://" + string(saveFname) + "?mode=w").c_str());
    isam_slam_trajectory_t_publish(out_log, "SLAM_COMPLETE_TRAJECTORY", &pose_list);
  }
}

void IsamSlam::renderGridmap()
{

  checkForUpdates();

  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);
  sm_tictoc("renderGridmap_nolocking");
  float map_value_bounds[2] = { CLAMPING_THRES_MIN, CLAMPING_THRES_MAX };
  for (unsigned t = 0; t < trajectory.size(); t++) {
    SlamPose * slampose = trajectory[t];
    if (slampose->rendered)
      continue;
    else
      slampose->rendered = true;
    Pose2d bodyPos = slampose->pose2d_node->value();
    double bodyP[2] = { bodyPos.x(), bodyPos.y() };
    //    slampose->updateScan();//update the scan to match the current slam pos

    for (unsigned s = 0; s < slampose->allScans.size(); s++) {
      if (s > 0 && trajectory.size() - t > 3)
        continue; //only draw maxranges for last 3 scans
      Scan * scan = slampose->allScans[s];
      //ray trace to compute the map...
      sm_tictoc("render_raytrace");
      for (unsigned i = 0; i < scan->numPoints; i++) {
        gridmap->rayTrace(bodyP, smPoint_as_array(&scan->ppoints[i]), MISS_INC, (s == 0) * HIT_INC, map_value_bounds);
      }
      if (s == 0) {
        //draw the contours as lines for cleaner walls
        for (unsigned cidx = 0; cidx < scan->contours.size(); cidx++) {
          for (unsigned i = 0; i + 1 < scan->contours[cidx]->points.size(); i++) {
            //draw the occupied regions
            smPoint p0 = scan->contours[cidx]->points[i];
            smPoint p1 = scan->contours[cidx]->points[i + 1];
            gridmap->rayTrace(smPoint_as_array(&p0), smPoint_as_array(&p1), HIT_INC, HIT_INC, map_value_bounds);
          }
        }
      }

      sm_tictoc("render_raytrace");
    }
  }
  sm_tictoc("renderGridmap_nolocking");

  //  cvSaveImage("gridmap.bmp", &gridmap->distim);
  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

}

void IsamSlam::publishGridMap()
{

  //  occ_map::FloatPixelMap * probMap = new occ_map::FloatPixelMap(gridmap, &logOddsToProb);
  occ_map::FloatPixelMap * probMap = new occ_map::FloatPixelMap(gridmap, true, &logOddsToML);
  const occ_map_pixel_map_t * lcm_msg = probMap->get_pixel_map_t(bot_timestamp_now());
  occ_map_pixel_map_t_publish(lcm, "SLAM_MAP", lcm_msg);
  delete probMap;
}

void IsamSlam::saveMap(const char * fname)
{
  occ_map::FloatPixelMap * probMap = new occ_map::FloatPixelMap(gridmap, true, &logOddsToML);
  probMap->saveToFile(fname);
  delete probMap;
}

void IsamSlam::drawGraph()
{

  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  double usPerMeter = 60 * 1e6;
  bot_lcmgl_line_width(lcmgl_graph, 1);
  bot_lcmgl_point_size(lcmgl_graph, 10);
  bot_lcmgl_color3f(lcmgl_graph, 1, 1, 0);

  //draw nodes
  for (unsigned i = 0; i < trajectory.size(); i++) {
    SlamPose * slampose = trajectory[i];
    Pose2d value = slampose->pose2d_node->value();
    double xyz[3] = { value.x(), value.y(), (trajectory[i]->utime - trajectory[0]->utime) / usPerMeter };
    bot_lcmgl_circle(lcmgl_graph, xyz, .3);
  }

  //draw odometry edges
  bot_lcmgl_color3f(lcmgl_graph, 0, 1, 1);
  bot_lcmgl_line_width(lcmgl_graph, 3);
  bot_lcmgl_begin(lcmgl_graph, GL_LINES);
  for (unsigned i = 1; i < trajectory.size(); i++) {
    SlamPose * slampose_prev = trajectory[i - 1];
    SlamPose * slampose_curr = trajectory[i];
    Pose2d value_prev = slampose_prev->pose2d_node->value();
    Pose2d value_curr = slampose_curr->pose2d_node->value();
    bot_lcmgl_vertex3f(lcmgl_graph, value_prev.x(), value_prev.y(), (slampose_prev->utime - trajectory[0]->utime)
        / usPerMeter);
    bot_lcmgl_vertex3f(lcmgl_graph, value_curr.x(), value_curr.y(), (slampose_curr->utime - trajectory[0]->utime)
        / usPerMeter);
  }
  bot_lcmgl_end(lcmgl_graph);

  //draw loop closures
  bot_lcmgl_line_width(lcmgl_graph, 3);
  bot_lcmgl_color3f(lcmgl_graph, 1, 0, 0);
  for (unsigned i = 1; i < trajectory.size(); i++) {
    SlamPose * slampose = trajectory[i];
    for (unsigned j = 1; j < slampose->constraint_ids.size(); j++) { //constraints after the first are loop closures
      SlamPose * slampose2 = trajectory[slampose->constraint_ids[j]];

      Pose2d_Node* pose1node = slampose->pose2d_node;
      Pose2d_Node* pose2node = slampose2->pose2d_node;
      Pose2d value1 = pose1node->value();
      Pose2d value2 = pose2node->value();
      bot_lcmgl_begin(lcmgl_graph, GL_LINES);
      bot_lcmgl_vertex3f(lcmgl_graph, value1.x(), value1.y(), (slampose->utime - trajectory[0]->utime) / usPerMeter);
      bot_lcmgl_vertex3f(lcmgl_graph, value2.x(), value2.y(), (slampose2->utime - trajectory[0]->utime) / usPerMeter);
      bot_lcmgl_end(lcmgl_graph);
    }

  }

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

  bot_lcmgl_switch_buffer(lcmgl_graph);
}

void IsamSlam::drawMap()
{

  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  bot_lcmgl_line_width(lcmgl_map, 3);

  for (unsigned i = 0; i < trajectory.size(); i++) {
    SlamPose * slampose = trajectory[i];
    slampose->updateScan(); //update the scan to match the current slam pos
    for (unsigned s = 0; s < 1; s++) {
      //      for (unsigned s = 0; s < slampose->allScans.size(); s++) {
      Scan * scan = slampose->allScans[s];
      if (s == 0)
        bot_lcmgl_color3f(lcmgl_map, 0, 0, 1);
      else
        bot_lcmgl_color3f(lcmgl_map, 1, 1, 0);

      //actually draw the contours for this node
      for (unsigned cidx = 0; cidx < scan->contours.size(); cidx++) {
        for (unsigned i = 0; i + 1 < scan->contours[cidx]->points.size(); i++) {
          bot_lcmgl_begin(lcmgl_map, GL_LINES);
          bot_lcmgl_vertex3f(lcmgl_map, scan->contours[cidx]->points[i].x, scan->contours[cidx]->points[i].y, 0);
          bot_lcmgl_vertex3f(lcmgl_map, scan->contours[cidx]->points[i + 1].x, scan->contours[cidx]->points[i + 1].y,
              0);
          bot_lcmgl_end(lcmgl_map);
        }
      }
    }
  }

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

  bot_lcmgl_switch_buffer(lcmgl_map);
}

void IsamSlam::draw3DPointCloud()
{
  checkForUpdates();
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  double blue_height = .2;
  double red_height = 3;
  double z_norm_scale = 1 / (red_height - blue_height);

  bot_lcmgl_push_attrib(lcmgl_point_cloud, GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
  bot_lcmgl_enable(lcmgl_point_cloud, GL_DEPTH_TEST);
  bot_lcmgl_depth_func(lcmgl_point_cloud, GL_LESS);

  bot_lcmgl_point_size(lcmgl_point_cloud, 2);
  bot_lcmgl_begin(lcmgl_point_cloud, GL_POINTS);

  for (unsigned t = 0; t < trajectory.size(); t++) {
    SlamPose * slampose = trajectory[t];
    Pose2d value = slampose->pose2d_node->value();
    Scan * scan = slampose->scan;
    double pBody[3] = { 0, 0, 0 };
    double pLocal[3];
    BotTrans bodyToLocal;
    bodyToLocal.trans_vec[0] = value.x();
    bodyToLocal.trans_vec[1] = value.y();
    bodyToLocal.trans_vec[2] = slampose->height;
    double rpy[3] = { slampose->rp[0], slampose->rp[1], value.t() };
    bot_roll_pitch_yaw_to_quat(rpy, bodyToLocal.rot_quat);
    for (unsigned i = 0; i < scan->numPoints; i++) {
      pBody[0] = scan->points[i].x;
      pBody[1] = scan->points[i].y;
      //transform to local frame
      bot_trans_apply_vec(&bodyToLocal, pBody, pLocal);
      double z_norm = (pLocal[2] - blue_height) * z_norm_scale;
      float * color3fv = bot_color_util_jet(z_norm);
      bot_lcmgl_color3f(lcmgl_point_cloud, color3fv[0], color3fv[1], color3fv[2]);
      bot_lcmgl_vertex3f(lcmgl_point_cloud, pLocal[0], pLocal[1], pLocal[2]);
    }
  }

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

  bot_lcmgl_end(lcmgl_point_cloud);
  bot_lcmgl_pop_attrib(lcmgl_point_cloud);
  bot_lcmgl_switch_buffer(lcmgl_point_cloud);

}

double IsamSlam::computeTrajectoryLength()
{

  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  double trajLength = 0;
  //draw odometry edges
  for (unsigned i = 1; i < trajectory.size(); i++) {
    SlamPose * slampose_prev = trajectory[i - 1];
    SlamPose * slampose_curr = trajectory[i];
    Pose2d value_prev = slampose_prev->pose2d_node->value();
    Pose2d value_curr = slampose_curr->pose2d_node->value();
    double pp[2] = { value_prev.x(), value_prev.y() };
    double pc[2] = { value_curr.x(), value_curr.y() };
    double dist = bot_vector_dist_2d(pp, pc);
    trajLength += dist;
  }
  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);

  return trajLength;
}

Pose2d IsamSlam::getCurrentPose()
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  Pose2d curr_pose;
  if (!trajectory.empty())
    curr_pose = trajectory.back()->pose2d_node->value();
  else
    curr_pose = origin_node->value();

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);
  return curr_pose;
}

void IsamSlam::addIntermediateNode(Pose2d &prev_curr_tranf, int64_t utime, double height, double rp[2])
{
  if (useThreads)
    pthread_mutex_lock(&trajectory_mutex);

  if (!trajectory.empty()) {
    SlamPose * slampose = new SlamPose(utime, -1, NULL, NULL, height, rp);
    slampose->pose2d_node->init(prev_curr_tranf);
    trajectory.back()->intermediate_nodes.push_back(slampose);
  }

  if (useThreads)
    pthread_mutex_unlock(&trajectory_mutex);
}
