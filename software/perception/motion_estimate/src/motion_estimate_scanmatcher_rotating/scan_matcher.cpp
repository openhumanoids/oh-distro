// Scanmatcher which only uses the scans when the laser is close to horizontal
// it filters them out using a head imu and the head-to-laser transform
// Pose is the fusion of the imu Pitch and ROll and the XYTheta from the laser
//
// 40 rpm works well
// Changing this line of ScanMatcher.cpp from this 
//    double sigma = .0675 / metersPerPixel; //sigma is in pixels
// to this, seems to inflate the scan uncertainty - this seems useful:
//    double sigma = .3675 / metersPerPixel; //sigma is in pixels

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <pointcloud_tools/pointcloud_lcm.hpp> // unpack lidar to xyz
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"

#include <scanmatch/ScanMatcher.hpp>

#include <ConciseArgs>

using namespace std;
using namespace Eigen;
using namespace boost::assign; // bring 'operator+()' into scope

using namespace scanmatch;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string lidar_channel_;
    
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    void headImuHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::imu_t* msg);   
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    
    pointcloud_vis* pc_vis_;
    pointcloud_lcm* pc_lcm_;
    int printf_counter_; // used for terminal feedback
    
    // PR-only pose of the head (from the head IMU)
    Eigen::Isometry3d world_to_head_pr_;
    double world_to_head_pitch_;
    double world_to_head_roll_;
    bool init_pr_;
    
    // Scanmatcher:   
    ScanMatcher * sm;
    sm_laser_type_t laser_type_;
    bool do_drawing_;
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_):
    lcm_(lcm_), verbose_(verbose_), 
    lidar_channel_(lidar_channel_){
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  
  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  lcm_->subscribe("HEAD_IMU",&Pass::headImuHandler,this);
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(65010,"Head Pose - PR only",5,1) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose Laser - PR only",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Horz Cloud - Laser"         ,1,1, 60000,1, {0.0, 1.0, 0.0} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60002,"OffH Cloud - Laser"         ,1,1, 60000,1, {1.0, 0.0, 0.0} ));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(65005,"Pose - Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(65006,"3D Cloud - Null"           ,1,1, 65005,1, { 0.0, 0.0, 1.0} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(65007,"2D Cloud - Null"           ,1,1, 65005,1, { 0.0, 1.0, 0.0} ));
  
  printf_counter_ =0;
  
  init_pr_=false;
  
  // Now to setup scanmatcher:
  // hardcoded scan matcher params
  double metersPerPixel = .02; //translational resolution for the brute force search
  double thetaResolution = .01; //angular step size for the brute force search
  
  // This was an important change: (mfallon)
  sm_incremental_matching_modes_t  matchingMode= SM_GRID_COORD; //SM_COORD_ONLY; //use gradient descent to improve estimate after brute force search
  int useMultires = 3; // low resolution will have resolution metersPerPixel * 2^useMultiRes

  // These values were increased from defaul - mfallon
  double initialSearchRangeXY = .65; //nominal range that will be searched over
  double initialSearchRangeTheta = .2;
  //SHOULD be set greater than the initialSearchRange
  double maxSearchRangeXY = .9; //if a good match isn't found I'll expand and try again up to this size...
  double maxSearchRangeTheta = .4; //if a good match isn't found I'll expand and try again up to this size...


  int maxNumScans = 30; //keep around this many scans in the history
  double addScanHitThresh = .80; //add a new scan to the map when the number of "hits" drops below this

  bool stationaryMotionModel = false; //use constant velocity model
  //don't use the prior for anything other than centering the window
  // was 0
  double motionModelPriorWeight =0.0; 

  int useThreads = 1;

  //create the actual scan matcher object
  sm = new ScanMatcher(metersPerPixel, thetaResolution, useMultires,
          useThreads,true);

  if (sm->isUsingIPP())
      fprintf(stderr, "Using IPP\n");
  else
      fprintf(stderr, "NOT using IPP\n");

  ScanTransform startPose;
  memset(&startPose, 0, sizeof(startPose));
  startPose.theta = M_PI / 2; //set the scan matcher to start at pi/2... cuz it looks better
  sm->initSuccessiveMatchingParams(maxNumScans, initialSearchRangeXY,
          maxSearchRangeXY, initialSearchRangeTheta, maxSearchRangeTheta,
          matchingMode, addScanHitThresh,
          stationaryMotionModel,motionModelPriorWeight,&startPose);  
  

  // Additional params:
  laser_type_ = SM_HOKUYO_UTM;
  do_drawing_ =true;

  cout << "Finished setting up\n";  
}


// NB: 'world' mentioned here is simply the lidar frame with pitch and roll corrected for
void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  if (!init_pr_){
    cout << "have laser but no pitch/roll from HEAD_IMU, refusing to publish\n";
    return;
  }

  int64_t pose_id=msg->utime;
  Eigen::Isometry3d null_pose;

  // 1. Determine the 'world'-to-sensor pose using: Pitch, roll and rotation of the laser head
  Eigen::Isometry3d head_to_scan;
  frames_cpp_->get_trans_with_utime( botframes_ ,  "ROTATING_SCAN", "head", msg->utime, head_to_scan);
  Eigen::Isometry3d world_to_scan =  world_to_head_pr_*head_to_scan;
  Isometry3dTime world_to_scan_prT = Isometry3dTime(pose_id, world_to_scan);
  
  if (verbose_){
    pc_vis_->pose_to_lcm_from_list(60000, world_to_scan_prT);
    Isometry3dTime world_to_head_prT = Isometry3dTime(pose_id, world_to_head_pr_);
    pc_vis_->pose_to_lcm_from_list(65010, world_to_head_prT);
  }
  
  // 2. Convert scan into simple point cloud:  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double maxRange = 29.7;
  double validBeamAngles[] ={-10,10}; // consider everything 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_cloud, maxRange,
      validBeamAngles[0], validBeamAngles[1]);  

  // 3. Project scan into 'world' frame:
  Eigen::Isometry3f pose_f = Isometry_d2f(world_to_scan);
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud_s2l (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*scan_cloud, *scan_cloud_s2l,
      pose_f.translation(), pose_quat);  
  
  
  // 4. Test of the Pitch and Roll are reasonable to give planar scans:
  double pitch_threshold =0.1;
  double roll_threshold = 0.1; // about 6degrees
  Eigen::Quaterniond quat_s2l( world_to_scan.rotation() );
  double ypr[3];
  quat_to_euler(quat_s2l, ypr[0], ypr[1], ypr[2]);
  
  if ( fabs( ypr[1]) > pitch_threshold) { // if about 6degrees
    if (verbose_) {
      cout << "ypr: " << ypr[0] << ", " << ypr[1] << ", " << ypr[2] << " | pitch too big\n";
      pc_vis_->ptcld_to_lcm_from_list(60002, *scan_cloud, pose_id, pose_id);       
    }
    return;
  }
  if ( (fabs( ypr[2]) > roll_threshold) &&   (fabs( ypr[2]) < M_PI - roll_threshold)    ) { // if about 6degrees
    if (verbose_) {    
      cout << "ypr: " << ypr[0] << ", " << ypr[1] << ", " << ypr[2] << " | roll too big\n";
      pc_vis_->ptcld_to_lcm_from_list(60002, *scan_cloud, pose_id, pose_id);  
    }
    return;
  }
  
  if (verbose_){
    // Plot original scan in sensor frame:
    pc_vis_->pose_to_lcm_from_list(60000, world_to_scan_prT);
    pc_vis_->ptcld_to_lcm_from_list(60001, *scan_cloud, pose_id, pose_id);  

    // Plot scan in local frame:
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(pose_id, null_pose);
    pc_vis_->pose_to_lcm_from_list(65005, null_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(65006, *scan_cloud_s2l, pose_id, pose_id);
  }
    
    
  // 4. Project valid points onto horizontal
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2d (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // distance off horizontal we are willing to accept non planar points
  double height_threshold = 0.6; // 0.2;  0.7
  // vertical distance between head and laser spindle axis (approximate is ok):
  double head_to_laser = 0.1;
  // remove returns to the sides as they are most effected by the rotation rate:
  double side_threshold = 9.0;
  for (size_t i=0; i< scan_cloud_s2l->points.size(); i++) {
    if ( fabs(scan_cloud_s2l->points[i].z - head_to_laser ) <  height_threshold){ 
      if (  fabs(scan_cloud_s2l->points[i].y) < side_threshold){
      pcl::PointXYZRGB pt = scan_cloud_s2l->points[i];
      pt.z =0;
      cloud2d->points.push_back( pt );
      }
    }
  }
  if (verbose_)  pc_vis_->ptcld_to_lcm_from_list(65007, *cloud2d, pose_id, pose_id); 
    
  // 5. Use these points in scan matching:
  smPoint * points = (smPoint *) calloc( cloud2d->points.size() , sizeof(smPoint));
  for (size_t i=0; i< cloud2d->points.size(); i++) {
    points[i].x = cloud2d->points[i].x;
    points[i].y =cloud2d->points[i].y;
  }
  int numValidPoints = cloud2d->points.size();
  
  std::cout << "A";
  //std::cout << numValidPoints <<" points in scan\n";
  if (numValidPoints < 150){
    std::cout << numValidPoints <<" is not enought points - return\n";
    return; 
  }

   
  // 6. Actually do the matching
  ScanTransform r = sm->matchSuccessive(points, numValidPoints,
          laser_type_, msg->utime, NULL); //don't have a better estimate than prev, so just set prior to NULL
                                              //utime is ONLY used to tag the scans that get added to the map, doesn't actually matter
  //Do drawing periodically!
  static double lastDrawTime = 0;
  if (do_drawing_ && sm_get_time() - lastDrawTime > .2) {
    lastDrawTime = sm_get_time();
    sm->drawGUI(points, numValidPoints, r, NULL);
  }
  free(points);

  // 7. publish pose:
  bot_core::pose_t pose;
  pose.utime = msg->utime;
  pose.pos[0] = r.x;
  pose.pos[1] = r.y;
  pose.pos[2] = 0;
  double rpy[3] = { world_to_head_roll_,world_to_head_pitch_, r.theta }; // pitch and roll comes from imu, yaw from laser
  Eigen::Quaterniond quat = euler_to_quat( r.theta, 0.0, 0.0);             
  pose.orientation[0] = quat.w();
  pose.orientation[1] = quat.x();
  pose.orientation[2] = quat.y();
  pose.orientation[3] = quat.z();
  lcm_->publish( ("POSE_HEAD") , &pose);        


  //Print periodically:
  if (printf_counter_%80 ==0){
    cout << "Filtering: " << lidar_channel_ << " "  << msg->utime << "\n";
    fprintf(
            stderr,
            "x=%+7.3f y=%+7.3f t=%+7.3f\t score=%f hits=%.2f sx=%.2f sxy=%.2f sy=%.2f st=%.2f, numValid=%d\n",
            r.x, r.y, r.theta, r.score, (double) r.hits
                    / (double) numValidPoints, r.sigma[0], r.sigma[1],
            r.sigma[4], r.sigma[8], numValidPoints);
  }
  printf_counter_++;
}


void Pass::headImuHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::imu_t* msg){
  Eigen::Quaterniond head_quat = Eigen::Quaterniond ( msg->orientation[0], msg->orientation[1],
                                                     msg->orientation[2], msg->orientation[3]);
  double head_ypr[3];
  quat_to_euler(head_quat, head_ypr[0], head_ypr[1], head_ypr[2]);
  head_ypr[0] = 0; // set yaw to zero 
  head_quat = euler_to_quat(head_ypr[0], head_ypr[1], head_ypr[2] );
  world_to_head_pr_.setIdentity();
  world_to_head_pr_.translation() << 0,0,0;
  world_to_head_pr_.rotate(head_quat);
  world_to_head_pitch_ = head_ypr[1];
  world_to_head_roll_ = head_ypr[1];
  
  init_pr_=true;
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=FALSE;
  string lidar_channel="ROTATING_SCAN";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(lidar_channel, "l", "lidar_channel", "Incoming LIDAR channel");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << lidar_channel << " is lidar_channel\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,lidar_channel);
  cout << "Ready to scanmatch horizontal scans" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
