#include "major-plane-detect.hpp"
#include <iostream>

#include <pointcloud_tools/filter_planes.hpp>


using namespace maps;
using namespace std;

MajorPlane::MajorPlane(boost::shared_ptr<lcm::LCM> &lcm_, int verbose_lcm_): lcm_(lcm_), verbose_lcm_(verbose_lcm_){
  mBotWrapper.reset(new BotWrapper(lcm_));
  mCollector.reset(new Collector());
  mCollector->setBotWrapper(mBotWrapper);
  mActiveMapId = 0;
  mLcmGl = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "major-plane-detect");
  drc::Clock::instance()->setLcm(lcm_);  

  // create new submap - this keeps all the points in this range
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  // enabling these creates a box thats fixed in world frame
  //mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*10;
  //mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*10;
  mapSpec.mResolution = 0.01;
  mActiveMapId = mCollector->getMapManager()->createMap(mapSpec);
  // start running wrapper
  std::string laserChannel("SCAN");
  mCollector->getDataReceiver()->addChannel(laserChannel, SensorDataReceiver::SensorTypePlanarLidar, laserChannel, "local");
  mCollector->start();  


  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451002,"Plane Detect | Null Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451003,"Plane Detect | Tracked Plane"           ,3,1, 4451002,1, { 1.0, 0.0, 0.0} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Plane Detect | Tracked Normal"           ,3,1, 4451002,1, { 1.0, 0.0, 0.0} ));
  //pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Plane Detect | Plane X"           ,1,1, 4451002,1, { 0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451005,"Plane Detect | Transform",5,1) );

  current_utime_=0;
}

bool MajorPlane::getSweep( Eigen::Vector3f bounds_center, Eigen::Vector3f bounds_size){

  // get submap we created earlier
  LocalMap::Ptr localMap = mCollector->getMapManager()->getMap(mActiveMapId);

  // find time range of desired swath (from 45 to 135 degrees)
  int64_t timeMin, timeMax;
  double ang_min = 0.0 *M_PI/180; // leading edge from the right hand side of sweep
  double ang_max = 179.99 *M_PI/180;
  // 0 and 180 fails
  
  int current_utime = drc::Clock::instance()->getCurrentTime();
  //cout << ang_min << " min | " << ang_max << " max\n";
        
  bool gotFirstSweep = mCollector->getLatestSwath(ang_min, ang_max,
                                        timeMin, timeMax); // these didnt work
  if (!gotFirstSweep){ // have not properly init'ed the collector - not a full sweep yet
    // cout << "not prop init yet\n"; 
    return false;
  }
  
  if (timeMin == last_sweep_time_){ // Is the sweep the same as last time?
    // cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | repeated swath\n";
    return false; 
  }
  last_sweep_time_ = timeMin;
  cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | new process\n";

  
  LocalMap::SpaceTimeBounds bounds;
  bounds.mTimeMin = timeMin;
  bounds.mTimeMax = timeMax;
  // Also add constraints to that the points are around the robot:
  // axis aligned box thats 6x6x6m
//  bounds.mPlanes = Utils::planesFromBox(   Eigen::Vector3f(-3,-3,-3),
//                                          Eigen::Vector3f(3,3,3));
  bounds.mPlanes = Utils::planesFromBox( bounds_center - bounds_size, bounds_center + bounds_size );

  // get and publish point cloud corresponding to this time range
  // (for debugging)
  cloud_ =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
  if (verbose_lcm_ >=2){
    bot_lcmgl_t* lcmgl = mLcmGl;
    bot_lcmgl_color3f(lcmgl, 1, 0.75, 0.75);
    bot_lcmgl_point_size(lcmgl, 2); //1
    for (int i = 0; i < cloud_->size(); ++i) {
      maps::PointCloud::PointType point = (*cloud_)[i];
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
      bot_lcmgl_end(lcmgl);
    }
    bot_lcmgl_switch_buffer(lcmgl);  
  }
  
  return true;
}

Eigen::Isometry3d MajorPlane::determinePlanePose(pcl::ModelCoefficients::Ptr plane_coeffs,
          Eigen::Vector4f centroid){
  double run = sqrt(plane_coeffs->values[1]*plane_coeffs->values[1] +  plane_coeffs->values[0]*plane_coeffs->values[0]
                        +  plane_coeffs->values[2]*plane_coeffs->values[2]);
  double yaw = atan2 ( plane_coeffs->values[1] , plane_coeffs->values[0]);
  double pitch_x = acos( plane_coeffs->values[2]/ run);
//  cout << "Pitch " << (pitch_x*180/M_PI) << " | Yaw " << (yaw*180/M_PI) << "\n";

  Eigen::Isometry3d plane_pose;
  plane_pose.setIdentity();
  plane_pose.translation() << centroid(0), centroid(1), centroid(2);
  Eigen::Quaterniond quat = euler_to_quat( yaw , pitch_x , 0 );             
  plane_pose.rotate( quat );
  return plane_pose;
}




// input: plane coeffs and a point on the new plane
// output: true if the overlap, false otherwise
bool matchPlaneToPrevious(pcl::ModelCoefficients::Ptr old_plane_coeffs, pcl::ModelCoefficients::Ptr new_plane_coeffs, 
        Eigen::Vector4f new_centroid){
  // If we have already found a plane, then compare it with the new one
  double max_angle_diff = 10; // if the planes are within X degrees
  double max_project_dist = 0.3; // max distance between point on plane and previous plane
  
  double top=new_plane_coeffs->values[0]*old_plane_coeffs->values[0] + new_plane_coeffs->values[1]*old_plane_coeffs->values[1] +
          new_plane_coeffs->values[2]*old_plane_coeffs->values[2];
  double angle=acos(top)* 180.0 / M_PI;
  if (angle > 90)
    angle = 180 -angle;
  
  if (angle > max_angle_diff){
      cout << "[PLANE] the plane-to-plane angle is: " << angle << "\n";
      cout << "PLANE NOT FOUND - INTER PLANE ANGLE IS LARGE\n"; 
      return false;
  }
  
  // Project centroid onto the main plane and see if distance is small.
  // if it isnt they are on parallel but not the same plane eg two parallel walls
  double top_d = old_plane_coeffs->values[0]*new_centroid(0) +
        old_plane_coeffs->values[1]*new_centroid(1) +
        old_plane_coeffs->values[2]*new_centroid(2) +
        old_plane_coeffs->values[3];
  double dist = pow(old_plane_coeffs->values[0],2) +
  pow(old_plane_coeffs->values[1],2) + pow(old_plane_coeffs->values[2],2);
  dist = fabs(top_d)/sqrt(dist);      
  if (dist > max_project_dist){
      cout << "[PLANE]  the centroid-to-plane dist is: " << dist << "\n";
      cout << "PLANE NOT FOUND - DISTANCE BETWEEN PLANES IS LARGE\n"; 
      return false;
  }
  
  return true;
}



void MajorPlane::storeNewPlane( pcl::ModelCoefficients::Ptr new_plane_coeffs, Eigen::Vector4f new_plane_centroid,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_plane_hull ){
  

  Eigen::Isometry3d new_plane_pose = determinePlanePose(new_plane_coeffs, new_plane_centroid);
  plane_pose_ = new_plane_pose;
  plane_coeffs_ = new_plane_coeffs;
  plane_pose_init_=true;  
  
  
  // Visualise hull and normal:
  if (verbose_lcm_ >=1){
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(current_utime_, null_pose);
    pc_vis_->pose_to_lcm_from_list(4451002, null_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(4451003, *new_plane_hull, current_utime_, current_utime_);          
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normals_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    normals_cloud->points.push_back( new_plane_hull->points[0]);
    pcl::PointXYZRGB pt;
    pt.x= normals_cloud->points[0].x + 0.2*new_plane_coeffs->values[0];
    pt.y= normals_cloud->points[0].y + 0.2*new_plane_coeffs->values[1];
    pt.z= normals_cloud->points[0].z + 0.2*new_plane_coeffs->values[2];
    pt.r =0;      pt.g =255;      pt.b =0;
    normals_cloud->points.push_back( pt );
    pc_vis_->ptcld_to_lcm_from_list(4451004, *normals_cloud, current_utime_, current_utime_);          
  }  
  
  
  cout  <<  "New Plane Coefficients: " << new_plane_coeffs->values[0]
      << " " << new_plane_coeffs->values[1] << " "  << new_plane_coeffs->values[2] << " " << new_plane_coeffs->values[3] << endl;
  cout << "Centroid: " << new_plane_centroid(0) << " " << new_plane_centroid(1) << " " << new_plane_centroid(2) << "\n"; // last element held at zero

  // Visualise a pose on the plane
  Isometry3dTime plane_poseT = Isometry3dTime(current_utime_,new_plane_pose  );
  pc_vis_->pose_to_lcm_from_list(4451005, plane_poseT);    
    
  
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


void MajorPlane::matchToAllPlanes(){
  int64_t tic = _timestamp_now(); 

  
  if (cloud_->points.size() < 5000){ //20000 originally
    cout << "Cloud is too small to detect plane [" << cloud_->points.size() << "]\n";
    return;
  }
  
  // 2. Extract the major planes and send them to lcm:
  int plane_fitter_id_ =1;
  FilterPlanes filtp;
  filtp.setInputCloud(cloud_);
  filtp.setPoseIDs(plane_fitter_id_,current_utime_);
  filtp.setLCM(lcm_->getUnderlyingLCM());
  filtp.setDistanceThreshold(0.02); // simulated lidar
  filtp.setStopProportion(0.050);  //was 0.1
  filtp.setStopCloudSize(200);
  vector<BasicPlane> plane_stack; 
  filtp.filterPlanes(plane_stack);
  std::cout << "[OUT] number of planes extracted: " << plane_stack.size() << "\n";
  
  GrowCloud grow;
  grow.visualizePlanes(plane_stack, pc_vis_, 5000000 );
  
  if (1==0){ // visualize all the planes found
    std::stringstream ss;
    grow.printPlaneStackCoeffs(plane_stack, ss);
    cout << ss.str();
    
    for (size_t i=0; i <plane_stack.size() ; i++){
      cout << i << ": " << plane_stack[i].n_source_points << "\n";
    }
    for (size_t i=0; i <plane_stack.size() ; i++){
      cout << i << ": " << plane_stack[i].centroid << " centroid\n";
    }
  }

  // for each plane compare to old, if true  
  int found_match=-1;
  for (size_t i=0; i < plane_stack.size() ; i++){
    pcl::ModelCoefficients::Ptr plane_stack_coeffs(new pcl::ModelCoefficients ( plane_stack[i].coeffs   ));
    if (matchPlaneToPrevious(plane_coeffs_, plane_stack_coeffs , plane_stack[i].centroid)){
      found_match=i;
      cout << "[PLANE] Found a match at plane " << i << "\n";
      break;   
    }
  }
  
  if (found_match == -1){
    cout << "[PLANE] Didn't find a match in " << plane_stack.size() << " planes\n";
    return;
  }
  
  pcl::ModelCoefficients::Ptr new_plane_coeffs(new pcl::ModelCoefficients ( plane_stack[found_match].coeffs   ));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB> (  plane_stack[found_match].cloud ) );
  storeNewPlane( new_plane_coeffs, plane_stack[found_match].centroid , cloud_hull);

  
  cout << "[PLANE] Time to analyse planes: " << ((double)((_timestamp_now() - tic) / 1E6)) << "\n";
}




void MajorPlane::matchToLargestPlane(){
  if (cloud_->points.size() < 5000){
    cout << "Cloud is too small to detect plane [" << cloud_->points.size() << "]\n";
    return;
  }    
  
  
  pcl::ModelCoefficients::Ptr new_plane_coeffs(new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02); // was 0.01m
  seg.setInputCloud (cloud_);
  seg.segment (*inliers, *new_plane_coeffs);
  if ( inliers->indices.size () == 0){
    printf ("Could not estimate a planar model for the given dataset.\n");
    return;
  }
  
  if (1==0){
    // i think these numbers are incorrect - HPR is illdefined
    double pitch = atan(new_plane_coeffs->values[0]/new_plane_coeffs->values[2]);
    double roll =- atan(new_plane_coeffs->values[1]/new_plane_coeffs->values[2]);
    double new_plane_coeffs_norm = sqrt(pow(new_plane_coeffs->values[0],2) +
        pow(new_plane_coeffs->values[1],2) + pow(new_plane_coeffs->values[2],2));
    double height = (new_plane_coeffs->values[2]*new_plane_coeffs->values[3]) / new_plane_coeffs_norm;
    
    cout  <<  "New Plane Coefficients: " << new_plane_coeffs->values[0]
      << " " << new_plane_coeffs->values[1] << " "  << new_plane_coeffs->values[2] << " " << new_plane_coeffs->values[3] << endl;
    cout << "Pitch: " << pitch << " (" << (pitch*180/M_PI) << "d). positive nose down\n";
    cout << "Roll : " << roll << " (" << (roll*180/M_PI) << "d). positive right side down\n";
    cout << "Height : " << height << " of device off ground [m]\n";
    cout << "Total points: " << cloud_->points.size() << ". inliers: " << inliers->indices.size () << endl << endl;
  }
  
  
  
  ///// Whats below here is not necessary - just for visual output
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // c Project the model inliers (seems to be necessary to fitting convex hull
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_);
  proj.setModelCoefficients (new_plane_coeffs);
  proj.filter (*cloud_projected);
  
  std::vector <pcl::Vertices> vertices;
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud_projected);
  chull.setDimension(2);
  chull.reconstruct (*cloud_hull,vertices);
  if (cloud_hull->points.size () ==0){
    cout <<"ERROR: CONVEX HULL HAS NO POINTS! - NEED TO RESOLVE THIS\n"; 
  }
        
  Eigen::Vector4f centroid;
  compute3DCentroid (*cloud_projected,centroid);

  cout  <<  "Plane Coefficients: " << new_plane_coeffs->values[0]
    << " " << new_plane_coeffs->values[1] << " "  << new_plane_coeffs->values[2] << " " << new_plane_coeffs->values[3] << endl;
  
  
  if (!matchPlaneToPrevious(plane_coeffs_,  new_plane_coeffs, centroid)){
    return;
  }
  
  storeNewPlane( new_plane_coeffs, centroid , cloud_hull);
}


bool MajorPlane::trackPlane( Eigen::Isometry3d &plane_pose , int64_t current_utime  ){
  current_utime_ = current_utime;
  
  if (plane_pose_init_){
    //matchToLargestPlane(); 
    matchToAllPlanes();
  }else{
    cout << "[Plane] Init Plane hasn't been set, refusing to track it\n";
  }
  
  
  plane_pose = plane_pose_;
  return plane_pose_init_;
}