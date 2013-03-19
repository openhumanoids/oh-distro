#include "major-plane-detect.hpp"
#include <iostream>

using namespace maps;
using namespace std;

MajorPlane::MajorPlane(boost::shared_ptr<lcm::LCM> &lcm_, int verbose_lcm_): lcm_(lcm_), verbose_lcm_(verbose_lcm_){
    mBotWrapper.reset(new BotWrapper(lcm_));
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mActiveMapId = 0;
    mLcmGl = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "test-points");
    drc::Clock::instance()->setLcm(lcm_);  



  // create new submap
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*10;
  mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*10;
  mapSpec.mResolution = 0.01;
  mActiveMapId = mCollector->getMapManager()->createMap(mapSpec);
  // start running wrapper
  std::string laserChannel("SCAN");
  mCollector->getDataReceiver()->addChannel(laserChannel, SensorDataReceiver::SensorTypePlanarLidar, laserChannel, "local");
  mCollector->start();  


  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451002,"Plane Detect | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451003,"Plane Detect | Plane"           ,3,1, 4451002,1, { 1.0, 0.0, 0.0} ));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Plane Detect | Plane Normal"           ,3,1, 4451002,1, { 0.3, 0.1, 0.1} ));
  //pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Plane Detect | Plane X"           ,1,1, 4451002,1, { 0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451005,"Plane Detect | Transform",5,1) );

  current_utime_=0;
}

bool MajorPlane::getSweep(){

  // get submap we created earlier
  LocalMap::Ptr localMap = mCollector->getMapManager()->getMap(mActiveMapId);

  // find time range of desired swath (from 45 to 135 degrees)
  int64_t timeMin, timeMax;
  double ang_min = 5.0 *M_PI/180; // leading edge from the right hand side of sweep
  double ang_max = 175.0 *M_PI/180;
  // 0 and 180 fails
  
  int current_utime = drc::Clock::instance()->getCurrentTime();
  //cout << ang_min << " min | " << ang_max << " max\n";
        
  mCollector->getLatestSwath(ang_min, ang_max,
                                        timeMin, timeMax); // these didnt work
  if (timeMin == last_sweep_time_){
    // cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | repeated swath\n";
    return false; 
  }
  last_sweep_time_ = timeMin;

  cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | new process\n";
  LocalMap::SpaceTimeBounds bounds;
  bounds.mTimeMin = timeMin;
  bounds.mTimeMax = timeMax;

  // get and publish point cloud corresponding to this time range
  // (for debugging)
  cloud_ =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
  if (verbose_lcm_ >=2){
    bot_lcmgl_t* lcmgl = mLcmGl;
    bot_lcmgl_color3f(lcmgl, 1, 0.75, 0.75);
    bot_lcmgl_point_size(lcmgl, 1);
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


void MajorPlane::findPlane(){
  if (cloud_->points.size() < 20000){
    cout << "Cloud is too small to detect plane [" << cloud_->points.size() << "]\n";
    return;
  }    
  

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01); // was 0.01m
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ> (cloudfloor));
  seg.setInputCloud (cloud_);
  seg.segment (*inliers, *coeff);
  if ( inliers->indices.size () == 0)
  {
    printf ("Could not estimate a planar model for the given dataset.\n");
    return;
  }
  

  if (1==1){
    // i think these numbers are incorrect:
    double pitch = atan(coeff->values[0]/coeff->values[2]);
    double roll =- atan(coeff->values[1]/coeff->values[2]);
    double coeff_norm = sqrt(pow(coeff->values[0],2) +
        pow(coeff->values[1],2) + pow(coeff->values[2],2));
    double height = (coeff->values[2]*coeff->values[3]) / coeff_norm;
    
    cout  <<  "New RANSAC Floor Coefficients: " << coeff->values[0]
      << " " << coeff->values[1] << " "  << coeff->values[2] << " " << coeff->values[3] << endl;
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
  proj.setModelCoefficients (coeff);
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
        
  // Visualise hull and normal:
  if (verbose_lcm_ >=1){
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(current_utime_, null_pose);
    pc_vis_->pose_to_lcm_from_list(4451002, null_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(4451003, *cloud_hull, current_utime_, current_utime_);          
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normals_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    normals_cloud->points.push_back( cloud_hull->points[0]);
    pcl::PointXYZRGB pt;
    pt.x= normals_cloud->points[0].x + coeff->values[0];
    pt.y= normals_cloud->points[0].y + coeff->values[1];
    pt.z= normals_cloud->points[0].z + coeff->values[2];
    pt.r =0;      pt.g =255;      pt.b =0;
    normals_cloud->points.push_back( pt );
    int plane_id2 = 11212 ;
    ptcld_cfg pcfg2 = ptcld_cfg(plane_id2,   "Normal"    ,3,1, 4451002,1,{0.3,.1,0.1} );
    pc_vis_->ptcld_to_lcm_from_list(4451004, *normals_cloud, current_utime_, current_utime_);          
  }

    
    
  Eigen::Isometry3f plane_pose_f;
  plane_pose_f.setIdentity();
  double run = sqrt(coeff->values[1]*coeff->values[1] +  coeff->values[0]*coeff->values[0]
                        +  coeff->values[2]*coeff->values[2]);
  double yaw = atan2 ( coeff->values[1] , coeff->values[0]);
  double pitch_x = acos( coeff->values[2]/ run);
  Eigen::Quaternionf quat = euler_to_quat_f( yaw , pitch_x , 0 );             
  plane_pose_f.rotate(    quat );
    
  plane_pose_f.translation() << centroid(0), centroid(1), centroid(2);
  pcl::transformPointCloud (*cloud_, *cloud_, plane_pose_f.inverse());
  //pc_vis_->ptcld_to_lcm_from_list(4451004, *cloud_, current_utime_, current_utime_);          

  
  plane_pose_ = isometryFloatToDouble(plane_pose_f);
  Isometry3dTime plane_poseT = Isometry3dTime(current_utime_,plane_pose_  );
  pc_vis_->pose_to_lcm_from_list(4451005, plane_poseT);    
  
  plane_pose_set_=true;
}


bool MajorPlane::getPlane( Eigen::Isometry3d &plane_pose , int64_t current_utime  ){
  current_utime_ = current_utime;
  if (getSweep()){
    findPlane();
  }
  
  plane_pose = plane_pose_;
  return plane_pose_set_;
}
