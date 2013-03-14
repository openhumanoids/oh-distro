// simple tracker in color space
// 
// position (and orientation) of affordance
// a plane of interest - currently largest
// relative offset between plane and object
// color of object
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>

/// MAPS:
#include <maps/SensorDataReceiver.hpp>
#include <maps/MapManager.hpp>
#include <maps/LocalMap.hpp>
#include <maps/Collector.hpp>
#include <maps/BotWrapper.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/PointCloudView.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <particle/particle_filter.hpp>

#include <drc_utils/Clock.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

using namespace cv;
using namespace std;
using namespace maps;

int mode =0;
int vis_offset =0;

class State {
public:
  boost::shared_ptr<lcm::LCM> mLcm;
  BotWrapper::Ptr mBotWrapper;
  boost::shared_ptr<Collector> mCollector;
  int mActiveMapId;
  bot_lcmgl_t* mLcmGl;
  
  State( boost::shared_ptr<lcm::LCM> &mLcm ): mLcm(mLcm) {
    mBotWrapper.reset(new BotWrapper(mLcm));
    mCollector.reset(new Collector());
    mCollector->setBotWrapper(mBotWrapper);
    mActiveMapId = 0;
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "test-points");
    drc::Clock::instance()->setLcm(mLcm);
  }
  
  ~State() {
    bot_lcmgl_destroy(mLcmGl);
  }
};


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
      int num_particles_, State* iState);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    std::string image_channel_;

    void propogatePF();
    void evaluateLikelihood( std::vector<float> &loglikelihoods );
    void updatePF( std::vector<float> &loglikelihoods );

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;

    // Camera Params:
    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;
    
    int counter_;

    pointcloud_vis* pc_vis_;
    image_io_utils*  imgutils_;
    
    ParticleFilter* pf; 
    int num_particles_;
  
    bot_core::image_t img_;  
    bot_core::image_t last_img_;    
       
    // Plane Detection:
    int64_t last_sweep_time_;
    bool getSweep();
    // Point Cloud of most recent sweep:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;    
    bool findPlane();
    // pose of a point on the plane with pitch and yaw but roll =0
    Eigen::Isometry3d plane_pose_ ;
    // has the above value been set?
    bool plane_pose_set_;
  protected:
    State* mState;
    
    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
    int num_particles_, State* iState): lcm_(lcm_), image_channel_(image_channel_), 
    num_particles_(num_particles_), mState(iState){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  vis_offset = mode*100;
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451000,"Tracker | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451001,"Tracker | Particles"           ,1,1, 4451000,1, { 0.0, 1.0, 0.0} ));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451002,"Tracker | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451003,"Tracker | Plane"           ,3,1, 4451002,1, { 1.0, 0.0, 0.0} ));
  
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Tracker | Plane X"           ,1,1, 4451002,1, { 0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451005,"Tracker | Transform",5,1) );

  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451006 +vis_offset,"Tracker | Poses",5,1) );
  
  
  std::string key_prefix_str = "cameras."+ image_channel_ +".intrinsic_cal";
  width_ = bot_param_get_int_or_fail(botparam_, (key_prefix_str+".width").c_str());
  height_ = bot_param_get_int_or_fail(botparam_,(key_prefix_str+".height").c_str());
  fx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fx").c_str());
  fy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fy").c_str());
  cx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cx").c_str());
  cy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cy").c_str());    
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_ );

  // Init Particle Filter:
  int rng_seed = 1;
  double resample_threshold =0.5;
  std::vector<double> initial_var  { .01  ,.01  , .001   , .001 };
  Eigen::Isometry3d init_pose;
  init_pose.setIdentity();
  if (mode==0){
    init_pose.translation() << -6.5, 0.0, 1.2;
  }else if(mode==1){
    cout << "add mode\n";
    init_pose.translation() << -7.0, -0.75, 1.3;
  }
  pf = new ParticleFilter(lcm_->getUnderlyingLCM(), num_particles_,init_pose,
            initial_var, rng_seed,resample_threshold);  
  
  counter_=0;
  img_.utime=0; // used to indicate no message recieved yet
  last_img_.utime=0; // used to indicate no message recieved yet
  
  plane_pose_.setIdentity();
  plane_pose_set_ = false;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud_ = cloud_ptr;    
  
}


// Convert the image into an HSV image
IplImage* GetThresholdedImage(IplImage* img){ 
  IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
  cvCvtColor(img, imgHSV, CV_RGB2HSV); // NB: conversion from typical LCM to HSV
  IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

  //GIMP        H = 0-360, S = 0-100 and V = 0-100. But 
  //OpenCV uses H: 0 - 180, S: 0 - 255, V: 0 - 255
  //OpenCV uses BGR format, not RGB
  // conversion from GIMP to OpenCV:  H/2  //S*2.55  //V*2.55  
  // Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
  //cvInRangeS(imgHSV, cvScalar(112, 100, 100), cvScalar(124, 255, 255), imgThreshed);
  // Yellow:
  //cvInRangeS(imgHSV, cvScalar(0, 100, 100), cvScalar(30, 255, 255), imgThreshed);
  // Orange (tropicana:
  //cvInRangeS(imgHSV, cvScalar(10, 50, 50), cvScalar(15, 255, 255), imgThreshed);
  // red bowl:
  //  cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(8, 255, 255), imgThreshed);
  // green (top of lemon juice):
  //cvInRangeS(imgHSV, cvScalar(55, 50, 50), cvScalar(65, 255, 255), imgThreshed);
  
  if(mode==0){
    // red valve in VRC:
    cvInRangeS(imgHSV, cvScalar(0, 100, 1), cvScalar(5, 255, 60), imgThreshed);
  }else if (mode==1){
    // green wheel in VRC:
    cvInRangeS(imgHSV, cvScalar(55, 50, 50), cvScalar(65, 255, 255), imgThreshed);
   cout << "color\n"; 
  }

  cvReleaseImage(&imgHSV);
  return imgThreshed;
}



bool Pass::getSweep(){
  
  // get submap we created earlier
  LocalMap::Ptr localMap =
    mState->mCollector->getMapManager()->getMap(mState->mActiveMapId);

  // find time range of desired swath (from 45 to 135 degrees)
  int64_t timeMin, timeMax;
  double ang_min = 5.0 *M_PI/180; // leading edge from the right hand side of sweep
  double ang_max = 175.0 *M_PI/180;
  // 0 and 180 fails
  
  int current_utime = drc::Clock::instance()->getCurrentTime();
  //cout << ang_min << " min | " << ang_max << " max\n";
        
  mState->mCollector->getLatestSwath(ang_min, ang_max,
                                        timeMin, timeMax); // these didnt work
  if (timeMin == last_sweep_time_){
    // cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | repeat\n";
    return false; 
  }
  last_sweep_time_ = timeMin;

  cout << timeMin << " timeMin | " << timeMax << " timeMax | " << current_utime << " utime | process\n";
  LocalMap::SpaceTimeBounds bounds;
  bounds.mTimeMin = timeMin;
  bounds.mTimeMax = timeMax;

  // get and publish point cloud corresponding to this time range
  // (for debugging)
  cloud_ =     localMap->getAsPointCloud(0, bounds)->getPointCloud();
  bot_lcmgl_t* lcmgl = mState->mLcmGl;
  bot_lcmgl_color3f(lcmgl, 1, 0.75, 0.75);
  bot_lcmgl_point_size(lcmgl, 1);
  for (int i = 0; i < cloud_->size(); ++i) {
    maps::PointCloud::PointType point = (*cloud_)[i];
    bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
    bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
    bot_lcmgl_end(lcmgl);
  }
  bot_lcmgl_switch_buffer(lcmgl);  

  return true;
}


bool Pass::findPlane(){

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
    return -1;
  }
  

  if (1==0){
    // i think these numbers are incorrect:
    double pitch = atan(coeff->values[0]/coeff->values[2]);
    double roll =- atan(coeff->values[1]/coeff->values[2]);
    double coeff_norm = sqrt(pow(coeff->values[0],2) +
        pow(coeff->values[1],2) + pow(coeff->values[2],2));
    double height = (coeff->values[2]*coeff->values[3]) / coeff_norm;
    
    cout  <<  "\nRANSAC Floor Coefficients: " << coeff->values[0]
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
        
  // Visualise hull:
  Eigen::Isometry3d null_pose;
  null_pose.setIdentity();
  Isometry3dTime null_poseT = Isometry3dTime(img_.utime, null_pose);
  pc_vis_->pose_to_lcm_from_list(4451002, null_poseT);  
  pc_vis_->ptcld_to_lcm_from_list(4451003, *cloud_hull, img_.utime, img_.utime);          
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr normals_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  normals_cloud->points.push_back( cloud_hull->points[0]);
  
  pcl::PointXYZRGB pt;
  pt.x= normals_cloud->points[0].x + coeff->values[0];
  pt.y= normals_cloud->points[0].y + coeff->values[1];
  pt.z= normals_cloud->points[0].z + coeff->values[2];
  pt.r =0;      pt.g =255;      pt.b =0;
  normals_cloud->points.push_back( pt );
  stringstream name_out2;
  int plane_id2 = 11212 ;
  ptcld_cfg pcfg2 = ptcld_cfg(plane_id2,   "Normal"    ,3,1, 4451002,1,{0.3,.1,0.1} );
  pc_vis_->ptcld_to_lcm(pcfg2, *normals_cloud, img_.utime, img_.utime );   
  

    
    
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
  
  pc_vis_->ptcld_to_lcm_from_list(4451004, *cloud_, img_.utime, img_.utime);          

  
  plane_pose_ = isometryFloatToDouble(plane_pose_f);
  Isometry3dTime plane_poseT = Isometry3dTime(img_.utime,plane_pose_  );
  pc_vis_->pose_to_lcm_from_list(4451005, plane_poseT);    
  
  plane_pose_set_=true;
         
  
  
}





void Pass::evaluateLikelihood( std::vector<float> &loglikelihoods ){
  if (img_.utime==0){     return;   } // if no msg recieved then ignore output command
    
  Mat src= Mat::zeros( img_.height,img_.width  ,CV_8UC3);
  src.data = img_.data.data();
  IplImage* frame = new IplImage(src);

  // 1. Threshold the image in HSV space (object = white, rest = black)
  IplImage* imgColorThresh = GetThresholdedImage(frame);
  // Calculate the moments to estimate the position of the ball
  CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
  cvMoments(imgColorThresh, moments, 1);
  // The actual moment values
  double moment10 = cvGetSpatialMoment(moments, 1, 0);
  double moment01 = cvGetSpatialMoment(moments, 0, 1);
  double area = cvGetCentralMoment(moments, 0, 0);
  // Holding the last and current ball positions
  int u_estimated = moment10/area;
  int v_estimated = moment01/area;

  // Print it out for debugging purposes
  cout << "est: " << u_estimated << " " << v_estimated << " | area: " << area << "\n";
  double min_area = 0; // 30 worked previous for real data
  if ((area> min_area) && (u_estimated>0 && v_estimated>0)){
    // Valid measurement
  }else{
   cout << "green not seen - returning ["<< area <<"]\n"; 
   return;
  }

  // 2. Project particles into camera frame:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (size_t i=0; i<num_particles_; i++) {
    pf_state particle_state;
    particle_state =pf->GetParticleState(i);
    Eigen::Vector3d t(particle_state.pose.translation());
    pcl::PointXYZRGB pt1;
    pt1.x = t.x();       pt1.y = t.y();    pt1.z = t.z();  
    pt1.r = 255; pt1.g = 0; pt1.b =0;
    pts->points.push_back(pt1);
  }  
  Eigen::Isometry3d local_to_camera;
  frames_cpp_->get_trans_with_utime( botframes_ , "local", "CAMERA"  , img_.utime, local_to_camera);
  Eigen::Isometry3f pose_f = isometryDoubleToFloat(local_to_camera);
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::transformPointCloud (*pts, *pts,
      pose_f.translation(), pose_quat);  
  
  // 3. Determine Likelihood in Image space:
  for (size_t i=0; i< pts->points.size(); i++) {
    // u = pt.x fx/pt.z   ... project point to pixel
    pcl::PointXYZRGB pt1 = pts->points[i];
    int u = floor( ((pt1.x * fx_)/pt1.z) + cx_);
    int v = floor( ((pt1.y * fy_)/pt1.z) + cy_);
    int dist = sqrt( pow( u - u_estimated ,2) + pow( v - v_estimated ,2) );
    // Crude Binary Likelihood:
    if (dist < 10){
      loglikelihoods[i] =0.5; //was 1
    }
  }    
  
  
  // Debug Output:
  if (1==0){
    // Visualise projected points (in a camera frame)
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(img_.utime, null_pose);
    pc_vis_->pose_to_lcm_from_list(4451000, null_poseT);  
    pc_vis_->ptcld_to_lcm_from_list(4451001, *pts, img_.utime, img_.utime);  
  }
    
  imgutils_->sendImage( (uint8_t*) imgColorThresh->imageData, img_.utime, img_.width, 
                        img_.height, 1, string(image_channel_ + "_THRESH")  );
  cv::Mat imgMat(frame);
  for (size_t i=0; i< pts->points.size(); i++) {
    pcl::PointXYZRGB pt1 = pts->points[i];
    int u = floor( ((pt1.x * fx_)/pt1.z)  +  512.5);
    int v = floor( ((pt1.y * fy_)/pt1.z) + 272.5);
    Point center( u, v );
    circle( imgMat, center, 3, Scalar(0,255,0), -1, 8, 0 );
  }  
  Point center( u_estimated, v_estimated );
  circle( imgMat, center, 3, Scalar(255,0,0), -1, 8, 0 );
  imgutils_->sendImage(imgMat.data, img_.utime, img_.width, 
                        img_.height, 3, string(image_channel_ + "_TRACKING")  );
}


void Pass::propogatePF(){
  std::vector <double> success_var { .0001,.0001, .000001, .000001 }; // not used
  std::vector <double> failure_var { .00001 ,.00001 , .000001  , .000001 }; // made lower
  
  double elapsed_time = 0.1;
  double msg_dt = (double) (img_.utime - last_img_.utime)/1E6;
  if ( fabs( msg_dt) < 0.5){ // avoid odd delta times
    elapsed_time = msg_dt;
  }
  //cout << elapsed_time << "\n";

  pf_state odom_diff;
  odom_diff.pose.setIdentity();
  odom_diff.velocity.setIdentity();
  pf->MoveParticles(odom_diff,failure_var,elapsed_time,1); //failed motion estimation
  
  // Apply a constraint onto the XY plane at 0,0,0
  // with freedom of rotation in that plane
  //  std::vector <double> xyzypr{ -6.0, 0, 0., 0., 0., 0.};
  //  std::vector <bool> set_xyzypr{ 1, 0, 0, 1, 1, 0};
  //  pf->SetState(xyzypr, set_xyzypr);
  
  if ( plane_pose_set_ ){
    vector < pf_state > pfs;
    for (size_t i=0; i<num_particles_; i++) {
      pfs.push_back( pf->GetParticleState(i)   );
    }
      
    for (size_t i=0; i<num_particles_; i++) {
      pfs[i].pose = plane_pose_.inverse() * pfs[i].pose ;
    }  
    

    std::vector <double> xyzypr{ 0, 0, 0.25, 0., 0., 0.};
    std::vector <bool> set_xyzypr{ 0, 0, 1, 0, 1, 1};
    if (mode ==0){
      xyzypr = { 0, 0, 0.25, 0., 0., 0.};
      set_xyzypr= { 0, 0, 1, 0, 1, 1};
    }else if(mode ==1){
      xyzypr = { 0, 0, 0.25, 0., 0., 0.};
      set_xyzypr= { 0, 0, 1, 0, 1, 1};
    }
    
    for(int i = 0; i < num_particles_ ; ++i) {
      double current_ypr[3];
      quat_to_euler(  Eigen::Quaterniond( pfs[i].pose.rotation()) , current_ypr[0], current_ypr[1], current_ypr[2]);
      if (set_xyzypr[3]){ current_ypr[0] = xyzypr[3]; }
      if (set_xyzypr[4]){ current_ypr[1] = xyzypr[4]; }
      if (set_xyzypr[5]){ current_ypr[2] = xyzypr[5]; }
      Eigen::Quaterniond revised_quat = euler_to_quat( current_ypr[0], current_ypr[1], current_ypr[2]);             
      
      Eigen::Isometry3d ipose;
      ipose.setIdentity();
      ipose.translation() << pfs[i].pose.translation();
      if (set_xyzypr[0]){ ipose.translation().x() = xyzypr[0]; }
      if (set_xyzypr[1]){ ipose.translation().y() = xyzypr[1]; }
      if (set_xyzypr[2]){ ipose.translation().z() = xyzypr[2]; }

      ipose.rotate(revised_quat);
      pfs[i].pose = ipose;
    }  
    
    
    for (size_t i=0; i<num_particles_; i++) {
      pfs[i].pose = plane_pose_ * pfs[i].pose;
    }  
    
    for (size_t i=0; i<num_particles_; i++) {
      pf->SetParticleState(i, pfs[i]);
    }  
    
    
    vector < Isometry3dTime > pf_poses;
    for (size_t i=0; i<num_particles_; i++) {
      pf_poses.push_back(   Isometry3dTime ( img_.utime+i, pfs[i].pose )    );
    }  
    pc_vis_->pose_collection_to_lcm_from_list(4451006 + vis_offset, pf_poses);
    
  }
  
}


void Pass::updatePF( std::vector<float> &loglikelihoods ){
  pf->LogLikelihoodParticles(loglikelihoods);
  pf->SendParticlesLCM( img_.utime ,0);//vo_estimate_status);

  double ESS = pf->ConsiderResample();
  std::cerr << "                              "<< ESS/num_particles_ << " ESS | " << img_.utime  << " utime\n";  
}


void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "returning cowardly\n";
    return;
  }
  img_= *msg;  
  
  if (getSweep()){
    findPlane();
  }
  
  std::vector<float> loglikelihoods;
  loglikelihoods.assign (num_particles_,0);    
  propogatePF();
  evaluateLikelihood(loglikelihoods);
  updatePF(loglikelihoods);
  
  last_img_ = img_;
}


int main(int argc, char ** argv) {
  string channel = "CAMERALEFT";
  int num_particles = 100;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
  opt.add(num_particles, "n", "num_particles","num particles");
  opt.add(mode, "m", "mode","Mode");
  opt.parse();
  std::cout << "channel: " << channel << "\n";    

  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  
  // create state object instance
  State state(lcm);
  // create new submap
  LocalMap::Spec mapSpec;
  mapSpec.mId = 1;
  mapSpec.mPointBufferSize = 5000;
  mapSpec.mActive = true;
  mapSpec.mBoundMin = Eigen::Vector3f(-1,-1,-1)*10;
  mapSpec.mBoundMax = Eigen::Vector3f(1,1,1)*10;
  mapSpec.mResolution = 0.01;
  state.mActiveMapId = state.mCollector->getMapManager()->createMap(mapSpec);
  // start running wrapper
  std::string laserChannel("SCAN");
  state.mCollector->getDataReceiver()->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");
  state.mCollector->start();  
  
  
  Pass app(lcm, channel, num_particles, &state);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}