// Red Valve:
// object-tracker  -a 0 -c -p -l 0
// fails with hist tracker. why?

/// Hist tracker:
// Duff Can:
// object-tracker  -a 2 -g -p -l 1
// Green Valve
// object-tracker  -a 1 -g -p -l 0


// color threshold and plane are hard coded
// hist tracker fails if mask is not entirely inside the image
// particle-to-uv incorrectly functions if facing in wrong direction from tracker (pixels projected into mirror positons)
// determine the projection between the object and the plane or have it provided explicitly by user
// need to have plane affordances published
// have plane tracker drill down into X planes rather than just 1


// TODO:
// 
// position (and orientation) of affordance
// a plane of interest - currently largest
// relative offset between plane and object
// color of object (find automatically using mask)

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <boost/config/no_tr1/complex.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>
#include <ConciseArgs>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <particle/particle_filter.hpp>
#include <trackers/major-plane-detect.hpp>
#include <trackers/color-tracker.hpp>
#include <trackers/histogram-tracker.hpp>
#include <trackers/icp-tracker.hpp>

#include <affordance/AffordanceUtils.hpp>

using namespace std;

// offset of affordance in mask labelling:
#define AFFORDANCE_OFFSET 64


int affordance_vis_offset =0;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, 
         int num_particles_, int tracker_instance_id_, int verbose_);
    
    ~Pass(){
    }    
    
    void initiate_tracking(int affordance_id, bool use_color_tracker, bool use_histogram_tracker, 
                       bool use_icp_tracker, bool use_plane_tracker, int plane_affordance_id);
    
  private:
    int verbose_;
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    void affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);
    void trackerCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::tracker_command_t* msg);    
    std::string mask_channel_, image_channel_;
    
    void updatePF();
    void propogatePF();
    void colorTrackerLikelihood( std::vector<float> &loglikelihoods );
    void histogramTrackerLikelihood( std::vector<float> &loglikelihoods );
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;
    pointcloud_vis* pc_vis_;
    image_io_utils*  imgutils_;

    // unique id of this tracker instance. Plane=0, otherwise above that
    int tracker_instance_id_;    

    // Camera Params:
    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;
    bot_core::image_t img_;  
    bot_core::image_t last_mask_;    
    
    // Current Camera and Head Pose:
    Eigen::Isometry3d local_to_camera_;
    Eigen::Isometry3d head_to_local_;
    
    // Tracker State Engine:
    bool tracker_initiated_;
    bool got_initial_affordance_;    
    bool got_mask_;
    
    // Tracker Configuration:
    int affordance_id_; // id of the affordance we want to track
    int affordance_vis_; // id for pc_vis
    ColorTracker* color_tracker_;
    HistogramTracker* histogram_tracker_;
    ICPTracker* icp_tracker_;    
    bool use_color_tracker_;
    bool use_histogram_tracker_;
    bool use_icp_tracker_;
    
    // Plane Tracking Variables:
    int plane_affordance_id_;
    MajorPlane* major_plane_;    
    bool use_plane_tracker_; // Will Plane Tracking be provided from another source?
    Eigen::Isometry3d plane_pose_ ; // Current status of plane estimation:
    pcl::ModelCoefficients::Ptr plane_coeffs_;

    bool plane_pose_set_;
    // The relative position of the plane and which dimensions to constrain:
    std::vector <double> plane_relative_xyzypr_; 
    std::vector <bool> plane_relative_xyzypr_set_;
    
    // Particle Filter Variables:
    ParticleFilter* pf_; 
    int num_particles_;
    std::vector<double> pf_initial_var_;
    std::vector <double> pf_drift_var_;    
    
    // The pose of the Mean of the particle filter
    Eigen::Isometry3d object_pose_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_bb_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_;
    
    drc::affordance_t last_affordance_msg_; // The last update of the affordance msg:
    void publishUpdatedAffordance();
    AffordanceUtils affutils;    
    
    int counter_;    
    int64_t last_utime_;    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
    int num_particles_, int tracker_instance_id_, int verbose_): 
    lcm_(lcm_), image_channel_(image_channel_), 
    num_particles_(num_particles_), tracker_instance_id_(tracker_instance_id_), verbose_(verbose_){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  mask_channel_="CAMERALEFT_MASKZIPPED";
  lcm_->subscribe( mask_channel_ ,&Pass::maskHandler,this);
  lcm_->subscribe("AFFORDANCE_PLUS_COLLECTION",&Pass::affordancePlusHandler,this); 
  lcm_->subscribe("TRACKER_COMMAND",&Pass::trackerCommandHandler,this);   
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Tracker | Pose - Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Tracker | New Sweep"     ,1,1, 1000,1, {0,0,1}));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1020,"Tracker | Updated Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1021,"Tracker | Aff Cloud [at Updated]"     ,1,1, 1020,1, {0,0.6,0}));
  
  
  
  std::string key_prefix_str = "cameras."+ image_channel_ +".intrinsic_cal";
  width_ = bot_param_get_int_or_fail(botparam_, (key_prefix_str+".width").c_str());
  height_ = bot_param_get_int_or_fail(botparam_,(key_prefix_str+".height").c_str());
  fx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fx").c_str());
  fy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".fy").c_str());
  cx_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cx").c_str());
  cy_ = bot_param_get_double_or_fail(botparam_, (key_prefix_str+".cy").c_str());    

  // Initialize Particle Filter:
  int rng_seed = 1;
  double resample_threshold =0.5;
  pf_initial_var_ =  { .001  ,.001  , .0001   , .0001 };
  if (1==0){ // use this for testing
    pf_drift_var_ = { .0001 ,.0001 , .00001  , .00001 };
  }else{
    pf_drift_var_ = { .00001 ,.00001 , .000001  , .000001 }; // made lower
  }
  Eigen::Isometry3d init_pose;
  init_pose.setIdentity();
  pf_ = new ParticleFilter(lcm_->getUnderlyingLCM(), num_particles_,init_pose,
            pf_initial_var_, rng_seed,resample_threshold);  
  
  counter_=0;
  last_utime_=0;

  // Image Masking:
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_);
    
  // Color Tracking:
  color_tracker_ = new ColorTracker(lcm_, width_, height_, fx_, fy_, cx_, cy_);
  // Histogram Tracking:  
  histogram_tracker_ = new HistogramTracker();
  // ICP Tracking:
  icp_tracker_ = new ICPTracker(lcm_, 0);
  pcl::ModelCoefficients::Ptr plane_coeffs_ptr_(new pcl::ModelCoefficients ());
  plane_coeffs_ = plane_coeffs_ptr_;
  
  // Plane Detection:
  major_plane_ = new MajorPlane( lcm_, 2);
  plane_pose_.setIdentity();
  plane_pose_set_ = false;
  
  // State Engine:
  got_mask_ = false;
  got_initial_affordance_ = false;
  tracker_initiated_ = false; // by default start uninitiated  
}

void Pass::propogatePF(){
  
  double elapsed_time = 0.1;
  double msg_dt = (double) (img_.utime - last_utime_)/1E6;
  if ( fabs( msg_dt) < 0.5){ // avoid odd delta times
    elapsed_time = msg_dt;
  }
  //cout << elapsed_time << "\n";

  pf_state odom_diff;
  odom_diff.pose.setIdentity();
  odom_diff.velocity.setIdentity();
  pf_->MoveParticles(odom_diff,pf_drift_var_,elapsed_time,1); //the 1 signifies failed motion estimation
  
  if (use_plane_tracker_){
    if ( plane_pose_set_ ){
      pf_->applyPlaneConstraint(plane_relative_xyzypr_, plane_relative_xyzypr_set_, plane_pose_);
    }  
  }
  
  
  vector < Isometry3dTime > pf_poses;
  for (size_t i=0; i<num_particles_; i++) {
    pf_state pf = pf_->GetParticleState(i)   ;
    pf_poses.push_back(   Isometry3dTime ( img_.utime+i, pf.pose )    );
  }  
  pc_vis_->pose_collection_to_lcm_from_list(affordance_vis_ + 3, pf_poses);
}


void Pass::colorTrackerLikelihood( std::vector<float> &loglikelihoods ){
  std::vector< Eigen::Vector3d > pts;
  for (size_t i=0; i<num_particles_; i++) {
    pf_state particle_state;
    particle_state =pf_->GetParticleState(i);
    Eigen::Vector3d t(particle_state.pose.translation());
    pts.push_back(t);
  }    
  loglikelihoods = color_tracker_->doColorTracker(pts, img_.data.data(), local_to_camera_, img_.utime);
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void Pass::histogramTrackerLikelihood( std::vector<float> &loglikelihoods ){
  cv::Mat img = cv::Mat( height_ , width_ , CV_8UC3, img_.data.data());
  cv::cvtColor(img, img, CV_RGB2BGR);
  int mask_id = AFFORDANCE_OFFSET + affordance_id_;
  float scale = 1.f;
  
  if (!histogram_tracker_->getMaskInitialized() ){
    cout << "Initializing Mask...\n";
    uint8_t* data = (uint8_t*) imgutils_->unzipImage(&last_mask_);
    cv::Mat aff_img = cv::Mat( height_ , width_ , CV_8UC1, data);
    cv::Mat1b mask = (aff_img == mask_id);
    histogram_tracker_->initialize(img, mask);
  } 

  if (histogram_tracker_->getMaskInitialized() ){
    std::vector< Eigen::Vector3d > pts;
    for (size_t i=0; i<num_particles_; i++) {
      pf_state particle_state;
      particle_state =pf_->GetParticleState(i);
      Eigen::Vector3d t(particle_state.pose.translation());
      pts.push_back(t);
    }        
      
    double tic = _timestamp_now(); 
    loglikelihoods = histogram_tracker_->update(img, scale, pts, local_to_camera_);
    if (counter_%10 ==0){ 
        printf("===> HISTOGRAM BACKPROJECTION: %4.2f ms\n", (_timestamp_now() - tic) * 1e-3); 
    }
    cv::cvtColor(img, img, CV_BGR2RGB);
    imgutils_->sendImage( (uint8_t*) img.data, img_.utime, width_, height_, 3, "TRACKER_HIST"  );
  }
}



void Pass::updatePF(){
  propogatePF();
  
  // Likelihood Functions:
  std::vector<float> loglikelihoods;
  loglikelihoods.assign ( num_particles_ ,0);    
  if (use_color_tracker_)
    colorTrackerLikelihood(loglikelihoods);
  if (use_histogram_tracker_)
    histogramTrackerLikelihood(loglikelihoods);
  
  pf_->LogLikelihoodParticles(loglikelihoods);
  
  // Update the pose and publish the updated affordance:
  object_pose_ = pf_->IntegratePose();
  
  //pf_->SendParticlesLCM( img_.utime ,0);//vo_estimate_status);
  double ESS = pf_->ConsiderResample();
  if(verbose_ >= 1){
    std::cerr << ESS/num_particles_ << " ESS | " << img_.utime  << " utime\n";  
  }
}

void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "Incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "Refusing to process...\n";
    return;
  }
  if (!tracker_initiated_){
    // haven't been initated, quit ...
    return;
  }
  img_= *msg;  
  int64_t update_time = msg->utime;
  
  // Update the camera and head poses
  frames_cpp_->get_trans_with_utime( botframes_ , "local", "CAMERA"  , img_.utime, local_to_camera_);
  frames_cpp_->get_trans_with_utime( botframes_ , "head", "local"  , img_.utime, head_to_local_);
  
  
  if (use_icp_tracker_){
    
    // Ask the maps collector for a sweep :
    // true if we got a new sweep, make a box 5x5x5m centered around the robot's head
    // Update: this is where plane tracking used to take place:
    if (major_plane_->getSweep(head_to_local_.cast<float>().translation() ,  Eigen::Vector3f( 2., 2., 2.)) ){ 
      // was:
      //plane_pose_set_ = major_plane_->trackPlane(plane_pose_, msg->utime);  
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud;
      if (use_plane_tracker_){
        new_cloud = major_plane_->getSweepCloudWithoutPlane(0.03,plane_coeffs_);      
      }else{
        new_cloud = major_plane_->getSweepCloud();
      }
      /*
      cout << "Got a new sweep with "<< new_cloud->points.size() <<" points\n";
      Isometry3dTime null_poseT = Isometry3dTime(update_time, Eigen::Isometry3d::Identity() );
      pc_vis_->pose_to_lcm_from_list(1000, null_poseT);
      pc_vis_->ptcld_to_lcm_from_list(1001, *new_cloud, null_poseT.utime, null_poseT.utime);
      */
      
      Isometry3dTime previous_object_poseT = Isometry3dTime(update_time, object_pose_); 
      pc_vis_->pose_to_lcm_from_list(affordance_vis_, previous_object_poseT);
      pc_vis_->ptcld_to_lcm_from_list(affordance_vis_+1, *object_cloud_, previous_object_poseT.utime, previous_object_poseT.utime);
      pc_vis_->ptcld_to_lcm_from_list(affordance_vis_+2, *object_bb_cloud_,previous_object_poseT.utime, previous_object_poseT.utime);  

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud_copy (new pcl::PointCloud<pcl::PointXYZRGB> ());
      object_cloud_copy = object_cloud_;
      icp_tracker_->doICPTracker( object_cloud_copy, new_cloud, object_pose_ );
      object_pose_ = icp_tracker_->getUpdatedPose();
      //std::cout << object_pose_.translation().transpose() << " new xyz\n";
      //std::cout << object_pose_.rotation() << " new rot\n";
      
      Isometry3dTime   updated_object_pose_T =  Isometry3dTime( update_time, object_pose_);
      pc_vis_->pose_to_lcm_from_list(1020, updated_object_pose_T);
      pc_vis_->ptcld_to_lcm_from_list(1021, *object_cloud_, updated_object_pose_T.utime, updated_object_pose_T.utime);  
      publishUpdatedAffordance(); // updated at sweep frequency
    }
  }else{
    if (got_mask_ && got_initial_affordance_){
      updatePF();
      publishUpdatedAffordance(); // updated at image frequency
    }else{
      cout << "haven't got affs and aff mask yet\n"; 
    }
  }
  
  last_utime_ = msg->utime;
}


void Pass::publishUpdatedAffordance(){
  last_affordance_msg_.aff_store_control =  drc::affordance_t::UPDATE;
  affutils.setXYZRPYFromIsometry3d(last_affordance_msg_.param_names, last_affordance_msg_.params,  object_pose_ );
  lcm_->publish("AFFORDANCE_TRACK", &last_affordance_msg_);
}



Eigen::Isometry3d affordanceToIsometry3d(std::vector<string> param_names, std::vector<double> params ){
  std::map<string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }
  Eigen::Quaterniond quat = euler_to_quat( am.find("yaw")->second , am.find("pitch")->second , am.find("roll")->second );             
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.translation()  << am.find("x")->second , am.find("y")->second, am.find("z")->second;
  transform.rotate(quat);  
  return transform;
}


std::vector<float> affordanceToPlane(std::vector<string> param_names, std::vector<double> params ){
  // Ridiculously hacky way of converting from plane affordance to plane coeffs.
  // the x-direction of the plane pose is along the axis - hence this
  
  std::map<string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }
  Eigen::Quaterniond quat = euler_to_quat( am.find("yaw")->second , am.find("pitch")->second , am.find("roll")->second );             
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.translation()  << am.find("x")->second , am.find("y")->second, am.find("z")->second;
  transform.rotate(quat);  

  Eigen::Isometry3d ztransform;
  ztransform.setIdentity();
  ztransform.translation()  << 0 ,0, 1; // determine a point 1m in the z direction... use this as the normal
  ztransform = transform*ztransform;
  float a =(float) ztransform.translation().x() -  transform.translation().x();
  float b =(float) ztransform.translation().y() -  transform.translation().y();
  float c =(float) ztransform.translation().z() -  transform.translation().z();
  float d = - (a*am.find("x")->second + b*am.find("y")->second + c*am.find("z")->second);
  
  /*
  cout << "pitch : " << 180.*am.find("pitch")->second/M_PI << "\n";
  cout << "yaw   : " << 180.*am.find("yaw")->second/M_PI << "\n";
  cout << "roll   : " << 180.*am.find("roll")->second/M_PI << "\n";   
  obj_cfg oconfig = obj_cfg(1251000,"Tracker | Affordance Pose Z",5,1);
  Isometry3dTime reinit_poseT = Isometry3dTime ( 0, ztransform );
  pc_vis_->pose_to_lcm(oconfig,reinit_poseT);
  */

  std::vector<float> plane = { a,b,c,d};
  return plane;
}

Eigen::Vector4f affordanceToCentroid(std::vector<string> param_names, std::vector<double> params ){
  std::map<string,double> am;
  for (size_t j=0; j< param_names.size(); j++){
    am[ param_names[j] ] = params[j];
  }

  Eigen::Vector4f plane_centroid( am.find("x")->second, am.find("y")->second , 
        am.find("z")->second, 0); // last element held at zero
  return plane_centroid;
}


void Pass::affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_plus_collection_t* msg){
  if (!tracker_initiated_){
    // If we haven't told tracker what do to, then ignore affordances
    return; 
  }  
  int64_t aff_utime =0 ;//TODO: replace this with a proper timestamp coming from the store

  std::vector<int> uids;
  for (size_t i=0; i<msg->affs_plus.size(); i++){
    uids.push_back( msg->affs_plus[i].aff.uid );
  }
  int aff_iter = std::distance( uids.begin(), std::find( uids.begin(), uids.end(), affordance_id_ ) );
  if (aff_iter == msg->affs_plus.size()){
     cout << "Error: Affordance UID " <<  affordance_id_ << " could not be found\n";
     return;
  }
  //std::cout << "The element contining the aff is " << aff_iter << '\n';  
  
  // Bootstrap the filter off the user-provided pose, then ignore:
  if  ( !got_initial_affordance_ ) {
    cout << "got initial position for Affordance "<< affordance_id_ <<"\n";
    drc::affordance_plus_t a = msg->affs_plus[aff_iter];
    object_pose_ = affutils.getPose( a.aff.param_names, a.aff.params );
    pf_ ->ReinitializeComplete(object_pose_, pf_initial_var_);
    Eigen::Vector3f boundbox_lower_left = -0.5* Eigen::Vector3f( a.aff.bounding_lwh[0], a.aff.bounding_lwh[1], a.aff.bounding_lwh[2]);
    Eigen::Vector3f boundbox_upper_right = 0.5* Eigen::Vector3f( a.aff.bounding_lwh[0], a.aff.bounding_lwh[1], a.aff.bounding_lwh[2]);
    icp_tracker_->setBoundingBox (boundbox_lower_left, boundbox_upper_right);
    // TODO: convert a mesh affordance to a point cloud if required
    object_cloud_ = affutils.getCloudFromAffordance(a.points);
    // cache the message and repeatedly update the position:
    last_affordance_msg_ = msg->affs_plus[aff_iter].aff;

    if( verbose_>=1 ){
      Isometry3dTime object_poseT = Isometry3dTime ( aff_utime, object_pose_ );
      pc_vis_->pose_to_lcm_from_list(affordance_vis_, object_poseT);
      object_bb_cloud_ = affutils.getBoundingBoxCloud(a.aff.bounding_xyz, a.aff.bounding_rpy, a.aff.bounding_lwh);
      pc_vis_->ptcld_to_lcm_from_list(affordance_vis_+2, *object_bb_cloud_,object_poseT.utime, object_poseT.utime);  
      pc_vis_->ptcld_to_lcm_from_list(affordance_vis_+1, *object_cloud_, object_poseT.utime, object_poseT.utime);
    }
  }
  got_initial_affordance_ =true;  

  // Update the tracked plane:
  if (use_plane_tracker_){
    
    int plane_aff_iter = std::distance( uids.begin(), std::find( uids.begin(), uids.end(), plane_affordance_id_ ) );
    if (plane_aff_iter == msg->affs_plus.size()){
        cout << "Error: Plane Affordance UID " <<  plane_affordance_id_ << " could not be found - disabling plane tracking\n";
        use_plane_tracker_=false;
        return;
    }
    
    //cout << "got updated plane position\n";
    std::vector<float> p_coeffs = affordanceToPlane(msg->affs_plus[plane_aff_iter].aff.param_names, msg->affs_plus[plane_aff_iter].aff.params );
    Eigen::Vector4f p_centroid = affordanceToCentroid(msg->affs_plus[plane_aff_iter].aff.param_names, msg->affs_plus[plane_aff_iter].aff.params  );
    major_plane_->setPlane(p_coeffs, p_centroid);    
    // TODO: this should be determined initally and retained:
    plane_relative_xyzypr_  = { 0, 0, 0.12, 0., 0., 0.};
    plane_relative_xyzypr_set_  = { 0, 0, 1, 0, 1, 1};
    plane_pose_set_=true;
    plane_coeffs_->values = p_coeffs;
    plane_pose_ = major_plane_->determinePlanePose(plane_coeffs_, p_centroid);
    
    if( verbose_>=1 ){
      //cout << p_coeffs[0] << " " << p_coeffs[1] << " " << p_coeffs[2] << " " << p_coeffs[3] << "\n";
      obj_cfg oconfig2 = obj_cfg(1351000,"Tracker | Affordance Plane",5,1);
      Isometry3dTime pT = Isometry3dTime ( aff_utime, plane_pose_ );
      pc_vis_->pose_to_lcm(oconfig2,pT);
    }
  }
}



void Pass::maskHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  last_mask_= *msg;  
  if (!got_mask_){
    cout << "got mask\n";  
  }
  got_mask_=true;
}


void Pass::initiate_tracking(int affordance_id, bool use_color_tracker, bool use_histogram_tracker, 
                             bool use_icp_tracker,
                             bool use_plane_tracker, int plane_affordance_id){
  affordance_id_ = affordance_id;
  use_color_tracker_ = use_color_tracker;
  use_histogram_tracker_ = use_histogram_tracker;
  use_icp_tracker_ = use_icp_tracker;
  use_plane_tracker_ = use_plane_tracker;
  plane_affordance_id_ = plane_affordance_id;
  tracker_initiated_ = true;
  got_initial_affordance_ = false;
  
  if( verbose_ >=1 ){
    stringstream ss2;
    ss2 << "Tracker | Pose of Aff " << affordance_id_;
    affordance_vis_ = 4451006 +  affordance_id_*10;
    // obj: id name type reset
    // pts: id name type reset objcoll usergb rgb  
    pc_vis_->obj_cfg_list.push_back( obj_cfg(affordance_vis_, ss2.str() ,5,1) );
    pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(affordance_vis_+1,"Tracker | Aff Cloud [at Prev]"     ,1,1, affordance_vis_,1, {1,0,0}));
    pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(affordance_vis_+2,"Tracker | Bounding Box [Prev]"     ,4,1, affordance_vis_,1, {1,0,0}));
    stringstream ss;
    ss << "Tracker | Particles of Aff " << affordance_id_;
    pc_vis_->obj_cfg_list.push_back( obj_cfg(affordance_vis_+3  , ss.str() ,5,1) );
  }
  
}

void Pass::trackerCommandHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::tracker_command_t* msg){
  cout << "got tracker command\n";
  
  if ( msg->tracker_id != tracker_instance_id_){
    cout << "my tracker id is ["<< (int) tracker_instance_id_ 
         <<"] but command is for ["<< (int) msg->tracker_id <<"]. Ignoring command\n";
         return;
  }

  affordance_id_ = -1;
  use_color_tracker_ = false;
  use_histogram_tracker_= false;
  use_icp_tracker_ = false;
  use_plane_tracker_= false;
  plane_affordance_id_ = -1;
  
  
  if (msg->tracker_type== drc::tracker_command_t::STOP){
    tracker_initiated_ = false;
    got_initial_affordance_ = false;
    got_mask_ = false;
    cout << "Halting Tracker on command\n";
    return;
  }else if(msg->tracker_type== drc::tracker_command_t::COLOR){ 
    use_color_tracker_ = true;
  }else if(msg->tracker_type== drc::tracker_command_t::HISTOGRAM){ 
    use_histogram_tracker_= true;
  }else if(msg->tracker_type== drc::tracker_command_t::ICP){ 
    use_icp_tracker_= true;
  }else{
    cout << "Tracker Type not understood ["<< ((int)msg->tracker_type)  <<"]\n";
  }
  
  if (msg->plane_uid >= 0){
    plane_affordance_id_ = msg->plane_uid;
    use_plane_tracker_=true;
  }
  
  // Update tracker status:
  // Largely Superflous
  initiate_tracking(msg->uid, use_color_tracker_, use_histogram_tracker_, 
                    use_icp_tracker_,
                    use_plane_tracker_, plane_affordance_id_);  
}


int main(int argc, char ** argv) {
  string image_channel = "CAMERALEFT";
  int num_particles = 300;
  int affordance_id = -1;
  bool use_color_tracker =false;
  bool use_histogram_tracker =false;
  bool use_plane_tracker =false;  
  bool use_icp_tracker =false;  
  int verbose = 0;
  int plane_affordance_id=-1;
  int tracker_instance_id=1; // id of this tracker instance. Plane Tracker==0, otherwise above that
  ConciseArgs opt(argc, (char**)argv);
  opt.add(image_channel, "e", "image_channel","image_channel");
  opt.add(num_particles, "n", "num_particles","num particles");
  opt.add(affordance_id, "a", "affordance_id","Affordance ID");
  opt.add(use_color_tracker, "c", "use_color_tracker","Use Color Tracker");
  opt.add(use_histogram_tracker, "g", "use_histogram_tracker","Use Histogram Tracker");
  opt.add(use_icp_tracker, "i", "use_icp_tracker","Use ICP Tracker");
  opt.add(use_plane_tracker, "p", "use_plane_tracker","Use Plane Tracker");
  opt.add(plane_affordance_id, "l", "plane_affordance_id","Plane Affordance ID");
  opt.add(tracker_instance_id, "t", "tracker","Tracker ID");
  opt.add(verbose, "v", "verbosity","0 none, 1 little, 2 debug");
  opt.parse();
  std::cout << "image_channel: " << image_channel << "\n";    
  std::cout << "num_particles: " << num_particles << "\n";    
  std::cout << "affordance_id: " << affordance_id << "\n";    
  std::cout << "use_color_tracker: " << use_color_tracker << "\n";    
  std::cout << "use_histogram_tracker: " << use_histogram_tracker << "\n";    
  std::cout << "plane_affordance_id: " << plane_affordance_id << "\n";    
  std::cout << "tracker_instance_id: " << tracker_instance_id << " [id of this tracker process]\n";    
  std::cout << "verbose: " << verbose << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, image_channel, num_particles, tracker_instance_id, verbose);

  if (affordance_id>=0){
    // if a valid id - initiate tracker on launch
    app.initiate_tracking(affordance_id, use_color_tracker, use_histogram_tracker,
      use_icp_tracker, use_plane_tracker, plane_affordance_id);
  }else{
    cout << "Starting Object Tracker Uninitiated\n";
  }  
  
  cout << "Object Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
