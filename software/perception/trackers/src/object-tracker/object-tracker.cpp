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
#include <trackers/color-threshold.hpp>
#include <trackers/HistTracker.hpp>

using namespace std;

// offset of affordance in mask labelling:
#define AFFORDANCE_OFFSET 64


int affordance_vis_offset =0;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_, int num_particles_);
    
    ~Pass(){
    }    
    
    void init_tracking(int affordance_id, bool use_color_tracker, bool use_histogram_tracker, 
                       bool use_plane_tracker, int plane_affordance_id);
    
  private:
    
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    std::string image_channel_;

    void maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    std::string mask_channel_;
    bot_core::image_t last_mask_;    
    
    void affordanceHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_collection_t* msg);
    
    void updatePF();
    void propogatePF();
    void colorThresholdLikelihood( std::vector<float> &loglikelihoods );
    void histogramThresholdLikelihood( std::vector<float> &loglikelihoods );
    
    
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* frames_cpp_;

    // Camera Params:
    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;
    bot_core::image_t img_;  
    int64_t last_utime_;    
    bool got_mask_;
    bool got_affs_;    
    
    int counter_;
    pointcloud_vis* pc_vis_;
    image_io_utils*  imgutils_;
    
    // Current Camera Pose:
    Eigen::Isometry3d local_to_camera_;
    Eigen::Isometry3d head_to_local_;
    
    // Particle Filter Variables:
    ParticleFilter* pf_; 
    int num_particles_;
    std::vector<double> pf_initial_var_;
    std::vector <double> pf_drift_var_;
  
    // Tracker Configuration:
    int affordance_id_; // id of the affordance we want to track
    int plane_affordance_id_;
    MajorPlane* major_plane_;    
    ColorThreshold* color_tracker_;
    HistTracker* histogram_tracker_;
    bool use_plane_tracker_;
    bool use_color_tracker_;
    bool use_histogram_tracker_;
    
    // Plane Tracking Variables:
    Eigen::Isometry3d plane_pose_ ; // Current status of plane estimation:
    bool plane_pose_set_;
    std::vector<double> input_plane_;
    // The relative position of the plane and which dimensions to constrain:
    std::vector <double> plane_relative_xyzypr_; 
    std::vector <bool> plane_relative_xyzypr_set_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
    int num_particles_): 
    lcm_(lcm_), image_channel_(image_channel_), 
    num_particles_(num_particles_){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);
  mask_channel_="CAMERALEFT_MASKZIPPED";
  lcm_->subscribe( mask_channel_ ,&Pass::maskHandler,this);
  lcm_->subscribe("AFFORDANCE_COLLECTION",&Pass::affordanceHandler,this);  
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451000,"Tracker | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451001,"Tracker | Particles"           ,1,1, 4451000,1, { 0.0, 1.0, 0.0} ));

  
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
  pf_initial_var_ =  { .01  ,.01  , .001   , .001 };
  pf_drift_var_ = { .0001 ,.0001 , .00001  , .00001 }; // made lower
  Eigen::Isometry3d init_pose;
  init_pose.setIdentity();
  pf_ = new ParticleFilter(lcm_->getUnderlyingLCM(), num_particles_,init_pose,
            pf_initial_var_, rng_seed,resample_threshold);  
  
  counter_=0;
  img_.utime=0; // used to indicate no message recieved yet
  last_utime_=0;

  // Image Masking:
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_);
  got_mask_ = false;
  got_affs_ = false;
    
  // Color Tracking:
  color_tracker_ = new ColorThreshold(lcm_, width_, height_, fx_, fy_, cx_, cy_);
  // Histogram Tracking:  
  histogram_tracker_ = new HistTracker();
  
  // Plane Detection:
  major_plane_ = new MajorPlane( lcm_, 2);
  plane_pose_.setIdentity();
  plane_pose_set_ = false;
}


void Pass::init_tracking(int affordance_id, bool use_color_tracker, bool use_histogram_tracker, bool use_plane_tracker, int plane_affordance_id){
  affordance_id_ = affordance_id;
  use_color_tracker_ = use_color_tracker;
  use_histogram_tracker_ = use_histogram_tracker;
  use_plane_tracker_ = use_plane_tracker;
  plane_affordance_id_ = plane_affordance_id;
  
  stringstream ss;
  ss << "Tracker | Pose of Aff " << affordance_id_;
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451006 +  affordance_id_  , ss.str() ,5,1) );
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
  pc_vis_->pose_collection_to_lcm_from_list(4451006 + affordance_id_, pf_poses);
}


void Pass::colorThresholdLikelihood( std::vector<float> &loglikelihoods ){
  std::vector< Eigen::Vector3d > pts;
  for (size_t i=0; i<num_particles_; i++) {
    pf_state particle_state;
    particle_state =pf_->GetParticleState(i);
    Eigen::Vector3d t(particle_state.pose.translation());
    pts.push_back(t);
  }    
  loglikelihoods = color_tracker_->colorThreshold(pts, img_.data.data(), local_to_camera_, img_.utime);
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void Pass::histogramThresholdLikelihood( std::vector<float> &loglikelihoods ){
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
    colorThresholdLikelihood(loglikelihoods);
  if (use_histogram_tracker_)
    histogramThresholdLikelihood(loglikelihoods);
  
  pf_->LogLikelihoodParticles(loglikelihoods);
  //pf_->SendParticlesLCM( img_.utime ,0);//vo_estimate_status);
  double ESS = pf_->ConsiderResample();
  std::cerr << "                              "<< ESS/num_particles_ << " ESS | " << img_.utime  << " utime\n";  
}

void Pass::imageHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  counter_++;
  if (counter_%30 ==0){ cout << counter_ << " | " << msg->utime << "\n";   }  
  if (width_ != msg->width){
    cout << "incoming width " << msg->width << " doesn't match assumed width " << width_ << "\n";
    cout << "Cowardly refusing to process\n";
    return;
  }
  img_= *msg;  

  // Update the camera and head poses
  frames_cpp_->get_trans_with_utime( botframes_ , "local", "CAMERA"  , img_.utime, local_to_camera_);
  frames_cpp_->get_trans_with_utime( botframes_ , "head", "local"  , img_.utime, head_to_local_);
  
  
  // Ask the maps collector for a plane:
  // true if we got a new sweep, make a box 5x5x5m centered around the robot's head
  // cout << "head_to_local_: " << head_to_local_.translation() << "\n";
  if (major_plane_->getSweep(head_to_local_.cast<float>().translation() ,  Eigen::Vector3f( 3., 3., 3.)) ){ 
    plane_pose_set_ = major_plane_->trackPlane(plane_pose_, msg->utime);  
  }
  
  if (got_mask_ && got_affs_){
    updatePF();
  }else{
    cout << "haven't got affs and aff mask yet\n"; 
  }
  
  last_utime_ = msg->utime;
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


void Pass::affordanceHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_collection_t* msg){
  if  ( !got_affs_ ) {
    cout << "got affs\n";
    Eigen::Isometry3d reinit_pose = affordanceToIsometry3d( msg->affs[affordance_id_].param_names, msg->affs[affordance_id_].params );
    pf_ ->ReinitializeComplete(reinit_pose, pf_initial_var_);
    
    obj_cfg oconfig = obj_cfg(1451000,"Tracker | Affordance Pose",5,1);
    Isometry3dTime reinit_poseT = Isometry3dTime ( 0, reinit_pose );
    pc_vis_->pose_to_lcm(oconfig,reinit_poseT);
    
    
    
    std::vector<float> p_coeffs = affordanceToPlane(msg->affs[plane_affordance_id_].param_names, msg->affs[plane_affordance_id_].params );
    Eigen::Vector4f p_centroid = affordanceToCentroid(msg->affs[plane_affordance_id_].param_names, msg->affs[plane_affordance_id_].params  );
    major_plane_->setPlane(p_coeffs, p_centroid);    
    plane_relative_xyzypr_  = { 0, 0, 0.12, 0., 0., 0.};
    plane_relative_xyzypr_set_  = { 0, 0, 1, 0, 1, 1};
    
    
    cout << p_coeffs[0] << " " << p_coeffs[1] << " " << p_coeffs[2] << " " << p_coeffs[3] << "\n";
    pcl::ModelCoefficients::Ptr new_p_coeffs(new pcl::ModelCoefficients ());
    new_p_coeffs->values = p_coeffs;
    Eigen::Isometry3d p = major_plane_->determinePlanePose(new_p_coeffs, p_centroid);
    obj_cfg oconfig2 = obj_cfg(1351000,"Tracker | Affordance Plane",5,1);
    Isometry3dTime pT = Isometry3dTime ( 0, p );
    pc_vis_->pose_to_lcm(oconfig2,pT);

    
        
    /*
    if (plane_affordance_id_ ==0){
      // Plane with valve and wheel on it
      std::vector<float> plane_coeffs = { 0.920891, -0.389821, 0.000566057, 6.52132};
      Eigen::Vector4f plane_centroid( -7.14644, -0.151769, 1.12628, 0); // last element held at zero
      major_plane_->setPlane(plane_coeffs, plane_centroid);    
      plane_relative_xyzypr_  = { 0, 0, 0.25, 0., 0., 0.};
      plane_relative_xyzypr_set_  = { 0, 0, 1, 0, 1, 1};
    }else if(plane_affordance_id_==1){
      // Table plane:
      std::vector<float> plane_coeffs = {-0.00466229, -0.000804597, 0.999989, -1.01261};
      Eigen::Vector4f plane_centroid( 1.49616, 1.79947, 1.02105, 0); // last element held at zero
      major_plane_->setPlane(plane_coeffs, plane_centroid);    
      plane_relative_xyzypr_  = { 0, 0, 0.12, 0., 0., 0.};
      plane_relative_xyzypr_set_  = { 0, 0, 1, 0, 1, 1};
      // xyz_rpy_w_l:1.49616 1.79947 1.02105 -0.000754216 0.00466083 -3.13113 0.816344 1.50757
    }
    */
  }
  got_affs_ =true;  
}



void Pass::maskHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  last_mask_= *msg;  
  if (!got_mask_){
    cout << "got mask\n";  
  }
  got_mask_=true;
}




int main(int argc, char ** argv) {
  string image_channel = "CAMERALEFT";
  int num_particles = 300;
  int affordance_id = 0;
  bool use_color_tracker =false;
  bool use_histogram_tracker =false;
  bool use_plane_tracker =false;  
  int plane_affordance_id=0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(image_channel, "i", "image_channel","image_channel");
  opt.add(num_particles, "n", "num_particles","num particles");
  opt.add(affordance_id, "a", "affordance_id","Affordance ID");
  opt.add(use_color_tracker, "c", "use_color_tracker","Use Color Tracker");
  opt.add(use_histogram_tracker, "g", "use_histogram_tracker","Use Histogram Tracker");
  opt.add(use_plane_tracker, "p", "use_plane_tracker","Use Plane Tracker");
  opt.add(plane_affordance_id, "l", "plane_affordance_id","Plane Affordance ID");
  opt.parse();
  std::cout << "image_channel: " << image_channel << "\n";    
  std::cout << "num_particles: " << num_particles << "\n";    
  std::cout << "affordance_id: " << affordance_id << "\n";    
  std::cout << "use_color_tracker: " << use_color_tracker << "\n";    
  std::cout << "use_histogram_tracker: " << use_histogram_tracker << "\n";    
  std::cout << "plane_affordance_id: " << plane_affordance_id << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, image_channel, num_particles);
  app.init_tracking(affordance_id, use_color_tracker, use_histogram_tracker,
      use_plane_tracker, plane_affordance_id);
  
  cout << "Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}