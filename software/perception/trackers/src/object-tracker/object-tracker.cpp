// color threshold and plane are hard coded
// 



// TODO:
// position (and orientation) of affordance
// a plane of interest - currently largest
// relative offset between plane and object
// color of object (find automatically using mask)

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
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
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
            int num_particles_, int affordance_id_);
    
    ~Pass(){
    }    
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
    
    int counter_;
    pointcloud_vis* pc_vis_;
    image_io_utils*  imgutils_;
    
    ParticleFilter* pf_; 
    int num_particles_;
    int affordance_id_; // id of the affordance we want to track
    std::vector<double> pf_initial_var_;
    std::vector <double> pf_drift_var_;
  
    bot_core::image_t img_;  
    int64_t last_utime_;    
       
    // Current status of plane estimation:
    Eigen::Isometry3d plane_pose_ ;
    bool plane_pose_set_;
    std::vector<double> input_plane_;
    
    bool got_mask_;
    bool got_affs_;
  protected:    
    MajorPlane* major_plane_;    
    ColorThreshold* color_tracker_;
    
    HistTracker* histogram_tracker_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
    int num_particles_, int affordance_id_): 
    lcm_(lcm_), image_channel_(image_channel_), 
    num_particles_(num_particles_), affordance_id_(affordance_id_){

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

  stringstream ss;
  ss << "Tracker | Pose of Aff " << affordance_id_;
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451006 +  affordance_id_  , ss.str() ,5,1) );
  
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
  major_plane_ = new MajorPlane( lcm_, 1);
  plane_pose_.setIdentity();
  plane_pose_set_ = false;
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
  
  // Apply a constraint onto the XY plane at 0,0,0
  // with freedom of rotation in that plane
  //  std::vector <double> xyzypr{ -6.0, 0, 0., 0., 0., 0.};
  //  std::vector <bool> set_xyzypr{ 1, 0, 0, 1, 1, 0};
  //  pf_->SetState(xyzypr, set_xyzypr);
  
  if ( plane_pose_set_ ){
    vector < pf_state > pfs;
    for (size_t i=0; i<num_particles_; i++) {
      pfs.push_back( pf_->GetParticleState(i)   );
    }
      
    for (size_t i=0; i<num_particles_; i++) {
      pfs[i].pose = plane_pose_.inverse() * pfs[i].pose ;
    }  
    
    std::vector <double> xyzypr{ 0, 0, 0.25, 0., 0., 0.};
    std::vector <bool> set_xyzypr{ 0, 0, 1, 0, 1, 1};
    
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
      pf_->SetParticleState(i, pfs[i]);
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
  Eigen::Isometry3d local_to_camera;
  frames_cpp_->get_trans_with_utime( botframes_ , "local", "CAMERA"  , img_.utime, local_to_camera);
  std::vector< Eigen::Vector3d > pts;
  for (size_t i=0; i<num_particles_; i++) {
    pf_state particle_state;
    particle_state =pf_->GetParticleState(i);
    Eigen::Vector3d t(particle_state.pose.translation());
    pts.push_back(t);
  }    
  loglikelihoods = color_tracker_->colorThreshold(pts, img_.data.data(), local_to_camera, img_.utime);
  // to disable :   loglikelihoods.assign ( pts->points.size() ,0);    
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
    Eigen::Isometry3d local_to_camera;
    frames_cpp_->get_trans_with_utime( botframes_ , "local", "CAMERA"  , img_.utime, local_to_camera);
    std::vector< Eigen::Vector3d > pts;
    for (size_t i=0; i<num_particles_; i++) {
      pf_state particle_state;
      particle_state =pf_->GetParticleState(i);
      Eigen::Vector3d t(particle_state.pose.translation());
      pts.push_back(t);
    }        
      
    double tic = _timestamp_now(); 
    loglikelihoods = histogram_tracker_->update(img, scale, pts, local_to_camera);
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
  colorThresholdLikelihood(loglikelihoods);
  //histogramThresholdLikelihood(loglikelihoods);
  
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

  // Ask the maps collector for a plane:
  plane_pose_set_ = major_plane_->getPlane(plane_pose_, msg->utime);
  return;
  
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

void Pass::affordanceHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::affordance_collection_t* msg){
  if  ( !got_affs_ ) {
    cout << "got affs\n";
    Eigen::Isometry3d reinit_pose = affordanceToIsometry3d( msg->affs[affordance_id_].param_names, msg->affs[affordance_id_].params );
    pf_ ->ReinitializeComplete(reinit_pose, pf_initial_var_);
  }
  got_affs_ =true;  
}



void Pass::maskHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  bot_core::image_t* msg){
  last_mask_= *msg;  
  if (!got_mask_){
    cout << "got mask\n";  
    
    
    /*
     * 
     * 251529000 timeMin | 254130000 timeMax | 254237000 utime | new process
New RANSAC Floor Coefficients: 0.920891 -0.389821 0.000566057 6.52132
Pitch 89.9676 | Yaw -22.9434
Centroid: -7.14644 -0.151769 1.12628
*/
    
    // the plane that the green object is on:
  //  input_plane_ = {0.921108, -0.389307, 0.000358538, 6.52338}; 
    // a second plane of the same for matching:
//    std::vector<double> second_plane = {0.921117, -0.389286, -0.000399105, 6.52459};
    
    
    std::vector<float> plane_coeffs = { 0.920891, -0.389821, 0.000566057, 6.52132};
    Eigen::Vector4f centroid( -7.14644, -0.151769, 1.12628, 0);
    
//    setPlane
    
//    0.919775 -0.392445 -0.000469124 6.51755


  }
  got_mask_=true;
}




int main(int argc, char ** argv) {
  string channel = "CAMERALEFT";
  int num_particles = 100;
  int affordance_id = 0;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(channel, "c", "channel","channel");
  opt.add(num_particles, "n", "num_particles","num particles");
  opt.add(affordance_id, "a", "affordance_id","Affordance ID");
  opt.parse();
  std::cout << "channel: " << channel << "\n";    
  std::cout << "num_particles: " << num_particles << "\n";    
  std::cout << "affordance_id: " << affordance_id << "\n";    

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, channel, num_particles, affordance_id);
  cout << "Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}