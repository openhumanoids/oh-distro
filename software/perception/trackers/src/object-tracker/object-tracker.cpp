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
#include <ConciseArgs>

#include <particle/particle_filter.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

#include <trackers/major-plane-detect.hpp>
#include <trackers/color-threshold.hpp>

using namespace std;

int mode =0;
int vis_offset =0;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
      int num_particles_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void imageHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    std::string image_channel_;

    void updatePF();
    void propogatePF();
    void colorThresholdLikelihood( std::vector<float> &loglikelihoods );
    
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
    int64_t last_utime_;    
       
    // Current status of plane estimation:
    Eigen::Isometry3d plane_pose_ ;
    bool plane_pose_set_;
  protected:    
    MajorPlane* major_plane_;    
    ColorThreshold* color_thres_;    
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
    int num_particles_): lcm_(lcm_), image_channel_(image_channel_), num_particles_(num_particles_){

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
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451006 +vis_offset,"Tracker | Poses",5,1) );
  
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
  std::vector<double> initial_var  { .1  ,.1  , .001   , .001 };
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
  last_utime_=0;
  
  // Color Thresholding:
  color_thres_ = new ColorThreshold(lcm_, width_, height_, fx_, fy_, cx_, cy_);

  // Plane Detection:
  major_plane_ = new MajorPlane( lcm_);
  plane_pose_.setIdentity();
  plane_pose_set_ = false;
}


void Pass::colorThresholdLikelihood( std::vector<float> &loglikelihoods ){

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
  
  loglikelihoods = color_thres_->colorThreshold(pts, img_.data.data(), local_to_camera, img_.utime);
  // to disable :   loglikelihoods.assign ( pts->points.size() ,0);    

}


void Pass::propogatePF(){
  std::vector <double> success_var { .0001,.0001, .000001, .000001 }; // not used
  std::vector <double> failure_var { .0001 ,.0001 , .00001  , .00001 }; // made lower
  
  double elapsed_time = 0.1;
  double msg_dt = (double) (img_.utime - last_utime_)/1E6;
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


void Pass::updatePF(){
  std::vector<float> loglikelihoods;
  propogatePF();
  
  // Color Threshold Likelihood
  colorThresholdLikelihood(loglikelihoods);
  
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
    cout << "Cowardly refusing to process\n";
    return;
  }
  img_= *msg;  

  // As the maps collector for a plane:
  plane_pose_set_ = major_plane_->getPlane(plane_pose_, msg->utime);
  updatePF();
  
  last_utime_ = msg->utime;
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
  
  Pass app(lcm, channel, num_particles);
  cout << "Tracker ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}