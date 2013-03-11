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


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <particle/particle_filter.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <ConciseArgs>

using namespace cv;
using namespace std;

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
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, std::string image_channel_,
    int num_particles_): lcm_(lcm_), image_channel_(image_channel_), 
    num_particles_(num_particles_){

  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  lcm_->subscribe( image_channel_ ,&Pass::imageHandler,this);

  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451000,"Pose - Null",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451001,"Cloud - Null"           ,1,1, 4451000,1, { 0.0, 1.0, 0.0} ));
  
  
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
  init_pose.translation() << -6.0, 0.5, 1.2;
  pf = new ParticleFilter(lcm_->getUnderlyingLCM(), num_particles_,init_pose,
            initial_var, rng_seed,resample_threshold);  
  
  counter_=0;
  img_.utime=0; // used to indicate no message recieved yet
  last_img_.utime=0; // used to indicate no message recieved yet
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
  // red valve in VRC:
  cvInRangeS(imgHSV, cvScalar(0, 100, 1), cvScalar(5, 255, 60), imgThreshed);

  cvReleaseImage(&imgHSV);
  return imgThreshed;
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
    int u = floor( pt1.x * fx_/pt1.z + cx_);
    int v = floor( pt1.y * fy_/pt1.z + cy_);
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
    int u = floor( pt1.x * fx_/pt1.z  +  512.5);
    int v = floor( pt1.y * fy_/pt1.z + 272.5);
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
  std::vector <double> failure_var { .0001 ,.0001 , .00001  , .00001 }; // made lower
  
  double elapsed_time = 0.1;
  double msg_dt = (double) (img_.utime - last_img_.utime)/1E6;
  if ( fabs( msg_dt) < 0.5){ // avoid odd delta times
    elapsed_time = msg_dt;
  }
  cout << elapsed_time << "\n";
  
  int64_t current_timestamp = img_.utime;

  pf_state odom_diff;
  odom_diff.pose.setIdentity();
  odom_diff.velocity.setIdentity();
  pf->MoveParticles(odom_diff,failure_var,elapsed_time,1); //failed motion estimation
  
  // Apply a constraint onto the XY plane at 0,0,0
  // with freedom of rotation in that plane
  std::vector <double> xyzypr{ -6.0, 0, 0., 0., 0., 0.};
  std::vector <bool> set_xyzypr{ 1, 0, 0, 1, 1, 0};
  pf->SetState(xyzypr, set_xyzypr);
}


void Pass::updatePF( std::vector<float> &loglikelihoods ){
  pf->LogLikelihoodParticles(loglikelihoods);
  pf->SendParticlesLCM( img_.utime ,0);//vo_estimate_status);

  double ESS;
  ESS= pf->ConsiderResample();
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
  opt.parse();
  std::cout << "channel: " << channel << "\n";    
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm, channel, num_particles);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}