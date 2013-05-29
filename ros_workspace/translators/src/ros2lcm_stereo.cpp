// rosparam set /compressed_listener/image_transport compressed; rosparam set /ros2lcm/stereo hand; rosrun translators  ros2lcm_stereo  __name:=ros2lcm_hand
// rosparam set /ros2lcm_head/image_transport compressed; rosparam set /ros2lcm/stereo head;  rosrun translators ros2lcm_stereo  __name:=ros2lcm_head
//  rosparam set /compressed_listener/image_transport compressed
//  rosrun translators  my_subscriber __name:=compressed_listener

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>


#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <sandia_hand_msgs/RawTactile.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#define DO_TIMING_PROFILE false
using namespace std;

int width =800;//1024; // hardcoded
int height =800;//544; // hardcoded
uint8_t* stereo_data = new uint8_t [2*3* width*height]; // 2 color scale images stacked
uint8_t* singleimage_data = new uint8_t [2*2* width*height]; // 1 color scale image
int jpeg_quality=90;
// Estimated Rates at 90% compression and 5Hz
// CAMERA 200KB x5Hz
// CAMERALEFT 114KB x5Hz
// CAMERA_LHAND 150KB x5Hz
// CAMERA_RHAND 150KB x5Hz
// = 3MB/sec

class App{
public:
  App(ros::NodeHandle node_, bool send_head_cameras_, bool send_hand_cameras_);
  ~App();

private:
  bool send_head_cameras_;  // listen for and publish the cameras
  bool send_hand_cameras_; // listen for and publish the hand cameras (the above must be true also!)
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Imu:
  ros::Subscriber head_imu_sub_;
  void head_imu_cb(const sensor_msgs::ImuConstPtr& msg);
  void send_imu(const sensor_msgs::ImuConstPtr& msg,string channel );

  // Laser:
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  // porterbot:
  ros::Subscriber scan_left_sub_,scan_right_sub_; // porterbot
  void scan_left_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void scan_right_cb(const sensor_msgs::LaserScanConstPtr& msg);

  // Rate messages:
  double hand_stereo_publish_fps_, head_stereo_publish_fps_;
  ros::Subscriber head_fps_sub_,hand_fps_sub_;
  void head_fps_cb(const std_msgs::Float64ConstPtr& msg);
  void hand_fps_cb(const std_msgs::Float64ConstPtr& msg);  
  
  // Combined Stereo Image:
  bot_core::image_t stereo_msg_out_;  
  void publishStereo(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info, std::string camera_out);
  image_transport::ImageTransport it_;
  
  ///////////////////////////////////////////////////////////////////////////////
  int64_t head_stereo_last_utime_;
  void head_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  int64_t l_hand_stereo_last_utime_;
  
  void l_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter l_hand_l_image_sub_, l_hand_r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_hand_l_info_sub_, l_hand_r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> l_hand_sync_;
  
  int64_t r_hand_stereo_last_utime_;
  void r_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image, const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image, const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::SubscriberFilter r_hand_l_image_sub_, r_hand_r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_hand_l_info_sub_, r_hand_r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> r_hand_sync_;
  
  // Hand Sensors:
  ros::Subscriber l_hand_tactile_sub_, r_hand_tactile_sub_;
  void l_hand_tactile_cb(const sandia_hand_msgs::RawTactileConstPtr& msg);
  void r_hand_tactile_cb(const sandia_hand_msgs::RawTactileConstPtr& msg);
  void send_tactile(const sandia_hand_msgs::RawTactileConstPtr& msg,string channel );  
  
  
  // Image Compression
  image_io_utils*  imgutils_;
};

App::App(ros::NodeHandle node_, bool send_head_cameras_, bool send_hand_cameras_) :
    node_(node_), it_(node_), sync_(10), l_hand_sync_(10), r_hand_sync_(10),
    send_head_cameras_(send_head_cameras_), send_hand_cameras_(send_hand_cameras_){
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  imgutils_ = new image_io_utils( lcm_publish_.getUnderlyingLCM(), width, height );
  
  if (send_head_cameras_){
    // IMU:
    head_imu_sub_ = node_.subscribe(string("/multisense_sl/imu"), 100, &App::head_imu_cb,this);
    // Laser:
    rotating_scan_sub_ = node_.subscribe(string("/multisense_sl/laser/scan"), 100, &App::rotating_scan_cb,this);
    
    // This is done by directly requesting from Gazebo at lower rate:
    head_stereo_publish_fps_ = 1.0;
    //head_fps_sub_ = node_.subscribe("/mit/set_head_fps", 100, &App::head_fps_cb,this);
    
    // Stereo Image:
    std::string lim_string ,lin_string,rim_string,rin_string;
    std::string head_stereo_root = "/multisense_sl/camera";
    // Grey:
    lim_string = head_stereo_root + "/left/image_rect";
    lin_string = head_stereo_root + "/left/camera_info";
    rim_string = head_stereo_root + "/right/image_rect";
    rin_string = head_stereo_root + "/right/camera_info";
    cout << lim_string << " is the left stereo image subscription [for stereo]\n";
    l_image_sub_.subscribe(it_, ros::names::resolve( lim_string ), 30);
    l_info_sub_.subscribe(node_, ros::names::resolve( lin_string ), 30);
    r_image_sub_.subscribe(it_, ros::names::resolve( rim_string ), 30);
    r_info_sub_.subscribe(node_, ros::names::resolve( rin_string ), 30);
    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
    sync_.registerCallback( boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4) );
    head_stereo_last_utime_=0;
  }
    
  /////////////////////////////// Hand Cameras /////////////////////////////////////
  if (send_hand_cameras_){
    // This is done by discarding whats coming from ros:
    hand_stereo_publish_fps_ = 5.0;
    hand_fps_sub_ = node_.subscribe("/mit/set_hand_fps", 10, &App::hand_fps_cb,this);

    
    l_hand_l_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/l_hand/camera/left/image_raw" ), 30);
    l_hand_l_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/l_hand/camera/left/camera_info" ), 30);
    l_hand_r_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/l_hand/camera/right/image_raw" ), 30);
    l_hand_r_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/l_hand/camera/right/camera_info"  ), 30);
    l_hand_sync_.connectInput(l_hand_l_image_sub_, l_hand_l_info_sub_, l_hand_r_image_sub_, l_hand_r_info_sub_);
    l_hand_sync_.registerCallback( boost::bind(&App::l_hand_stereo_cb, this, _1, _2, _3, _4) );        
    l_hand_stereo_last_utime_=0;

    r_hand_l_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/r_hand/camera/left/image_raw" ), 30);
    r_hand_l_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/r_hand/camera/left/camera_info" ), 30);
    r_hand_r_image_sub_.subscribe(it_, ros::names::resolve( "/sandia_hands/r_hand/camera/right/image_raw" ), 30);
    r_hand_r_info_sub_.subscribe(node_, ros::names::resolve( "/sandia_hands/r_hand/camera/right/camera_info"  ), 30);
    r_hand_sync_.connectInput(r_hand_l_image_sub_, r_hand_l_info_sub_, r_hand_r_image_sub_, r_hand_r_info_sub_);
    r_hand_sync_.registerCallback( boost::bind(&App::r_hand_stereo_cb, this, _1, _2, _3, _4) );   
    r_hand_stereo_last_utime_=0;
    
    
    l_hand_tactile_sub_ = node_.subscribe( string("/sandia_hands/l_hand/tactile_raw"),100, &App::l_hand_tactile_cb ,this);
    r_hand_tactile_sub_ = node_.subscribe( string("/sandia_hands/r_hand/tactile_raw"),100, &App::r_hand_tactile_cb ,this);
  }
  
};

App::~App()  {
}



void App::send_tactile(const sandia_hand_msgs::RawTactileConstPtr& msg,string channel ){
  drc::raw_tactile_t msg_out;
  msg_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);  

  msg_out.f0.assign( msg->f0.begin(), msg->f0.end() );
  msg_out.n_f0 = msg_out.f0.size();
  msg_out.f1.assign( msg->f1.begin(), msg->f1.end() );
  msg_out.n_f1 = msg_out.f1.size();
  msg_out.f2.assign( msg->f2.begin(), msg->f2.end() );
  msg_out.n_f2 = msg_out.f2.size();
  msg_out.f3.assign( msg->f3.begin(), msg->f3.end() );
  msg_out.n_f3 =msg_out.f3.size();
  msg_out.palm.assign( msg->palm.begin(), msg->palm.end() );
  msg_out.n_palm =msg_out.palm.size();

  lcm_publish_.publish(channel, &msg_out);
}

void App::l_hand_tactile_cb(const sandia_hand_msgs::RawTactileConstPtr& msg){
  //ROS_ERROR("ROS2LCM Stereo: got l hand tacile");  
  send_tactile(msg,"TACTILE_RAW_LHAND");  
}
void App::r_hand_tactile_cb(const sandia_hand_msgs::RawTactileConstPtr& msg){
  //ROS_ERROR("ROS2LCM Stereo: got r hand tacile");  
  send_tactile(msg,"TACTILE_RAW_RHAND");  
}

void App::hand_fps_cb(const std_msgs::Float64ConstPtr& msg){
  hand_stereo_publish_fps_ = double( msg->data);
  ROS_ERROR("ROS2LCM Stereo: set output fps to : %f", hand_stereo_publish_fps_ );  
}

void App::head_imu_cb(const sensor_msgs::ImuConstPtr& msg){
  send_imu(msg,"HEAD_IMU");
  //send_imu_as_pose(msg,"POSE_HEAD_ORIENT"); // not necessary, just for easy rendering in bot_frames in viewer 
}


void App::send_imu(const sensor_msgs::ImuConstPtr& msg,string channel ){
  drc::imu_t imu_msg;
  imu_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);  
  //imu_msg.frame_id = msg->header->frame_id;
  imu_msg.orientation[0] = msg->orientation.w; // NB: order here is wxyz
  imu_msg.orientation[1] = msg->orientation.x;
  imu_msg.orientation[2] = msg->orientation.y;
  imu_msg.orientation[3] = msg->orientation.z;
  imu_msg.angular_velocity[0] = msg->angular_velocity.x;
  imu_msg.angular_velocity[1] = msg->angular_velocity.y;
  imu_msg.angular_velocity[2] = msg->angular_velocity.z;
  imu_msg.linear_acceleration[0] = msg->linear_acceleration.x;
  imu_msg.linear_acceleration[1] = msg->linear_acceleration.y;
  imu_msg.linear_acceleration[2] = msg->linear_acceleration.z;
  for (size_t i=0;i<9;i++){
    imu_msg.orientation_covariance[i] = msg->orientation_covariance[i];
    imu_msg.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
    imu_msg.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
  }
  lcm_publish_.publish(channel, &imu_msg);
}

int scan_counter=0;
void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  if (scan_counter%80 ==0){
    ROS_ERROR("LSCAN [%d]", scan_counter );
    //std::cout << "SCAN " << scan_counter << "\n";
  }  
  scan_counter++;
  send_lidar(msg, "SCAN");
}
void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities; // currently all identical
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;

  lcm_publish_.publish(channel.c_str(), &scan_out);
}

int stereo_counter=0;
void App::head_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info){
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  //if ( float(current_utime - head_stereo_last_utime_) > float(1E6/head_stereo_publish_fps_ ) ){
    //std::cout << "send head\n";
    head_stereo_last_utime_ = current_utime;
    publishStereo(l_image,l_cam_info,r_image,r_cam_info,"CAMERA");
    
    if (stereo_counter%30 ==0){
      ROS_ERROR("HDCAM [%d]", stereo_counter );
      //std::cout << "HCAMS " << stereo_counter << "\n";
    }
    stereo_counter++;
  //}else{
    //std::cout << "dont send head\n";
  //}  
}

int lhand_stereo_counter=0;
void App::l_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  if ( float(current_utime -l_hand_stereo_last_utime_) > float(1E6/hand_stereo_publish_fps_ ) ){
    //std::cout << "send l hand\n";
    l_hand_stereo_last_utime_ = current_utime;
    publishStereo(l_image,l_cam_info,r_image,r_cam_info,"CAMERA_LHAND");
    
    if (lhand_stereo_counter%30 ==0){
      ROS_ERROR("LHCAM [%d]", lhand_stereo_counter );
    }
    lhand_stereo_counter++;

  }else{
    //std::cout << "dont send hand l\n";
  }
}

int rhand_stereo_counter=0;
void App::r_hand_stereo_cb(const sensor_msgs::ImageConstPtr& l_image,    const sensor_msgs::CameraInfoConstPtr& l_cam_info,    const sensor_msgs::ImageConstPtr& r_image,    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  if ( float(current_utime - r_hand_stereo_last_utime_) > float(1E6/hand_stereo_publish_fps_ ) ){
    //std::cout << "send r hand\n";
    r_hand_stereo_last_utime_ = current_utime;
    publishStereo(l_image,l_cam_info,r_image,r_cam_info,"CAMERA_RHAND");
    
    if (rhand_stereo_counter%30 ==0){
      ROS_ERROR("RHCAM [%d]", rhand_stereo_counter );
    }    
    rhand_stereo_counter++;
  }else{
    //std::cout << "dont send r hand\n";
  }
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

// display_tic_toc: a helper function which accepts a set of 
// timestamps and displays the elapsed time between them as 
// a fraction and time used [for profiling]
void display_tic_toc(std::vector<int64_t> &tic_toc,const std::string &fun_name){
  int tic_toc_size = tic_toc.size();
  double percent_tic_toc_last = 0;
  double dtime = ((double) (tic_toc[tic_toc_size-1] - tic_toc[0])/1000000);
  cout << "fraction_" << fun_name << ",";  
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";  
}


void App::publishStereo(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info, std::string camera_out){
  
  #if DO_TIMING_PROFILE
    std::vector<int64_t> tic_toc;
    tic_toc.push_back(_timestamp_now());
  #endif
  
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  int isize = l_image->width*l_image->height;
  //cout << l_image->width << " w | h " << l_image->height << "\n";
  //cout << l_image->encoding << " q\n";

  int n_colors=0;
  if (l_image->encoding.compare("mono8") == 0){
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (l_image->width*l_image->height));
    n_colors=1;
  }else if (l_image->encoding.compare("rgb8") == 0){
    copy(l_image->data.begin(), l_image->data.end(), stereo_data);
    copy(r_image->data.begin(), r_image->data.end(), stereo_data + (3*l_image->width*l_image->height));
    n_colors=3;
  }else {
    cout << l_image->encoding << " image encoded not supported, not publishing\n";
    cout << camera_out << "\n";
    return;
  }
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
  #endif
    
  //int64_t tic = _timestamp_now();  
  if (1==1){
    imgutils_->jpegImageThenSend(stereo_data, current_utime, 
                l_image->width, 2*l_image->height, jpeg_quality, camera_out, n_colors );
  }else{
    stereo_msg_out_.utime =current_utime;
    stereo_msg_out_.width =l_image->width;
    stereo_msg_out_.height =2*l_image->height;
    stereo_msg_out_.nmetadata =0;
    stereo_msg_out_.row_stride=l_image->width;
    if (n_colors ==1){
      stereo_msg_out_.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
      stereo_msg_out_.size =2*isize;
    }else if(n_colors ==3){
      stereo_msg_out_.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
      stereo_msg_out_.size =2*n_colors*isize;
    }else{
      std::cout << "Color Error\n"; 
    }
    
    stereo_msg_out_.data.assign(stereo_data, stereo_data + ( 2*isize));
    lcm_publish_.publish(camera_out, &stereo_msg_out_);
  }
  //std::cout << "pub: " <<  float((_timestamp_now() - tic)*1E-6) << "\n";;  
  #if DO_TIMING_PROFILE
    tic_toc.push_back(_timestamp_now());
    display_tic_toc(tic_toc,"stereo");
  #endif    
}



int main(int argc, char **argv){
  bool send_head_cameras = false;
  bool send_hand_cameras = false;
  
  ros::init(argc, argv, "ros2lcm_stereo");
  std::string which_camera;
  if (argc >= 2){
     which_camera = argv[1];
  }else {
    ROS_ERROR("Need to have another arguement in the launch file");
  }
  ros::NodeHandle nh;
  
  std::string  which_transport;
  if (which_camera.compare("head") == 0){
    send_head_cameras = true;
    which_transport = "/ros2lcm_head";
  }else if (which_camera.compare("hand") == 0){
    send_hand_cameras = true;
    which_transport = "/ros2lcm_hand";
  }else {
    ROS_ERROR("which_camera not understood");
    std::cout << which_camera << " is not understood\n";
    exit(-1);
  }

  std::string transport;
  if (nh.getParam(string( which_transport + "/image_transport"), transport)) {
    std::cout << "transport is " << transport << "\n";
  }
  
  ROS_ERROR("Stereo Camera Translator Sleeping: [%s] [%s]", which_camera.c_str() , transport.c_str());
  sleep(4);
  ROS_ERROR("Stereo Camera Translator Ready: [%s] [%s]", which_camera.c_str() , transport.c_str());
  
  App *app = new App(nh, send_head_cameras, send_hand_cameras);
  std::cout << "ros2lcm translator ready\n";
  
  ros::spin();
  return 0;
}