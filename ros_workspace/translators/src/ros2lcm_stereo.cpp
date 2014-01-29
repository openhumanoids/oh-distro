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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <opencv2/opencv.hpp>

#define DO_TIMING_PROFILE false
using namespace std;

int width =800;//1024; // hardcoded
int height =800;//544; // hardcoded
uint8_t* stereo_data = new uint8_t [2*3* width*height]; // 2 color scale images stacked
uint8_t* left_image_data = new uint8_t [3* width*height]; // 1 color scale image
uint8_t* right_image_data = new uint8_t [3* width*height]; // 1 color scale image
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
  


  // Rate messages:
  double hand_stereo_publish_fps_, head_stereo_publish_fps_;
  ros::Subscriber head_fps_sub_,hand_fps_sub_;
  void head_fps_cb(const std_msgs::Float64ConstPtr& msg);
  void hand_fps_cb(const std_msgs::Float64ConstPtr& msg);  
  
  // Combined Stereo Image:
  multisense::images_t multisense_msg_out_;
  bot_core::image_t lcm_left_;
  bot_core::image_t lcm_right_;
  bot_core::image_t lcm_disp_;
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
  
  
  // Image Compression
  image_io_utils*  imgutils_;
  
  bool do_jpeg_compress_;
  int jpeg_quality_;
  bool do_zlib_compress_;
  int depth_compress_buf_size_;
  uint8_t* depth_compress_buf_;
  int color_compress_buffer_size_;
  uint8_t* color_compress_buffer_;  
};

App::App(ros::NodeHandle node_, bool send_head_cameras_, bool send_hand_cameras_) :
    send_head_cameras_(send_head_cameras_), send_hand_cameras_(send_hand_cameras_), node_(node_), it_(node_), sync_(10),l_hand_sync_(10), r_hand_sync_(10){
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  imgutils_ = new image_io_utils( lcm_publish_.getUnderlyingLCM(), width, height );
  
  if (send_head_cameras_){
    // LASER is now in the ros2lcm.cpp
    
    // This is done by directly requesting from Gazebo at lower rate:
    head_stereo_publish_fps_ = 1.0;
    //head_fps_sub_ = node_.subscribe("/mit/set_head_fps", 100, &App::head_fps_cb,this);
    
    // Stereo Image:
    std::string lim_string ,lin_string,rim_string,rin_string;
    std::string head_stereo_root = "/multisense_sl/camera";
    // Grey:
    lim_string = head_stereo_root + "/left/image_rect_color"; // was image_rect for vrc
    lin_string = head_stereo_root + "/left/camera_info";
    rim_string = head_stereo_root + "/right/image_rect_color";// was image_rect for vrc
    rin_string = head_stereo_root + "/right/camera_info";
    cout << lim_string << " is the left stereo image subscription [for stereo]\n";
    l_image_sub_.subscribe(it_, ros::names::resolve( lim_string ), 30);
    l_info_sub_.subscribe(node_, ros::names::resolve( lin_string ), 30);
    r_image_sub_.subscribe(it_, ros::names::resolve( rim_string ), 30);
    r_info_sub_.subscribe(node_, ros::names::resolve( rin_string ), 30);
    sync_.connectInput(l_image_sub_, l_info_sub_, r_image_sub_, r_info_sub_);
    sync_.registerCallback( boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4) );
    head_stereo_last_utime_=0;
    
    multisense_msg_out_.images.push_back( lcm_left_);
    multisense_msg_out_.images.push_back( lcm_disp_);
    multisense_msg_out_.image_types.push_back(0);// LEFT
    multisense_msg_out_.image_types.push_back(2);// multisense::images_t::DISPARITY );
    //stereo_msg_out.images.push_back( lcm_right_);
    //stereo_msg_out.image_types.push_back(1);// multisense::images_t::RIGHT );    
    
    
    // allocate space for zlib compressing depth data
    color_compress_buffer_size_ = 800 * 800 * sizeof(int8_t) * 10;
    color_compress_buffer_= new uint8_t[color_compress_buffer_size_];  // x4 was used for zlib in kinect_lcm
    
    
    depth_compress_buf_size_ = 800 * 800 * sizeof(int8_t) * 10;
    depth_compress_buf_ = (uint8_t*) malloc(depth_compress_buf_size_);
    do_jpeg_compress_=true;
    jpeg_quality_ = 94;
    do_zlib_compress_ = true;      
    
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
        
  }
  
};

App::~App()  {
}


void App::hand_fps_cb(const std_msgs::Float64ConstPtr& msg){
  hand_stereo_publish_fps_ = double( msg->data);
  ROS_ERROR("ROS2LCM Stereo: set output fps to : %f", hand_stereo_publish_fps_ );  
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

  if (l_image->encoding.compare("rgb8") != 0){
    cout << l_image->encoding << " " << camera_out << " | encoded not supported, not publishing\n";
    return;
  }
  
  int64_t current_utime = (int64_t) floor(l_image->header.stamp.toNSec()/1000);
  int isize = l_image->width*l_image->height;
  int n_colors=3;
  
  /// Left:
  lcm_left_.utime = current_utime;
  lcm_left_.width =l_image->width;
  lcm_left_.height =l_image->height;
  lcm_left_.nmetadata =0;
  lcm_left_.row_stride=n_colors*l_image->width;
  if (do_jpeg_compress_){
    int jpeg_compressed_size =  isize*n_colors;//image_buf_size;
    jpeg_compress_8u_rgb  (l_image->data.data(), l_image->width, l_image->height, l_image->width*n_colors, color_compress_buffer_ , &jpeg_compressed_size, jpeg_quality_);
    lcm_left_.data.resize( jpeg_compressed_size);
    memcpy(&lcm_left_.data[0], color_compress_buffer_ , jpeg_compressed_size);
    lcm_left_.size = jpeg_compressed_size;
    lcm_left_.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG; 
  }else{
    lcm_left_.data.resize(  n_colors*isize);
    memcpy(&lcm_left_.data[0], l_image->data.data() , n_colors*isize);
    lcm_left_.size = n_colors*isize;    
    lcm_left_.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
  }
  
  // No disparity is being computed, so will create some 
  // fake data here for compatiablity:
  lcm_disp_.utime = current_utime;
  lcm_disp_.width =l_image->width;
  lcm_disp_.height =l_image->height;
  lcm_disp_.nmetadata =0;
  lcm_disp_.row_stride=2*l_image->width;  // 2 bytes per pixel
  if (1==1){ 
    /*
    int uncompressed_size = isize;
    // Insert proper compression here if needed:
    unsigned long compressed_size = depth_compress_buf_size_;
    compress2( depth_compress_buf_, &compressed_size, (const Bytef*) imageDataP, uncompressed_size,
            Z_BEST_SPEED);
    lcm_disp_.data.resize(compressed_size);
    */
    unsigned long zlib_compressed_size = 1000; // fake compressed size
    lcm_disp_.data.resize( zlib_compressed_size);
    lcm_disp_.size = zlib_compressed_size;
    multisense_msg_out_.image_types[1] = 5;// multisense::images_t::DISPARITY_ZIPPED );
  }else{
    lcm_disp_.data.resize( 2*isize);
    lcm_disp_.size =2*isize; 
    multisense_msg_out_.image_types[1] = 2;// multisense::images_t::DISPARITY );
  } 

  multisense_msg_out_.images[0] = lcm_left_;
  multisense_msg_out_.images[1] = lcm_disp_;
  multisense_msg_out_.n_images = 2;//2;// stereo_msg_out_.images.size();
  multisense_msg_out_.utime =current_utime;
  lcm_publish_.publish("CAMERA",&multisense_msg_out_);

/* Required for right image transmission:
 *     lcm_right_.pixelformat =bot_core::image_t::PIXEL_FORMAT_GRAY;
    lcm_right_.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    
  lcm_right_.data.resize(  n_colors*isize);
  memcpy(&lcm_right_.data[0], r_image->data.data() , n_colors*isize);
  lcm_right_.size = n_colors*isize;
  
  lcm_right_.utime = current_utime;
  lcm_right_.width =r_image->width;
  lcm_right_.height =r_image->height;
  lcm_right_.nmetadata =0;
  lcm_right_.row_stride=n_colors*r_image->width; 
*/
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
  
  new App(nh, send_head_cameras, send_hand_cameras);
  std::cout << "ros2lcm translator ready\n";
  
  ros::spin();
  return 0;
}
