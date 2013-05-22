// Selective ros2lcm translator
// mfallon aug,sept 2012

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
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <atlas_msgs/ForceTorqueSensors.h>

// deprecated:
//#include <atlas_gazebo_msgs/RobotState.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <ros/callback_queue.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <ConciseArgs>

#define DO_TIMING_PROFILE true

#define VERBOSE false
using namespace std;


int width =800;//1024; // hardcoded
int height =800;//544; // hardcoded
uint8_t* stereo_data = new uint8_t [2*3* width*height]; // 2 color scale images stacked
uint8_t* singleimage_data = new uint8_t [2*2* width*height]; // 1 color scale image
int jpeg_quality=90;


class App{
public:
  App(const std::string & stereo_in, const std::string & stereo_out, ros::NodeHandle node_, bool send_hand_cameras_);
  ~App();

private:
  bool send_hand_cameras_; // listen for and publish the hand cameras
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Left and Right Images Seperately:
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );
    
  image_transport::Subscriber left_image_sub_;
  
  image_transport::ImageTransport it_;
  
  const std::string stereo_in_,stereo_out_;
  
  // Image Compression
  image_io_utils*  imgutils_;  
};

App::App(const std::string & stereo_in,
    const std::string & stereo_out,
    ros::NodeHandle node_, bool send_hand_cameras_) :
    stereo_in_(stereo_in), stereo_out_(stereo_out), it_(node_),
    node_(node_), send_hand_cameras_(send_hand_cameras_){
      
      // sync_(10), l_hand_sync_(10), r_hand_sync_(10), it_(node_), 
  ROS_INFO("Initializing Translator");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  
  if (1==1){
    // Mono-Cameras:
    left_image_sub_ = it_.subscribe("/multisense_sl/camera/left/image_rect_color", 1, &App::left_image_cb,this);
    //left_image_sub_ = node_.subscribe( "/multisense_sl/camera/left/image_raw", 10, &App::left_image_cb,this);
    
    imgutils_ = new image_io_utils( lcm_publish_.getUnderlyingLCM(), width, height );
    
  }
  
  std::cout << "only getting left\n";
  return;
  
};

App::~App()  {
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


// typical contents: 640 x 480 images (out of date)
// points: xyz
// channels: rgb u v
int l_counter =0;
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  //std::cout << l_counter << " left image\n";
  
  if (l_counter%30 ==0){
    ROS_ERROR("LEFTC [%d]", l_counter );
    ///std::cout << l_counter << " left image\n";
  }  
  l_counter++;
  
  drc::utime_t utime_msg;
  utime_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  lcm_publish_.publish("TICKER", &utime_msg);  
  
  send_image(msg, "CAMERALEFT");
}

// TODO: jpeg compression:
//       look into openni_utils/openni_ros2rgb for code
// TODO: pre-allocate image_data and retain for speed - when size is known
void App::send_image(const sensor_msgs::ImageConstPtr& msg,string channel ){
  int64_t current_utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  /*cout << msg->width << " " << msg->height << " | "
       << msg->encoding << " is encoding | "
       << current_utime << " | "<< channel << "\n";*/

  int n_colors=3;
  int isize = msg->width*msg->height;

  //ROS_ERROR("Received size [%d]", msg->data.size() );
  
  copy(msg->data.begin(), msg->data.end(), singleimage_data);
  if (1==1){
    imgutils_->jpegImageThenSend(singleimage_data, current_utime, 
                msg->width, msg->height, jpeg_quality, channel, n_colors );
  }else{
    bot_core::image_t lcm_img;
    lcm_img.utime =current_utime;
    lcm_img.width =msg->width;
    lcm_img.height =msg->height;
    lcm_img.nmetadata =0;
    lcm_img.row_stride=n_colors*msg->width;
    lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
    lcm_img.size =n_colors*isize;
    lcm_img.data.assign(singleimage_data, singleimage_data + ( n_colors*isize));
    lcm_publish_.publish(channel.c_str(), &lcm_img);
  }
}


int main(int argc, char **argv){
  bool send_hand_cameras = true;
  
  ros::init(argc, argv, "ros2lcm_camera");
  ros::NodeHandle nh;
  
  App *app = new App( "multisense_sl/camera", "CAMERA", nh, send_hand_cameras);
  std::cout << "ros2lcm translator ready\n";
  ros::spin();
  return 0;
}
