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

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <rosgraph_msgs/Clock.h>

using namespace std;

class App{
public:
  App(const std::string & stereo_in,
      const std::string & stereo_out);
  ~App();

private:
  ros::NodeHandle node_;
  
  // Left and Right Images Seperately:
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber left_image_sub_,right_image_sub_;

  // Combined Stereo Image:
  void stereo_cb(const sensor_msgs::ImageConstPtr& l_image,
      const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::ImageConstPtr& r_image,
      const sensor_msgs::CameraInfoConstPtr& r_cam_info);
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter l_image_sub_, r_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub_, r_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
  const std::string stereo_in_,stereo_out_;

  ros::Subscriber joint_states_sub_;
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);

  ros::Subscriber real_head_scan_sub_;
  void real_head_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
};

App::App(const std::string & stereo_in,
    const std::string & stereo_out):
    stereo_in_(stereo_in),
    stereo_out_(stereo_out),
    it_(node_),
    sync_(10) {
  ROS_INFO("Initializing Translator");

  // Real Multisense SL head:
  real_head_scan_sub_ = node_.subscribe(string("/laser/scan"), 10, &App::real_head_scan_cb,this);

  
  // Mono-Cameras:
  left_image_sub_ = node_.subscribe(string("/stereo/left/image_rect"), 10, &App::left_image_cb,this);
  right_image_sub_ = node_.subscribe(string("/stereo/right/image_rect"), 10, &App::right_image_cb,this);

  joint_states_sub_ = node_.subscribe("/joint_states", 10, &App::joint_states_cb,this);
};

App::~App()  {
}


int stereo_counter=0;
void App::stereo_cb(const sensor_msgs::ImageConstPtr& l_image,
    const sensor_msgs::CameraInfoConstPtr& l_cam_info,
    const sensor_msgs::ImageConstPtr& r_image,
    const sensor_msgs::CameraInfoConstPtr& r_cam_info)
{
  stereo_counter++;
  if (stereo_counter%10 ==0){
    std::cout << stereo_counter << " STEREO\n";
  }
}

int l_counter =0;
int r_counter =0;
int scan_counter=0;
int js_counter=0;
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  l_counter++;
  if (l_counter%30 ==0){
    cout << "got scan "<< scan_counter << " | left " 
                  << l_counter      << " | right " 
                  << r_counter      << " | joints " 
                  << js_counter     <<"\n";
  }
}
void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
  r_counter++;
  if (r_counter%30 ==0){
    cout << "got scan "<< scan_counter << " | left " 
                  << l_counter      << " | right " 
                  << r_counter      << " | joints " 
                  << js_counter     <<"\n";
  }
}

void App::real_head_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  scan_counter++;
  if (scan_counter%80 ==0){
    cout << "got scan "<< scan_counter << " | left " 
                  << l_counter      << " | right " 
                  << r_counter      << " | joints " 
                  << js_counter     <<"\n";
  }
}

void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  js_counter++;
  if (js_counter%160 ==0){
    cout << "got scan "<< scan_counter << " | left " 
                  << l_counter      << " | right " 
                  << r_counter      << " | joints " 
                  << js_counter     <<"\n";
  }
}

int main(int argc, char **argv){
  std::cout << "real_multisense_subscribe launched\n";
  ros::init(argc, argv, "real_multisense_subscribe");
  App *app = new App("camera", "CAMERA");
  ros::spin();
  return 0;
}
