// Tool to display the rate of data output from the sensor (via ROS)
// mfallon 2013
#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>

using namespace std;

class App{
public:
  App();
  ~App();

private:
  ros::NodeHandle node_;
  void print_status();
  
  // Left and Right Images Seperately:
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber left_image_sub_,right_image_sub_;
  int l_counter;
  int r_counter;

  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);
  ros::Subscriber joint_states_sub_;
  int scan_counter;

  void real_head_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  ros::Subscriber real_head_scan_sub_;
  int js_counter;
};

App::App(){
  ROS_INFO("Initializing multisense_subscribe");
  real_head_scan_sub_ = node_.subscribe(string("/laser/scan"), 10, &App::real_head_scan_cb,this);
  left_image_sub_ = node_.subscribe(string("/stereo/left/image_rect"), 10, &App::left_image_cb,this);
  right_image_sub_ = node_.subscribe(string("/stereo/right/image_rect"), 10, &App::right_image_cb,this);
  joint_states_sub_ = node_.subscribe("/joint_states", 10, &App::joint_states_cb,this);

  l_counter =0;
  r_counter =0;
  scan_counter=0;
  js_counter=0;
};

void App::print_status(){
    cout << "got scan "<< scan_counter << " | left " 
                  << l_counter      << " | right " 
                  << r_counter      << " | joints " 
                  << js_counter     <<"\n";
}

void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  l_counter++;
  if (l_counter%30 ==0){ print_status();  }
}
void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
  r_counter++;
  if (r_counter%30 ==0){ print_status();  }
}

void App::real_head_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  scan_counter++;
  if (scan_counter%80 ==0){ print_status();  }
}

void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  js_counter++;
  if (js_counter%160 ==0){  print_status();  }
}

int main(int argc, char **argv){
  std::cout << "multisense_subscribe launched - reports heartbeat of device\n";
  ros::init(argc, argv, "multisense_subscribe");
  App *app = new App();
  ros::spin();
  return 0;
}
