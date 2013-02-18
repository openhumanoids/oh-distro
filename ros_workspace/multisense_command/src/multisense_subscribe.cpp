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

//#include <lcmtypes/drc_lcmtypes.hpp>


#include <ConciseArgs>


using namespace std;

class App{
public:
  App(bool get_left_and_right_, bool get_left_and_disparity_);
  ~App();

private:
  ros::NodeHandle node_;
  void print_status();
  
  bool get_left_and_right_;
  bool get_left_and_disparity_;
  
  // Left and Right Images Seperately:
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber left_image_sub_;
  int l_counter;
  void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber right_image_sub_;
  int r_counter;
  void disparity_image_cb(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber disparity_image_sub_;
  int disparity_counter;
  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);
  ros::Subscriber joint_states_sub_;
  int scan_counter;

  void real_head_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  ros::Subscriber real_head_scan_sub_;
  int js_counter;
};

App::App(bool get_left_and_right_, bool get_left_and_disparity_):
       get_left_and_right_(get_left_and_right_ ), get_left_and_disparity_( get_left_and_disparity_){
  ROS_INFO("Initializing multisense_subscribe");
  real_head_scan_sub_ = node_.subscribe(string("/laser/scan"), 10, &App::real_head_scan_cb,this);
  if (get_left_and_right_){
    left_image_sub_ = node_.subscribe(string("/stereo/left/image_rect"), 10, &App::left_image_cb,this);
    right_image_sub_ = node_.subscribe(string("/stereo/right/image_rect"), 10, &App::right_image_cb,this);
  }
  if (get_left_and_disparity_){
    disparity_image_sub_ = node_.subscribe(string("/stereo/depth"), 10, &App::disparity_image_cb,this);
  }
  joint_states_sub_ = node_.subscribe("/joint_states", 10, &App::joint_states_cb,this);

  l_counter =0;
  r_counter =0;
  disparity_counter =0;
  scan_counter=0;
  js_counter=0;
};

void App::print_status(){
    cout << "got scan "<< scan_counter << " | left " 
                  << l_counter      << " | right " 
                  << r_counter      << " | disparity " 
                  << disparity_counter      << " | joints " 
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
void App::disparity_image_cb(const sensor_msgs::ImageConstPtr& msg){
  disparity_counter++;
  if (disparity_counter%30 ==0){ print_status();  }
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
  ConciseArgs parser(argc, argv, "lcm2ros");
  bool get_left_and_right =false;
  bool get_left_and_disparity =false;
  parser.add(get_left_and_right, "r", "right", "Get left and right");
  parser.add(get_left_and_disparity, "d", "disparity", "Get left and disparity");
  parser.parse();
  cout << "get_left_and_right: " << get_left_and_right << "\n"; 
  cout << "get_left_and_disparity: " << get_left_and_disparity << "\n"; 
  
  
  std::cout << "multisense_subscribe launched - reports heartbeat of device\n";
  ros::init(argc, argv, "multisense_subscribe");
  App *app = new App(get_left_and_right, get_left_and_disparity);
  ros::spin();
  return 0;
}
