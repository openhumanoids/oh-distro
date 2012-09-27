// Selective ros2lcm translator
//
// As of August 2012:
// - tilting laser
// - base_laser
// - some tf poses  - as rigid_transform_t
// - odometery - as rigid_transform_t
// - RGB stills from wide stereo
// - Point clouds from wide stereo
//
// mfallon aug 2012
#include "cv.h"
#include "highgui.h"


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/message_filter.h>

#include <lcm/lcm-cpp.hpp>
//#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/visualization.h>

#define VERBOSE false
using namespace std;

class App{
  public:
    App();
    ~App();

    lcm_t* get_lcmref(){return lcmref_;}

    double left_time;
    double right_time;

  private:
    lcm_t* lcmref_ ;

    ros::NodeHandle node_;
    tf::Transformer transformer;

    uint8_t* image_data;

    ros::Subscriber tilt_scan_sub_, base_scan_sub_, odom_sub_, tf_sub_;
    ros::Subscriber clock_sub_,left_image_sub_,right_image_sub_,points_sub_;
    void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
    void right_image_cb(const sensor_msgs::ImageConstPtr& msg);
};

App::App(){
  lcmref_ = lcm_create(NULL);

  left_image_sub_ = node_.subscribe(string("/wide_stereo/left/image_raw"), 10, &App::left_image_cb,this);
  right_image_sub_ = node_.subscribe(string("/wide_stereo/right/image_raw"), 10, &App::right_image_cb,this);
};

App::~App()  {
}



// typical contents: 640 x 480 images
// points: xyz
// channels: rgb u v
void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  left_time = msg->header.stamp.toSec();
//  std::cout << "here right\n";
  double diff =  right_time - left_time;
  std::cout << "L: "<< msg->header.stamp.toSec() << " " << diff  << " [RAW]\n";

}
void App::right_image_cb(const sensor_msgs::ImageConstPtr& msg){
  right_time = msg->header.stamp.toSec();
//  std::cout << "here right\n";
  double diff =  right_time - left_time;
  std::cout << "R: "<<msg->header.stamp.toSec() << " " << diff  << " [RAW]\n";


}



int main(int argc, char **argv){
  std::cout << "ros2lcm_translator launched\n";
  ros::init(argc, argv, "ros2lcm_translator");
  App *app = new App();
  ros::spin();
  return 0;
}
