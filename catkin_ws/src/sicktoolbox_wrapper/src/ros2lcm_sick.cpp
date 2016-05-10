// ### Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

// ### Standard includes
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>

#include <lcmtypes/bot_core.hpp>
#include <lcm/lcm-cpp.hpp>


class App
{
public:
  explicit App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

  ros::Subscriber joint_states_sub_;
  void joint_states_cb(const sensor_msgs::LaserScanConstPtr& msg);

  void publishLidar(const sensor_msgs::LaserScanConstPtr& msg, std::string channel);

};

App::App(ros::NodeHandle node_)
{
  ROS_INFO("Initializing Sick Translator");
  if (!lcmPublish_.good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  joint_states_sub_ = node_.subscribe(std::string("/sick_scan"), 100, &App::joint_states_cb,
                                      this);
}

App::~App()
{
}


int scan_counter = 0;
void App::joint_states_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
  if (scan_counter % 80 == 0)
  {
    ROS_ERROR("LSCAN [%d]", scan_counter);
    // std::cout << "SCAN " << scan_counter << "\n";
  }
  scan_counter++;
  publishLidar(msg, "SICK_SCAN");
}

void App::publishLidar(const sensor_msgs::LaserScanConstPtr& msg, std::string channel)
{
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
  scan_out.nranges = msg->ranges.size();
  scan_out.nintensities = msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcmPublish_.publish(channel.c_str(), &scan_out);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm_sick");
  ros::NodeHandle nh;
  new App(nh);
  ROS_INFO_STREAM("ros2lcm_sick translator ready");
  ROS_ERROR_STREAM("ROS2LCM Sick Translator Ready");
  ros::spin();
  return 0;
}
