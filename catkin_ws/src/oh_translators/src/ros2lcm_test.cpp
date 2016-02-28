// Copyright 2015 Maurice Fallon, tbd

// test tool for message spacing in ROS

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc/behavior_t.hpp"

#include <tf/transform_listener.h>

struct Joints
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<std::string> name;
};

int mode;
#define MODE_MAIN 0
#define MODE_ALL 1

class App
{
public:
  explicit App(ros::NodeHandle node_in);
  ~App();

private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

//  tf::TransformListener listener_;

  ros::Subscriber jointStatesSub_;
  ros::Subscriber headJointStatesSub_;
  ros::Subscriber poseSub_;
  ros::Subscriber laserScanSub_;
  ros::Subscriber imuBatchSub_;
  ros::Subscriber leftFootSensorSub_;
  ros::Subscriber rightFootSensorSub_;
  ros::Subscriber behaviorSub_;

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void headJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void poseCallBack(const nav_msgs::OdometryConstPtr& msg);
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void leftFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg);
  void rightFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg);
  void behaviorCallback(const std_msgs::Int32ConstPtr& msg);
};

App::App(ros::NodeHandle node_in) :
    node_(node_in)
{
  ROS_INFO("Initializing Translator");
  if (!lcmPublish_.good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  int queue_size = 100;

  poseSub_ = node_.subscribe(std::string("/ihmc_ros/atlas/output/robot_pose"), queue_size, &App::poseCallBack, this);
  jointStatesSub_ = node_.subscribe(std::string("/ihmc_ros/atlas/output/joint_states"), queue_size,
                                    &App::jointStatesCallback, this);
  // imuBatchSub_ = node_.subscribe(std::string("/ihmc_ros/atlas/output/batch_raw_imu"),
  // queue_size, &App::imuBatchCallback,this);

  if (mode == MODE_ALL)
  {
    headJointStatesSub_ = node_.subscribe(std::string("/multisense/joint_states"), queue_size,
                                          &App::headJointStatesCallback, this);
    leftFootSensorSub_ = node_.subscribe(std::string("/ihmc_ros/atlas/output/foot_force_sensor/left"), queue_size,
                                          &App::leftFootSensorCallback, this);
    rightFootSensorSub_ = node_.subscribe(std::string("/ihmc_ros/atlas/output/foot_force_sensor/right"), queue_size,
                                          &App::rightFootSensorCallback, this);
    // using previously used queue_size for scan:
    behaviorSub_ = node_.subscribe(std::string("/ihmc_ros/atlas/output/behavior"), 100, &App::behaviorCallback, this);
    laserScanSub_ = node_.subscribe(std::string("/multisense/lidar_scan"), 100, &App::laserScanCallback, this);
  }
}

App::~App()
{
}

void App::poseCallBack(const nav_msgs::OdometryConstPtr& msg)
{
  int64_t utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
  std::cerr << utime << "" << std::endl;
}

void App::jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  int64_t utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
  std::cerr << "\t\t\t" << utime << "" << std::endl;
}

void App::headJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
}

void App::leftFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg)
{
}

void App::rightFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg)
{
}

void App::behaviorCallback(const std_msgs::Int32ConstPtr& msg)
{
}

void App::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
}

int main(int argc, char **argv)
{
  mode = MODE_ALL;

  std::string mode_argument;
  if (argc >= 2)
  {
    mode_argument = argv[1];
  }
  else
  {
    ROS_ERROR("Need to have another argument in the launch file");
  }

  if (mode_argument.compare("mode_main") == 0)
  {
    mode = MODE_MAIN;
  }
  else if (mode_argument.compare("mode_all") == 0)
  {
    mode = MODE_ALL;
  }
  else
  {
    ROS_ERROR("mode_argument not understood");
    std::cout << mode_argument << " is not understood\n";
    exit(-1);
  }

  ros::init(argc, argv, "ros2lcm_test");
  ros::NodeHandle nh;
  new App(nh);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Translator Ready (test)");
  ros::spin();
  return 0;
}
