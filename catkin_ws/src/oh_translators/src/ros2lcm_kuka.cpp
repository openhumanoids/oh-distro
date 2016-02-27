// Copyright 2015 Maurice Fallon, Vladimir Ivan, Wolfgang Merkt
// Selective ros2lcm translator for Edinburgh Kuka Arm

// ### Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ipab_msgs/PlanStatus.h>

// ### Standard includes
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>

// ### LCM
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/drc/robot_plan_w_keyframes_t.hpp"
#include "lcmtypes/drc/plan_status_t.hpp"

// ### Other
#include <Eigen/Dense>

class App
{
public:
  explicit App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcm_publish_;
  ros::NodeHandle node_;

  ros::Subscriber joint_states_sub_, plan_status_sub_;
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);
  void plan_status_cb(const ipab_msgs::PlanStatusConstPtr& msg);
};

App::App(ros::NodeHandle node_)
{
  ROS_INFO_STREAM("Initializing KUKA LWR Joint State Translator");
  if (!lcm_publish_.good())
  {
    ROS_ERROR_STREAM("lcm is not good()");
  }
  joint_states_sub_ = node_.subscribe(std::string("/joint_states"), 100, &App::joint_states_cb, this);
  plan_status_sub_ = node_.subscribe(std::string("/kuka/plan_status"), 100, &App::plan_status_cb, this);
}

App::~App()
{
}

void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg)
{
  int n_joints = msg->position.size();

  bot_core::joint_state_t msg_out;
  msg_out.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec

  msg_out.joint_position.assign(n_joints, 0);
  msg_out.joint_velocity.assign(n_joints, 0);
  msg_out.joint_effort.assign(n_joints, 0);
  msg_out.num_joints = n_joints;
  msg_out.joint_name = msg->name;
  for (int i = 0; i < n_joints; i++)
  {
    msg_out.joint_position[i] = msg->position[i];
    msg_out.joint_velocity[i] = msg->velocity[i];
    msg_out.joint_effort[i] = msg->effort[i];
  }

  lcm_publish_.publish("KUKA_STATE", &msg_out);
}

void App::plan_status_cb(const ipab_msgs::PlanStatusConstPtr& msg)
{
  drc::plan_status_t msg_out;
  msg_out.utime = msg->utime;
  msg_out.execution_status = msg->execution_status;
  msg_out.plan_type = msg->plan_type;
  msg_out.last_plan_msg_utime = msg->last_plan_msg_utime;
  msg_out.last_plan_start_utime = msg->last_plan_start_utime;

  lcm_publish_.publish("PLAN_EXECUTION_STATUS", &msg_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh);
  ROS_INFO_STREAM("ros2lcm KUKA LWR Joint State translator ready");
  ROS_ERROR_STREAM("ROS2LCM KUKA LWR Joint State Translator Ready");
  ros::spin();
  return 0;
}
