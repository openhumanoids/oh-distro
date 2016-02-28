// Copyright 2015 Vladimir Ivan, Yiming Yang

// ### Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <ipab_msgs/PlannerResponse.h>

// ### Standard libraries
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

// ### Other
#include <Eigen/Dense>
#include <kdl/frames.hpp>

class App
{
public:
  explicit App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcm_publish_;
  ros::NodeHandle node_;

  ros::Subscriber traj_sub_;
  ros::Subscriber ik_sub_;
  void traj_cb(const ipab_msgs::PlannerResponseConstPtr& msg);
  void ik_cb(const ipab_msgs::PlannerResponseConstPtr& msg);
};

App::App(ros::NodeHandle node_)
{
  ROS_INFO_STREAM("Initializing LCM EXOTica Translator");
  if (!lcm_publish_.good())
  {
    ROS_ERROR_STREAM("lcm is not good()");
  }

  traj_sub_ = node_.subscribe(std::string("/exotica/robot_plan"), 100, &App::traj_cb, this);
  ik_sub_ = node_.subscribe(std::string("/exotica/robot_ik"), 100, &App::ik_cb, this);
}

App::~App()
{
}

void App::traj_cb(const ipab_msgs::PlannerResponseConstPtr& msg)
{
  drc::robot_plan_w_keyframes_t msg_out;
  msg_out.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
  msg_out.robot_name = msg->robot;
  int T = msg->points.size();
  msg_out.num_states = T;
  msg_out.num_keyframes = 0;
  msg_out.num_breakpoints = 0;
  msg_out.is_keyframe.assign(T, false);
  msg_out.is_breakpoint.assign(T, false);
  msg_out.plan_info.assign(T, msg->info);
  msg_out.num_grasp_transitions = 0;
  msg_out.num_bytes = 0;
  msg_out.plan.resize(T);
  for (int t = 0; t < T; t++)
  {
    msg_out.plan[t].utime = msg->points[t].time_from_start.toNSec() / 1000;
    int n_joints = msg->joint_names.size();
    msg_out.plan[t].joint_position.assign(n_joints, 0);
    msg_out.plan[t].joint_velocity.assign(n_joints, 0);
    msg_out.plan[t].joint_effort.assign(n_joints, 0);

    msg_out.plan[t].num_joints = n_joints;
    msg_out.plan[t].joint_name = msg->joint_names;

    for (int i = 0; i < n_joints; i++)
    {
      msg_out.plan[t].joint_position[i] = msg->points[t].positions[i];
      // Ignoring velocity and effort for now
    }

    //  Actually, this applies to all base types
    // if(msg->base_type == 10)
    {
      msg_out.plan[t].pose.translation.x = msg->points[t].positions[0];
      msg_out.plan[t].pose.translation.y = msg->points[t].positions[1];
      msg_out.plan[t].pose.translation.z = msg->points[t].positions[2];
      KDL::Rotation base_rot = KDL::Rotation::RPY(msg->points[t].positions[3], msg->points[t].positions[4],
                                                  msg->points[t].positions[5]);
      base_rot.GetQuaternion(msg_out.plan[t].pose.rotation.x, msg_out.plan[t].pose.rotation.y,
                             msg_out.plan[t].pose.rotation.z, msg_out.plan[t].pose.rotation.w);
    }
  }

  lcm_publish_.publish("CANDIDATE_MANIP_PLAN", &msg_out);
}

void App::ik_cb(const ipab_msgs::PlannerResponseConstPtr& msg)
{
  drc::robot_plan_w_keyframes_t msg_out;
  msg_out.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
  msg_out.robot_name = msg->robot;
  int T = msg->points.size();
  msg_out.num_states = T;
  msg_out.num_keyframes = 0;
  msg_out.num_breakpoints = 0;
  msg_out.is_keyframe.assign(T, false);
  msg_out.is_breakpoint.assign(T, false);
  msg_out.plan_info.assign(T, msg->info);
  msg_out.num_grasp_transitions = 0;
  msg_out.num_bytes = 0;
  msg_out.plan.resize(T);
  for (int t = 0; t < T; t++)
  {
    msg_out.plan[t].utime = msg->points[t].time_from_start.toNSec() / 1000;
    int n_joints = msg->joint_names.size();
    msg_out.plan[t].joint_position.assign(n_joints, 0);
    msg_out.plan[t].joint_velocity.assign(n_joints, 0);
    msg_out.plan[t].joint_effort.assign(n_joints, 0);
    msg_out.plan[t].pose.rotation.w = 1.0;
    msg_out.plan[t].num_joints = n_joints;
    msg_out.plan[t].joint_name = msg->joint_names;
    for (int i = 0; i < n_joints; i++)
    {
      msg_out.plan[t].joint_position[i] = msg->points[t].positions[i];
      // Ignoring velocity and effort for now
    }
    if (msg->base_type == 10)
    {
      msg_out.plan[t].pose.translation.x = msg->points[t].positions[0];
      msg_out.plan[t].pose.translation.y = msg->points[t].positions[1];
      msg_out.plan[t].pose.translation.z = msg->points[t].positions[2];
      KDL::Rotation base_rot = KDL::Rotation::RPY(msg->points[t].positions[3], msg->points[t].positions[4],
                                                  msg->points[t].positions[5]);
      base_rot.GetQuaternion(msg_out.plan[t].pose.rotation.x, msg_out.plan[t].pose.rotation.y,
                             msg_out.plan[t].pose.rotation.z, msg_out.plan[t].pose.rotation.w);
    }
  }

  lcm_publish_.publish("CANDIDATE_MANIP_IKPLAN", &msg_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh);
  ROS_INFO_STREAM("ros2lcm translator ready");
  ROS_ERROR_STREAM("ROS2LCM Joint State Translator Ready");
  ros::spin();
  return 0;
}
