// Copyright 2015 Vladimir Ivan, Maurice Fallon, Wolfgang Merkt

/*
 - Designer will publish:
 http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/JointTrajectory.html
 - Designer will receive: (with root link as 0,0,0)
 http://docs.ros.org/indigo/api/sensor_msgs/html/msg/JointState.html
 */
#include <cstdlib>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include <trajectory_msgs/JointTrajectory.h>
#include <ipab_msgs/PlannerRequest.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

class LCM2ROS
{
public:
  LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in);
  ~LCM2ROS()
  {
  }

private:
  boost::shared_ptr<lcm::LCM> lcm_;
  ros::NodeHandle nh_;
  ros::NodeHandle* rosnode;
  ros::Publisher robot_plan_pub_;
  ros::ServiceClient robot_plan_pause_service_;

  void robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg);
  void robotPlanPauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                             const drc::plan_control_t* msg);
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in) : lcm_(lcm_in), nh_(nh_in)
{
  lcm_->subscribe("COMMITTED_ROBOT_PLAN", &LCM2ROS::robotPlanHandler, this);
  robot_plan_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/kuka/robot_plan", 10);

  lcm_->subscribe("COMMITTED_PLAN_PAUSE", &LCM2ROS::robotPlanPauseHandler, this);
  robot_plan_pause_service_ = nh_.serviceClient<std_srvs::Empty>("stop_trajectory_execution");
}

void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got plan");

  trajectory_msgs::JointTrajectory m;
  m.header.stamp = ros::Time().fromSec(msg->utime * 1E-6);
  m.joint_names = msg->plan[0].joint_name;

  for (int i = 0; i < msg->num_states; i++)
  {
    bot_core::robot_state_t state = msg->plan[i];
    trajectory_msgs::JointTrajectoryPoint point;

    point.positions = std::vector<double>(state.joint_position.begin(), state.joint_position.end());
    point.velocities = std::vector<double>(state.joint_velocity.begin(), state.joint_velocity.end());
    point.accelerations.assign(state.joint_position.size(), 0.0);  // not provided, send zeros
    point.effort = std::vector<double>(state.joint_effort.begin(), state.joint_effort.end());
    point.time_from_start = ros::Duration().fromSec(state.utime * 1E-6);
    m.points.push_back(point);
  }

  robot_plan_pub_.publish(m);
}

void LCM2ROS::robotPlanPauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                    const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS STOP button pressed");

  std_srvs::Empty srv;  // TODO(wxm): change to Trigger
  if (robot_plan_pause_service_.call(srv))
  {
    ROS_ERROR("Successfully stopped trajectory execution");
  }
  else
  {
    ROS_ERROR("Could not stop trajectory execution - DANGER");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lcm2ros", ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh);
  ROS_INFO_STREAM("lcm2ros translator ready");
  ROS_ERROR_STREAM("LCM2ROS Translator Ready");

  while (0 == lcm->handle())
  {
  }

  return 0;
}
