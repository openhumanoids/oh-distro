// Copyright 2015 Vladimir Ivan

/*
 - Designer will publish:
 http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/JointTrajectory.html
 - Designer will receive: (with root link as 0,0,0)
 http://docs.ros.org/indigo/api/sensor_msgs/html/msg/JointState.html
 */
#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"
#include "lcmtypes/drc/affordance_collection_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/exotica_planner_request_t.hpp"
#include "lcmtypes/drc/map_octree_t.hpp"
#include <trajectory_msgs/JointTrajectory.h>
#include <ipab_msgs/PlannerRequest.h>
#include <octomap_msgs/Octomap.h>
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

  ros::Publisher planner_request_pub_;
  ros::Publisher ik_request_pub_;
  ros::Publisher octomap_pub_;

  void plannerRequestHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                             const drc::exotica_planner_request_t* msg);
  void ikRequestHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                        const drc::exotica_planner_request_t* msg);
  void octreeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                        const drc::map_octree_t* msg);
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in) : lcm_(lcm_in), nh_(nh_in)
{
  lcm_->subscribe("PLANNER_REQUEST", &LCM2ROS::plannerRequestHandler, this);
  planner_request_pub_ = nh_.advertise<ipab_msgs::PlannerRequest>("/exotica/planner_request", 10);

  lcm_->subscribe("IK_REQUEST", &LCM2ROS::ikRequestHandler, this);
  ik_request_pub_ = nh_.advertise<ipab_msgs::PlannerRequest>("/exotica/ik_request", 10);

  lcm_->subscribe("MAP_OCTREE", &LCM2ROS::octreeHandler, this);
  octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap_binary", 1);
}

void translatePlannerRequest(const drc::exotica_planner_request_t* msg, ipab_msgs::PlannerRequest& m)
{
  m.header.stamp = ros::Time().fromSec(msg->utime * 1E-6);
  m.poses = msg->poses;
  m.constraints = msg->constraints;
  m.affordances = msg->affordances;
  m.seed_pose = msg->seed_pose;
  m.nominal_pose = msg->nominal_pose;
  m.end_pose = msg->end_pose;
  m.joint_names = msg->joint_names;
  m.options = msg->options;
}

void LCM2ROS::plannerRequestHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                    const drc::exotica_planner_request_t* msg)
{
  ROS_ERROR("LCM2ROS got PLANNER_REQUEST");
  ipab_msgs::PlannerRequest m;
  translatePlannerRequest(msg, m);
  planner_request_pub_.publish(m);
}

void LCM2ROS::ikRequestHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const drc::exotica_planner_request_t* msg)
{
  ROS_ERROR("LCM2ROS got IK_REQUEST");
  ipab_msgs::PlannerRequest m;
  translatePlannerRequest(msg, m);
  ik_request_pub_.publish(m);
}

void LCM2ROS::octreeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const drc::map_octree_t* msg)
{
  octomap_msgs::Octomap m;
  m.header.frame_id = "/world_frame";
  m.header.stamp = ros::Time().fromSec(msg->utime * 1E-6);

  // hard coded because the incoming message has no resolution
  // A better solution might be to also check the view_id if there are
  // multiple octrees being published
  m.id = "OcTree";
  m.resolution = 0.01;
  m.binary = true;
  m.data.resize(msg->num_bytes);
  memcpy(&m.data[0], msg->data.data(), msg->num_bytes);

  octomap_pub_.publish(m);
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
