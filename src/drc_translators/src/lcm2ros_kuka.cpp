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
#include "lcmtypes/drc/planner_request_t.hpp"
#include <trajectory_msgs/JointTrajectory.h>
#include <ipab_msgs/PlannerRequest.h>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    ros::NodeHandle* rosnode;

    void robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg);
    ros::Publisher robot_plan_pub_;
    void plannerRequestHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::planner_request_t* msg);
    ros::Publisher planner_request_pub_;

};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  lcm_->subscribe("COMMITTED_ROBOT_PLAN",&LCM2ROS::robotPlanHandler, this);
  robot_plan_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/kuka/robot_plan",10);

  lcm_->subscribe("PLANNER_REQUEST",&LCM2ROS::plannerRequestHandler, this);
  planner_request_pub_ = nh_.advertise<ipab_msgs::PlannerRequest>("/planner_request",10);
  rosnode = new ros::NodeHandle();
}

void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg) {
  ROS_ERROR("LCM2ROS got plan");

  trajectory_msgs::JointTrajectory m;
  m.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  m.joint_names = msg->plan[0].joint_name;

  for (int i=0; i < msg->num_states; i++){
    drc::robot_state_t state = msg->plan[i];
    trajectory_msgs::JointTrajectoryPoint point;

    point.positions =     std::vector<double>(state.joint_position.begin(), state.joint_position.end());
    point.velocities = std::vector<double>(state.joint_velocity.begin(), state.joint_velocity.end());
    point.accelerations.assign ( state.joint_position.size()   ,0.0); // not provided, send zeros
    point.effort = std::vector<double>(state.joint_effort.begin(), state.joint_effort.end());;
    point.time_from_start = ros::Duration().fromSec(state.utime*1E-6);
    m.points.push_back(point);
  }

  robot_plan_pub_.publish(m);
}


void LCM2ROS::plannerRequestHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::planner_request_t* msg) {
  ROS_ERROR("LCM2ROS got PLANNER_REQUEST");
  ipab_msgs::PlannerRequest m;
  m.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  m.poses = msg->poses;
  m.constraints = msg->constraints;
  planner_request_pub_.publish(m);
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh);
  cout << "\nlcm2ros translator ready\n";
  ROS_ERROR("LCM2ROS Translator Ready");

  while(0 == lcm->handle());
  return 0;
}
