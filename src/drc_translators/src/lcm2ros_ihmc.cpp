#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/drc/walking_plan_t.hpp"
#include "lcmtypes/drc/walking_plan_request_t.hpp"
#include "lcmtypes/drc/footstep_plan_t.hpp"

#include <ihmc_msgs/Point2dMessage.h>
#include <ihmc_msgs/FootstepDataListMessage.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <map>

#define LEFT 0
#define RIGHT 1

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;

    void poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
    void footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg);
    ros::Publisher pose_body_pub_;
    ros::Publisher walking_plan_pub_;
    ros::NodeHandle* rosnode;    


    ihmc_msgs::FootstepDataMessage createFootStepList(int foot_to_start_with, double x_pos, double y_pos, double z_pos, double orient_w, double orient_x, double orient_y, double orient_z);
    void sendBasicSteps();
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  lcm_->subscribe("POSE_TEMP_DISABLED",&LCM2ROS::poseBodyHandler, this);
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST",&LCM2ROS::footstepPlanHandler, this);

  pose_body_pub_ = nh_.advertise<nav_msgs::Odometry>("/pose_body",10);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListMessage>("/atlas/inputs/ihmc_msgs/FootstepDataListMessage",10);
  rosnode = new ros::NodeHandle();
}


void LCM2ROS::poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg) {
  //ROS_ERROR("LCM2ROS got pose_t");
  nav_msgs::Odometry mout;
  mout.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  mout.pose.pose.position.x = msg->pos[0];
  mout.pose.pose.position.y = msg->pos[1];
  mout.pose.pose.position.z = msg->pos[2];
  mout.pose.pose.orientation.w = msg->orientation[0];
  mout.pose.pose.orientation.x = msg->orientation[1];
  mout.pose.pose.orientation.y = msg->orientation[2];
  mout.pose.pose.orientation.z = msg->orientation[3];
  mout.twist.twist.linear.x = msg->vel[0];
  mout.twist.twist.linear.y = msg->vel[1];
  mout.twist.twist.linear.z = msg->vel[2];
  mout.twist.twist.angular.x = msg->rotation_rate[0];
  mout.twist.twist.angular.y = msg->rotation_rate[1];
  mout.twist.twist.angular.z = msg->rotation_rate[2];
  pose_body_pub_.publish(mout);
}


ihmc_msgs::FootstepDataMessage LCM2ROS::createFootStepList(int foot_to_start_with, double x_pos, double y_pos, double z_pos, double orient_w, double orient_x, double orient_y, double orient_z){
  ihmc_msgs::FootstepDataMessage footStepList;
  footStepList.robotSide = foot_to_start_with;
  footStepList.location.x = x_pos;
  footStepList.location.y = y_pos;
  footStepList.location.z = z_pos;
  footStepList.orientation.w = orient_w;
  footStepList.orientation.x = orient_x;
  footStepList.orientation.y = orient_y;
  footStepList.orientation.z = orient_z;
  return footStepList;
}

void LCM2ROS::sendBasicSteps(){
  ihmc_msgs::FootstepDataListMessage mout;
  //mout.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  mout.transferTime = 1.5;
  mout.swingTime = 1.5;
  mout.trajectoryWaypointGenerationMethod = 0;
  mout.footstepDataList.push_back( createFootStepList(LEFT , 0,  0.12,   0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(RIGHT, 0, -0.12,   0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(LEFT , 0.1,  0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(RIGHT, 0.1, -0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(LEFT , 0.2,  0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(RIGHT, 0.2, -0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(LEFT , 0.1,  0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(RIGHT, 0.1, -0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(LEFT , 0  ,  0.12, 0, 1,0,0,0) );
  mout.footstepDataList.push_back( createFootStepList(RIGHT, 0  , -0.12, 0, 1,0,0,0) );
  walking_plan_pub_.publish(mout);
}


void LCM2ROS::footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg) {
  ROS_ERROR("LCM2ROS got plan");
  // sendBasicSteps();

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transferTime = 1.5;
  mout.swingTime = 1.5;
  mout.trajectoryWaypointGenerationMethod = 0;

  for (int i=2; i < msg->footstep_plan.num_steps; i++){ // skip the first two standing steps
    drc::footstep_t s = msg->footstep_plan.footsteps[i];
    mout.footstepDataList.push_back( createFootStepList(s.is_right_foot , s.pos.translation.x, s.pos.translation.y, s.pos.translation.z, s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z) );    
  } 
  walking_plan_pub_.publish(mout);


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
