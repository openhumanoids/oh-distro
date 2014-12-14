#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/drc/walking_plan_t.hpp"
#include "lcmtypes/drc/walking_plan_request_t.hpp"
#include "lcmtypes/drc/footstep_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"

#include "lcmtypes/valkyrie/com_height_packet_message_t.hpp"
#include "lcmtypes/valkyrie/pause_command_message_t.hpp"
#include "lcmtypes/valkyrie/hand_pose_packet_message_t.hpp"

#include <ihmc_msgs/Point2dMessage.h>
#include <ihmc_msgs/FootstepDataListMessage.h>
#include <ihmc_msgs/ComHeightPacketMessage.h>
#include <ihmc_msgs/PauseCommandMessage.h>
#include <ihmc_msgs/HandPosePacketMessage.h>

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
    ros::NodeHandle* rosnode;    

    void footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg);
    ros::Publisher walking_plan_pub_;
    ihmc_msgs::FootstepDataMessage createFootStepList(int foot_to_start_with, double x_pos, double y_pos, double z_pos, double orient_w, double orient_x, double orient_y, double orient_z);
    void sendBasicSteps();

    void comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::com_height_packet_message_t* msg);
    ros::Publisher com_height_pub_;

    void pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::pause_command_message_t* msg);
    void stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
    ros::Publisher pause_pub_;

    void handPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::hand_pose_packet_message_t* msg);
    ros::Publisher hand_pose_pub_;
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST",&LCM2ROS::footstepPlanHandler, this);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListMessage>("/atlas/inputs/ihmc_msgs/FootstepDataListMessage",10);

  lcm_->subscribe("VAL_COMMAND_COM_HEIGHT",&LCM2ROS::comHeightHandler, this);
  com_height_pub_ =  nh_.advertise<ihmc_msgs::ComHeightPacketMessage>("/atlas/inputs/ihmc_msgs/ComHeightPacketMessage",10);

  lcm_->subscribe("VAL_COMMAND_PAUSE",&LCM2ROS::pauseHandler, this);
  lcm_->subscribe("STOP_WALKING",&LCM2ROS::stopHandler, this); // from drake-designer
  pause_pub_ =  nh_.advertise<ihmc_msgs::PauseCommandMessage>("/atlas/inputs/ihmc_msgs/PauseCommandMessage",10);

  lcm_->subscribe("VAL_COMMAND_HAND_POSE",&LCM2ROS::handPoseHandler, this);
  hand_pose_pub_ =  nh_.advertise<ihmc_msgs::HandPosePacketMessage>("/atlas/inputs/ihmc_msgs/HandPosePacketMessage",10);


  rosnode = new ros::NodeHandle();
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
  mout.transferTime = 1.0;
  mout.swingTime = 1.0;
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
  mout.transferTime = 1.2;
  mout.swingTime = 1.2;
  mout.trajectoryWaypointGenerationMethod = 0;
  for (int i=2; i < msg->footstep_plan.num_steps; i++){ // skip the first two standing steps
    drc::footstep_t s = msg->footstep_plan.footsteps[i];
    mout.footstepDataList.push_back( createFootStepList(s.is_right_foot , s.pos.translation.x, s.pos.translation.y, s.pos.translation.z, 
                                                        s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z) );    
  } 
  walking_plan_pub_.publish(mout);
}


void LCM2ROS::comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::com_height_packet_message_t* msg) {
  ROS_ERROR("LCM2ROS got com height");
  ihmc_msgs::ComHeightPacketMessage mout;
  mout.MIN_COM_HEIGHT = msg->min_com_height;
  mout.MAX_COM_HEIGHT = msg->max_com_height;
  mout.heightOffset = msg->height_offset;
  com_height_pub_.publish(mout);
}

void LCM2ROS::pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::pause_command_message_t* msg) {
  ROS_ERROR("LCM2ROS got pause %d", (int) msg->pause);
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = msg->pause;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg) {
  ROS_ERROR("LCM2ROS got drake-designer - sending pause=true");
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = true;
  pause_pub_.publish(mout);
}

void LCM2ROS::handPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::hand_pose_packet_message_t* msg) {
  ROS_ERROR("LCM2ROS got handPose packet");
  ihmc_msgs::HandPosePacketMessage mout;
  mout.robotSide = msg->robot_side;
  mout.dataType = msg->data_type;
  mout.referenceFrame = msg->reference_frame;
  mout.toHomePosition = msg->to_home_position;
  mout.position = msg->position;
  mout.orientation = msg->orientation;
  mout.trajectoryTime = msg->trajectoryTime;
  mout.jointAngles = msg->joint_angles;
  hand_pose_pub_.publish(mout);
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
