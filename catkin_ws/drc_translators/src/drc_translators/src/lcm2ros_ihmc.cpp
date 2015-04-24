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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds

#define LEFT 0
#define RIGHT 1

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string robotName_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    ros::NodeHandle* node_;
    string robotName_;

    void footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg);
    void footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::footstep_plan_t* msg);
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

    pronto_vis* pc_vis_;
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string robotName_):
    lcm_(lcm_),nh_(nh_), robotName_(robotName_) {
  // If pronto is running never send plans like this:
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST",&LCM2ROS::footstepPlanHandler, this);
    // COMMITTED_FOOTSTEP_PLAN is creating in Pronto frame and transformed into BDI/IHMC frame:
  lcm_->subscribe("BDI_ADJUSTED_FOOTSTEP_PLAN",&LCM2ROS::footstepPlanBDIModeHandler, this);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListMessage>("/ihmc_ros/" + robotName_ + "/control/footstep_list",10);

  lcm_->subscribe("VAL_COMMAND_COM_HEIGHT",&LCM2ROS::comHeightHandler, this);
  com_height_pub_ =  nh_.advertise<ihmc_msgs::ComHeightPacketMessage>("/ihmc_ros/" + robotName_ + "/control/com_height",10);

  lcm_->subscribe("VAL_COMMAND_PAUSE",&LCM2ROS::pauseHandler, this);
  lcm_->subscribe("STOP_WALKING",&LCM2ROS::stopHandler, this); // from drake-designer
  pause_pub_ =  nh_.advertise<ihmc_msgs::PauseCommandMessage>("/ihmc_ros/" + robotName_ + "/control/pause_footstep_exec",10);

  lcm_->subscribe("VAL_COMMAND_HAND_POSE",&LCM2ROS::handPoseHandler, this);
  hand_pose_pub_ =  nh_.advertise<ihmc_msgs::HandPosePacketMessage>("/ihmc_ros/" + robotName_ + "/control/hand_pose",10);

  node_ = new ros::NodeHandle();

  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9995,"Output step positions",5,1) );

}

ihmc_msgs::FootstepDataMessage LCM2ROS::createFootStepList(int foot_to_start_with, double x_pos, double y_pos, double z_pos, double orient_w, double orient_x, double orient_y, double orient_z){
  ihmc_msgs::FootstepDataMessage footStepList;
  footStepList.robot_side = foot_to_start_with;
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
  mout.transfer_time = 1.0;
  mout.swing_time = 1.0;
  //mout.trajectoryWaypointGenerationMethod = 0;
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0,  0.12,   0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0, -0.12,   0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0.1,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0.1, -0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0.2,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0.2, -0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0.1,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0.1, -0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0  ,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0  , -0.12, 0, 1,0,0,0) );
  walking_plan_pub_.publish(mout);
}


Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {

  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  // Post DRC Trails: replace these with Eigen's own conversions
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }

  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

void LCM2ROS::footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg) {
  ROS_ERROR("LCM2ROS got WALKING_CONTROLLER_PLAN_REQUEST (non-pronto and drake mode)");
  // sendBasicSteps();

  // Note:
  // TODO: remove pc_vis_

  std::vector< Eigen::Isometry3d > steps;
  std::vector<Isometry3dTime> stepsT;
  for (int i=0; i < msg->footstep_plan.num_steps; i++){ // skip the first two standing steps
      drc::footstep_t s = msg->footstep_plan.footsteps[i];

      Eigen::Isometry3d step;
      step.setIdentity();
      step.translation().x() = s.pos.translation.x;
      step.translation().y() = s.pos.translation.y;
      step.translation().z() = s.pos.translation.z;
      step.rotate(Eigen::Quaterniond(s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z));

      if(robotName_.compare("valkyrie")==0){
        ROS_ERROR("LCM2ROS flipping valkyrie foot orientations");
        if (s.is_right_foot){
          Eigen::Isometry3d fix_transform;
          fix_transform.setIdentity();
          fix_transform.translation().x() = 0;
          fix_transform.translation().y() = 0;
          fix_transform.translation().z() = 0;
          fix_transform.rotate(euler_to_quat( 0*M_PI/180, -90*M_PI/180, 0 )) ;
          step = step*fix_transform;
        }else{
          Eigen::Isometry3d fix_transform;
          fix_transform.setIdentity();
          fix_transform.translation().x() = 0;
          fix_transform.translation().y() = 0;
          fix_transform.translation().z() = 0;
          fix_transform.rotate(euler_to_quat( 180*M_PI/180, 90*M_PI/180, 0 )) ;
          step = step*fix_transform;
        }
      }else{
        ROS_ERROR("LCM2ROS not flipping atlas foot orientations");
      }

      Isometry3dTime stepT = Isometry3dTime(i, step);
      stepsT.push_back(stepT);
      steps.push_back(step);

  }
  pc_vis_->pose_collection_to_lcm_from_list(9995, stepsT);

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transfer_time = 1.2;
  mout.swing_time = 1.2;
  // mout.trajectoryWaypointGenerationMethod = 0;
  for (int i=2; i < msg->footstep_plan.num_steps; i++){ // skip the first two standing steps
    drc::footstep_t s = msg->footstep_plan.footsteps[i];
    //mout.footstep_data_list.push_back( createFootStepList(s.is_right_foot , s.pos.translation.x, s.pos.translation.y, s.pos.translation.z,
    //                                                    s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z) );

    Eigen::Quaterniond r(steps[i].rotation());
    Eigen::Vector3d t(steps[i].translation());
    mout.footstep_data_list.push_back( createFootStepList(s.is_right_foot , t[0], t[1], t[2], r.w(),r.x(),r.y(),r.z() ));
  }
  walking_plan_pub_.publish(mout);
}

void LCM2ROS::footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::footstep_plan_t* msg) {
  ROS_ERROR("LCM2ROS got BDI_ADJUSTED_FOOTSTEP_PLAN (pronto and bdi mode)");
  // sendBasicSteps();

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transfer_time = 1.2;
  mout.swing_time = 1.2;
  // mout.trajectoryWaypointGenerationMethod = 0;
  for (int i=2; i < msg->num_steps; i++){ // skip the first two standing steps
    drc::footstep_t s = msg->footsteps[i];
    mout.footstep_data_list.push_back( createFootStepList(s.is_right_foot , s.pos.translation.x, s.pos.translation.y, s.pos.translation.z,
                                                        s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z) );
  }
  walking_plan_pub_.publish(mout);
}


void LCM2ROS::comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const valkyrie::com_height_packet_message_t* msg) {
  ROS_ERROR("LCM2ROS got com height");
  ihmc_msgs::ComHeightPacketMessage mout;
  mout.height_offset = msg->height_offset;
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
  mout.robot_side = msg->robot_side;
  mout.to_home_position = msg->to_home_position;
  mout.trajectory_time = msg->trajectory_time;
  mout.joint_angles = msg->joint_angles;
  hand_pose_pub_.publish(mout);
}

int main(int argc,char** argv) {
  std::string robotName;// = "valkyrie"; // "atlas"

  if (argc >= 2){
     ROS_ERROR("meah %s", argv[1] );
     robotName = argv[1];
  }else {
    ROS_ERROR("Need to have an argument: robot name");
    exit(-1);
  }

  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh, robotName);
  ROS_ERROR("LCM2ROS Translator Ready [robotName: %s]", robotName.c_str() );
  
  while(0 == lcm->handle());
  return 0;
}
