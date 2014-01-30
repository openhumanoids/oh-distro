#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/data_request_t.hpp"
#include "lcmtypes/drc/joint_command_t.hpp"
#include "lcmtypes/drc/joint_command_relative_t.hpp"
#include "lcmtypes/drc/sandia_simple_grasp_t.hpp"
#include <lcmtypes/irobothand.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sandia_hand_msgs/SimpleGrasp.h>
#include <sandia_hand_msgs/RelativeJointCommands.h>
// #include <sandia_hand_msgs/CameraStreaming.h>
#include <handle_msgs/HandleControl.h>

#include <mit_helios_scripts/MITIRobotHandSpread.h>
#include <mit_helios_scripts/MITIRobotHandCalibrate.h>
#include <mit_helios_scripts/MITIRobotHandCurrentControlClose.h>
#include <mit_helios_scripts/MITIRobotHandPositionControlClose.h>


#include <map>
#include <ConciseArgs>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
	  lcm::LCM lcm_publish_ ;

    void LHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    void RHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher sandia_l_hand_joint_cmd_pub_,sandia_r_hand_joint_cmd_pub_;
    ros::Publisher irobot_l_hand_joint_cmd_pub_,irobot_r_hand_joint_cmd_pub_;
    void publishSandiaHandCommand(const drc::joint_command_t* msg, bool is_left);
    void publishIrobotHandCommand(const drc::joint_command_t* msg, bool is_left); 
    
    ros::Publisher sandia_l_hand_relative_joint_cmd_pub_,sandia_r_hand_relative_joint_cmd_pub_;
    void RelativeJointCommandsHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_relative_t* msg);

    void handCameraTriggerHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::data_request_t* msg);
    
    // ros::Publisher camera_streaming_pub_right_, camera_streaming_pub_left_;
    
    // Ignore "sandia" - this message is used by both hands
    void simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sandia_simple_grasp_t* msg);  
    ros::Publisher irobot_l_hand_simple_cmd_pub_, irobot_r_hand_simple_cmd_pub_;
    
    
    ///
    void IrobotSpreadHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const irobothand::spread_t* msg);  
    ros::Publisher irobot_left_spread_pub_, irobot_right_spread_pub_;
    ///
    void IrobotCalibrateHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const irobothand::calibrate_t* msg);  
    ros::Publisher irobot_left_calibrate_pub_, irobot_right_calibrate_pub_;
    //
    void IrobotCurrentControlCloseHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const irobothand::current_control_close_t* msg);  
    ros::Publisher irobot_left_current_control_close_pub_, irobot_right_current_control_close_pub_;    
    //
    void IrobotPositionControlCloseHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const irobothand::position_control_close_t* msg);  
    ros::Publisher irobot_left_position_control_close_pub_, irobot_right_position_control_close_pub_;    
    
    ros::NodeHandle* rosnode;
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
 
  /// Sandia Hands joint command API
  lcm_->subscribe("L_HAND_JOINT_COMMANDS",&LCM2ROS::LHandJointCommandHandler,this); 
  lcm_->subscribe("R_HAND_JOINT_COMMANDS",&LCM2ROS::RHandJointCommandHandler,this);  
  
  //advertise sandia hand commands and irobot hand commands
  sandia_l_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/l_hand/joint_commands",10);
  sandia_r_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",10); 
  irobot_l_hand_joint_cmd_pub_ = nh_.advertise<handle_msgs::HandleControl>("/irobot_hands/l_hand/control",10);
  irobot_r_hand_joint_cmd_pub_ = nh_.advertise<handle_msgs::HandleControl>("/irobot_hands/r_hand/control",10); 

  lcm_->subscribe("L_HAND_JOINT_COMMANDS_RELATIVE",&LCM2ROS::RelativeJointCommandsHandler,this); 
  lcm_->subscribe("TRIGGER_CAMERA",&LCM2ROS::handCameraTriggerHandler,this);
  lcm_->subscribe("R_HAND_JOINT_COMMANDS_RELATIVE",&LCM2ROS::RelativeJointCommandsHandler,this); 
  sandia_l_hand_relative_joint_cmd_pub_ = nh_.advertise<sandia_hand_msgs::RelativeJointCommands>("/sandia_hands/l_hand/relative_joint_commands",10);
  sandia_r_hand_relative_joint_cmd_pub_ = nh_.advertise<sandia_hand_msgs::RelativeJointCommands>("/sandia_hands/r_hand/relative_joint_commands",10); 
   
  lcm_->subscribe("IROBOT_LEFT_SIMPLE_GRASP",&LCM2ROS::simpleGraspCmdHandler,this);
  lcm_->subscribe("IROBOT_RIGHT_SIMPLE_GRASP",&LCM2ROS::simpleGraspCmdHandler,this);
  irobot_l_hand_simple_cmd_pub_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/irobot_hands/l_hand/simple_command",10);
  irobot_r_hand_simple_cmd_pub_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/irobot_hands/r_hand/simple_command",10); 

  // camera_streaming_pub_left_ = nh_.advertise<sandia_hand_msgs::CameraStreaming>("/sandia_hands/l_hand/camera_streaming",10);
  // camera_streaming_pub_right_ = nh_.advertise<sandia_hand_msgs::CameraStreaming>("/sandia_hands/r_hand/camera_streaming",10);
  
  
  /// iRobot Messages - keep this clean!
  lcm_->subscribe("IROBOT_LEFT_SPREAD",&LCM2ROS::IrobotSpreadHandler,this);  
  lcm_->subscribe("IROBOT_RIGHT_SPREAD",&LCM2ROS::IrobotSpreadHandler,this);  
  irobot_left_spread_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandSpread>("/irobot_hands/l_hand/mit_spread",10);
  irobot_right_spread_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandSpread>("/irobot_hands/r_hand/mit_spread",10);

  lcm_->subscribe("IROBOT_LEFT_CALIBRATE",&LCM2ROS::IrobotCalibrateHandler,this);  
  lcm_->subscribe("IROBOT_RIGHT_CALIBRATE",&LCM2ROS::IrobotCalibrateHandler,this);  
  irobot_left_calibrate_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandCalibrate>("/irobot_hands/l_hand/mit_calibrate",10);
  irobot_right_calibrate_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandCalibrate>("/irobot_hands/r_hand/mit_calibrate",10);
  
  lcm_->subscribe("IROBOT_LEFT_CURRENT_CONTROL_CLOSE",&LCM2ROS::IrobotCurrentControlCloseHandler,this);  
  lcm_->subscribe("IROBOT_RIGHT_CURRENT_CONTROL_CLOSE",&LCM2ROS::IrobotCurrentControlCloseHandler,this);  
  irobot_left_current_control_close_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandCurrentControlClose>("/irobot_hands/l_hand/mit_current_control_close",10);
  irobot_right_current_control_close_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandCurrentControlClose>("/irobot_hands/r_hand/mit_current_control_close",10);
  
  lcm_->subscribe("IROBOT_LEFT_POSITION_CONTROL_CLOSE",&LCM2ROS::IrobotPositionControlCloseHandler,this);  
  lcm_->subscribe("IROBOT_RIGHT_POSITION_CONTROL_CLOSE",&LCM2ROS::IrobotPositionControlCloseHandler,this);  
  irobot_left_position_control_close_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandPositionControlClose>("/irobot_hands/l_hand/mit_position_control_close",10);
  irobot_right_position_control_close_pub_ = nh_.advertise<mit_helios_scripts::MITIRobotHandPositionControlClose>("/irobot_hands/r_hand/mit_position_control_close",10);
  

  rosnode = new ros::NodeHandle();
}

void LCM2ROS::handCameraTriggerHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::data_request_t* msg) {
  // sandia_hand_msgs::CameraStreaming msgout;
  // msgout.mode = 2;
  // if (msg->type == drc::data_request_t::CAMERA_IMAGE_LHAND) {
  //   camera_streaming_pub_left_.publish(msgout);
  // }
  // else if (msg->type == drc::data_request_t::CAMERA_IMAGE_RHAND) {
  //   camera_streaming_pub_right_.publish(msgout);
  // }
}

void LCM2ROS::RelativeJointCommandsHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_relative_t* msg) {

  sandia_hand_msgs::RelativeJointCommands msgout;
  
  std::vector<uint8_t> max_effort(msg->max_effort.begin(), msg->max_effort.end());
  
  for (int i=0;i<12; i++){
    msgout.max_effort[i] = msg->max_effort[i];
    msgout.position[i] = msg->position[i];
  }

  if (channel == "L_HAND_JOINT_COMMANDS_RELATIVE"){
    ROS_ERROR("LCM2ROS Sending got relative: %s", channel.c_str());
    sandia_l_hand_relative_joint_cmd_pub_.publish(msgout);       
  }else if (channel == "R_HAND_JOINT_COMMANDS_RELATIVE"){
    ROS_ERROR("LCM2ROS Sending got relative: %s", channel.c_str());
    sandia_r_hand_relative_joint_cmd_pub_.publish(msgout);       
  }  
  
}
  

void LCM2ROS::simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sandia_simple_grasp_t* msg){
  //std::cout << "got: " << channel << "\n"; 
  ROS_ERROR("got %s", channel.c_str());
  
  sandia_hand_msgs::SimpleGrasp msgout;
  msgout.name = msg->name;
  msgout.closed_amount = msg->closed_amount;

  ROS_ERROR("LCM2ROS Sending simple command message: %s", channel.c_str());
  if (channel == "IROBOT_LEFT_SIMPLE_GRASP"){
    irobot_l_hand_simple_cmd_pub_.publish(msgout);     
  }else if (channel == "IROBOT_RIGHT_SIMPLE_GRASP"){
    irobot_r_hand_simple_cmd_pub_.publish(msgout);
  } 
  
}
  
  
// Sandia Hands joint command handlers
void LCM2ROS::LHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

 

  // use msg->robot_name to determine whether to use sandia_hand_publisher or irobot_hand_publisher
  if(msg->robot_name=="sandia"){
    bool is_left=true;
    publishSandiaHandCommand(msg,is_left);
  }
  else if (msg->robot_name=="irobot"){
     bool is_left=true;
    publishIrobotHandCommand(msg,is_left); 
  }
  else{
    ROS_ERROR("robot_name field in drc::joint_command_t for grasp commands should be sandia or irobot");
  }
} 

void LCM2ROS::RHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {


  // use msg->robot_name to determine whether to use sandia_hand_publisher or irobot_hand_publisher
  if(msg->robot_name=="sandia"){
    bool is_left=false;
    publishSandiaHandCommand(msg,is_left);    
  }
  else if (msg->robot_name=="irobot"){
    bool is_left=false;
    publishIrobotHandCommand(msg,is_left);  
  }
  else{
    ROS_ERROR("robot_name field in drc::joint_command_t for grasp commands should be sandia or irobot");
  }

}
  
  
void LCM2ROS::publishSandiaHandCommand(const drc::joint_command_t* msg, bool is_left)
{
  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  for (int i=0; i<msg->num_joints; i++) {
    if(is_left)
      joint_command_msg.name.push_back("sandia_hands::l_hand::" + msg->name[i]); // must use scoped name
    else 
      joint_command_msg.name.push_back("sandia_hands::r_hand::" + msg->name[i]); // must use scoped name
    joint_command_msg.position.push_back(msg->position[i]);
    joint_command_msg.velocity.push_back(msg->velocity[i]);
    joint_command_msg.effort.push_back(msg->effort[i]);
    joint_command_msg.kp_position.push_back(msg->kp_position[i]);
    joint_command_msg.kd_position.push_back(msg->kd_position[i]);
    joint_command_msg.ki_position.push_back(msg->ki_position[i]);
    joint_command_msg.kp_velocity.resize(msg->kp_velocity[i]);
    joint_command_msg.i_effort_min.push_back(msg->i_effort_min[i]);
    joint_command_msg.i_effort_max.push_back(msg->i_effort_max[i]);
  }
  if(ros::ok()) {
    if(is_left)
      sandia_l_hand_joint_cmd_pub_.publish(joint_command_msg);
    else
      sandia_r_hand_joint_cmd_pub_.publish(joint_command_msg);
  } 
  
}  

void LCM2ROS::publishIrobotHandCommand(const drc::joint_command_t* msg, bool is_left)
{

    handle_msgs::HandleControl control_msg;
    for (size_t i=0;i<5;i++){
     control_msg.type[i] = control_msg.POSITION;
     control_msg.value[i] = 0;
     control_msg.valid[i]=false;
    }
     
    
    std::vector<std::string>::const_iterator found;
    std::string joint_name;
    if(is_left)
      joint_name = "left_finger[0]/joint_base";
    else
      joint_name = "right_finger[0]/joint_base";
    found = std::find (msg->name.begin(), msg->name.end(),joint_name);
    if (found != msg->name.end()) { 
     unsigned int index = found - msg->name.begin();
     double radians_to_ticks = (3500/(0.5*M_PI));  
     control_msg.value[0] = radians_to_ticks*msg->position[index];
     control_msg.valid[0] = true;
    }
      
   if(is_left)
      joint_name = "left_finger[1]/joint_base";
    else
      joint_name = "right_finger[1]/joint_base";
    found = std::find (msg->name.begin(), msg->name.end(),joint_name);
    if (found != msg->name.end()) { 
     unsigned int index = found - msg->name.begin();
     double radians_to_ticks = (3500/(0.5*M_PI));  
     control_msg.value[1] = radians_to_ticks*msg->position[index];
     control_msg.valid[1] = true;
    }     
        
    if(is_left)
      joint_name = "left_finger[2]/joint_base";
    else
      joint_name = "right_finger[2]/joint_base";
    found = std::find (msg->name.begin(), msg->name.end(),joint_name);
    if (found != msg->name.end()) { 
     unsigned int index = found - msg->name.begin();
     double radians_to_ticks = (3500/(0.5*M_PI));  
     control_msg.value[2] = radians_to_ticks*msg->position[index];
     control_msg.valid[2] = true;
    }     
     
    // IGNORE ANTAGONISTIC TENDON
    
    if(is_left)
      joint_name = "left_finger[0]/joint_base_rotation";
    else
      joint_name = "right_finger[0]/joint_base_rotation";
    found = std::find (msg->name.begin(), msg->name.end(),joint_name);
    if (found != msg->name.end()) { 
     unsigned int index = found - msg->name.begin();
     double radians_to_ticks = (768/(0.5*M_PI)); 
     control_msg.value[4] = radians_to_ticks*msg->position[index];
     control_msg.valid[4] = true;
    }  
    
    if(ros::ok()) {
    if(is_left)
      irobot_l_hand_joint_cmd_pub_.publish(control_msg);
    else
      irobot_r_hand_joint_cmd_pub_.publish(control_msg);
  } 
 
}

/// iRobot-only Messages
void LCM2ROS::IrobotSpreadHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const irobothand::spread_t* msg) {
  mit_helios_scripts::MITIRobotHandSpread msgout;
  msgout.angle_radians= msg->angle_radians;
  if (channel == "IROBOT_LEFT_SPREAD") {
    irobot_left_spread_pub_.publish(msgout);
  }else{
    irobot_right_spread_pub_.publish(msgout);  
  }
}

void LCM2ROS::IrobotCalibrateHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const irobothand::calibrate_t* msg) {
  mit_helios_scripts::MITIRobotHandCalibrate msgout;
  msgout.in_jig= msg->in_jig;
  if (channel == "IROBOT_LEFT_CALIBRATE") {
    irobot_left_calibrate_pub_.publish(msgout);
  }else{
    irobot_right_calibrate_pub_.publish(msgout);  
  }
}

void LCM2ROS::IrobotCurrentControlCloseHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const irobothand::current_control_close_t* msg){
  mit_helios_scripts::MITIRobotHandCurrentControlClose msgout;
  msgout.current_milliamps= msg->current_milliamps;
  for (int i=0; i < 4; i++)
    msgout.valid[i]= msg->valid[i]; 
  
  if (channel == "IROBOT_LEFT_CURRENT_CONTROL_CLOSE") {
    irobot_left_current_control_close_pub_.publish(msgout);
  }else{
    irobot_right_current_control_close_pub_.publish(msgout);  
  }  
}

void LCM2ROS::IrobotPositionControlCloseHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const irobothand::position_control_close_t* msg){
  mit_helios_scripts::MITIRobotHandPositionControlClose msgout;
  msgout.close_fraction= msg->close_fraction;
  for (int i=0; i < 4; i++)
    msgout.valid[i]= msg->valid[i]; 
  
  if (channel == "IROBOT_LEFT_POSITION_CONTROL_CLOSE") {
    irobot_left_position_control_close_pub_.publish(msgout);
  }else{
    irobot_right_position_control_close_pub_.publish(msgout);  
  }  
}



int main(int argc,char** argv) {

  ros::init(argc,argv,"lcm2ros_hands",ros::init_options::NoSigintHandler);

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh);
  cout << "\nlcm2ros_hands translator ready\n";
  while(0 == lcm->handle());
  return 0;
}
