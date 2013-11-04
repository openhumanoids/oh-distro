#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sandia_hand_msgs/SimpleGrasp.h>
#include <sandia_hand_msgs/RelativeJointCommands.h>
#include <sandia_hand_msgs/CameraStreaming.h>
#include <handle_msgs/HandleControl.h>
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
    
    ros::Publisher simple_grasp_pub_right_ , simple_grasp_pub_left_ ;
    ros::Publisher camera_streaming_pub_right_, camera_streaming_pub_left_;
    void simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::simple_grasp_t* msg);  
  
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
    
  lcm_->subscribe("SIMPLE_GRASP_COMMAND",&LCM2ROS::simpleGraspCmdHandler,this);  
  simple_grasp_pub_left_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/l_hand/simple_grasp",10); 
  simple_grasp_pub_right_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp",10); 
//drc_robot.pmd:        exec = "rostopic pub /sandia_hands/r_hand/simple_grasp sandia_hand_msgs/SimpleGrasp  '{closed_amount: 100.0, name: cylindrical}'";

  camera_streaming_pub_left_ = nh_.advertise<sandia_hand_msgs::CameraStreaming>("/sandia_hands/l_hand/camera_streaming",10);
  camera_streaming_pub_right_ = nh_.advertise<sandia_hand_msgs::CameraStreaming>("/sandia_hands/r_hand/camera_streaming",10);

  rosnode = new ros::NodeHandle();
}

void LCM2ROS::handCameraTriggerHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::data_request_t* msg) {
  sandia_hand_msgs::CameraStreaming msgout;
  msgout.mode = 2;
  if (msg->type == drc::data_request_t::CAMERA_IMAGE_LHAND) {
    camera_streaming_pub_left_.publish(msgout);
  }
  else if (msg->type == drc::data_request_t::CAMERA_IMAGE_RHAND) {
    camera_streaming_pub_right_.publish(msgout);
  }
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
  


void LCM2ROS::simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::simple_grasp_t* msg) {
  ROS_ERROR("LCM2ROS Sending got simple");

  if(ros::ok()) {
    sandia_hand_msgs::SimpleGrasp msgout;
    msgout.name = "cylindrical";
    if (msg->left_state != drc::simple_grasp_t::UNCHANGED) {
      if (msg->left_state == drc::simple_grasp_t::CLOSED) {
        msgout.closed_amount = 100;
      }
      else msgout.closed_amount = 0;
      ROS_ERROR("LCM2ROS Sending simple grasp command (left)");
      simple_grasp_pub_left_.publish(msgout);
    }
    if (msg->right_state != drc::simple_grasp_t::UNCHANGED) {
      if (msg->right_state == drc::simple_grasp_t::CLOSED) {
        msgout.closed_amount = 100;
      }
      else msgout.closed_amount = 0;
      ROS_ERROR("LCM2ROS Sending simple grasp command (right)");
      simple_grasp_pub_right_.publish(msgout);    
    }
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
