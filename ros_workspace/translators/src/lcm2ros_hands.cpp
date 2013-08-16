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

    void sandiaLHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    void sandiaRHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher sandia_l_hand_joint_cmd_pub_;
    ros::Publisher sandia_r_hand_joint_cmd_pub_;
    
    ros::Publisher simple_grasp_pub_right_ , simple_grasp_pub_left_ ;
    void simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::simple_grasp_t* msg);  
    
    ros::NodeHandle* rosnode;
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
 

  
  /// Sandia Hands joint command API
  lcm_->subscribe("L_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaLHandJointCommandHandler,this);  
  sandia_l_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/l_hand/joint_commands",10);
  lcm_->subscribe("R_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaRHandJointCommandHandler,this);  
  sandia_r_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",10); 

  lcm_->subscribe("SIMPLE_GRASP_COMMAND",&LCM2ROS::simpleGraspCmdHandler,this);  
  simple_grasp_pub_left_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/l_hand/simple_grasp",10); 
  simple_grasp_pub_right_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp",10); 
//drc_robot.pmd:        exec = "rostopic pub /sandia_hands/r_hand/simple_grasp sandia_hand_msgs/SimpleGrasp  '{closed_amount: 100.0, name: cylindrical}'";


  rosnode = new ros::NodeHandle();
}


void LCM2ROS::simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::simple_grasp_t* msg) {
  ROS_ERROR("LCM2ROS Sending got simpele");

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
void LCM2ROS::sandiaLHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);

  for (int i=0; i<msg->num_joints; i++) {
    joint_command_msg.name.push_back("sandia_hands::l_hand::" + msg->name[i]); // must use scoped name
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
    sandia_l_hand_joint_cmd_pub_.publish(joint_command_msg);
  } 
} 

void LCM2ROS::sandiaRHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);

  for (int i=0; i<msg->num_joints; i++) {
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
    sandia_r_hand_joint_cmd_pub_.publish(joint_command_msg);
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
