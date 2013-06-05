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

#include <ConciseArgs>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    
    // DRCSIM 2.0 joint command API
    void jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher joint_cmd_pub_;

    // DRCSIM 2.6 atlas command API
    void atlasCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::atlas_command_t* msg);
    ros::Publisher atlas_cmd_pub_;
       
    ros::Publisher spindle_speed_pub_, head_fps_pub_, hand_fps_pub_, multisense_sl_fps_pub_;
    void sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg);
    
    void sandiaLHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    void sandiaRHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher sandia_l_hand_joint_cmd_pub_;
    ros::Publisher sandia_r_hand_joint_cmd_pub_;
    
    // Non-api translations:
    ros::Publisher body_twist_cmd_pub_;
    void actuatorCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg);
    void bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg);
    ros::Publisher gas_pedal_pub_, brake_pedal_pub_;
    void estopHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::nav_goal_timed_t* msg);   
    
    ros::Publisher simple_grasp_pub_right_ , simple_grasp_pub_left_ ;
    void simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::simple_grasp_t* msg);   
    
    
    ros::NodeHandle* rosnode;
    bool have_set_multisense_rate_; // have you set the initial multisesne rate
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  have_set_multisense_rate_= false;
	     
  /// DRCSIM 2.0 joint command API
  lcm_->subscribe("JOINT_COMMANDS",&LCM2ROS::jointCommandHandler,this);  
  joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/atlas/joint_commands",10, true);

  /// DRCSIM 2.6 atlas command API
  lcm_->subscribe("ATLAS_COMMAND",&LCM2ROS::atlasCommandHandler,this);  
  atlas_cmd_pub_ = nh_.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command",10, true);

  /// Spinning Laser control:
  lcm_->subscribe("SENSOR_REQUEST",&LCM2ROS::sensor_request_Callback,this);
  spindle_speed_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/set_spindle_speed",10);
  //head_fps_pub_ = nh_.advertise<std_msgs::Float64>("/mit/set_head_fps",10);
  hand_fps_pub_ = nh_.advertise<std_msgs::Float64>("/mit/set_hand_fps",10);
  
  multisense_sl_fps_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/fps",10);
  
  /// Sandia Hands joint command API
  lcm_->subscribe("L_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaLHandJointCommandHandler,this);  
  sandia_l_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/l_hand/joint_commands",10);
  lcm_->subscribe("R_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaRHandJointCommandHandler,this);  
  sandia_r_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",10); 

  lcm_->subscribe("SIMPLE_GRASP_COMMAND",&LCM2ROS::simpleGraspCmdHandler,this);  
  simple_grasp_pub_left_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/l_hand/simple_grasp",10); 
  simple_grasp_pub_right_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp",10); 
//drc_robot.pmd:        exec = "rostopic pub /sandia_hands/r_hand/simple_grasp sandia_hand_msgs/SimpleGrasp  '{closed_amount: 100.0, name: cylindrical}'";
  
  
  
  lcm_->subscribe("NAV_CMDS",&LCM2ROS::bodyTwistCmdHandler,this);
  body_twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  /// Experiments with car control:
  lcm_->subscribe("NAV_GOAL_ESTOP",&LCM2ROS::estopHandler,this);
  gas_pedal_pub_ = nh_.advertise<std_msgs::Float64>("drc_vehicle/gas_pedal/cmd", 1000);
  brake_pedal_pub_ = nh_.advertise<std_msgs::Float64>("drc_vehicle/brake_pedal/cmd", 1000);
  

  rosnode = new ros::NodeHandle();
}


void LCM2ROS::simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::simple_grasp_t* msg) {
  ROS_ERROR("LCM2ROS Sending got simpele");
  sandia_hand_msgs::SimpleGrasp msgout_l;
  msgout_l.name = "cylindrical";
  if (msg->close_left){
    msgout_l.closed_amount =100;
  }else{
    msgout_l.closed_amount =0;
  }

  sandia_hand_msgs::SimpleGrasp msgout_r;
  msgout_r.name = "cylindrical";
  if (msg->close_right){
    msgout_r.closed_amount =100;
  }else{
    msgout_r.closed_amount =0;
  }
  
  if(ros::ok()) {
    ROS_ERROR("LCM2ROS Sending simple grasp commands");
    simple_grasp_pub_left_.publish(msgout_l);
    simple_grasp_pub_right_.publish(msgout_r);    
  }   
}
  
void LCM2ROS::jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  
  joint_command_msg.ki_position.resize(msg->num_joints);
  joint_command_msg.kp_velocity.resize(msg->num_joints);
  joint_command_msg.i_effort_min.resize(msg->num_joints);
  joint_command_msg.i_effort_max.resize(msg->num_joints);

  for (int i=0; i<msg->num_joints; i++) {
    joint_command_msg.name.push_back("atlas::" + msg->name[i]); // must use scoped name
    joint_command_msg.position.push_back(msg->position[i]);
    joint_command_msg.velocity.push_back(msg->velocity[i]);
    joint_command_msg.effort.push_back(msg->effort[i]);

    joint_command_msg.kp_position.push_back(msg->kp_position[i]);
    joint_command_msg.kd_position.push_back(msg->kd_position[i]);

    // NOTE: This slows things down significantly, just set to zero instead
    // for now never change i gains or clamps
//    rosnode->getParam("atlas_controller/gains/" + msg->name[i] + "/p", joint_command_msg.kp_position[i]);
//    rosnode->getParam("atlas_controller/gains/" + msg->name[i] + "/d", joint_command_msg.kd_position[i]);
//    rosnode->getParam("atlas_controller/gains/" + msg->name[i] + "/i", joint_command_msg.ki_position[i]);
//    rosnode->getParam("atlas_controller/gains/" + msg->name[i] + "/i_clamp", joint_command_msg.i_effort_max[i]);
//    joint_command_msg.i_effort_min[i] = -joint_command_msg.i_effort_max[i];
  }
  if(ros::ok()) {
    joint_cmd_pub_.publish(joint_command_msg);
  } 
}  



void LCM2ROS::atlasCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::atlas_command_t* msg) {
  
  atlas_msgs::AtlasCommand atlas_command_msg;
  atlas_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  
  atlas_command_msg.ki_position.resize(msg->num_joints);
  atlas_command_msg.kp_velocity.resize(msg->num_joints);
  atlas_command_msg.i_effort_min.resize(msg->num_joints);
  atlas_command_msg.i_effort_max.resize(msg->num_joints);

  atlas_command_msg.desired_controller_period_ms = msg->desired_controller_period_ms;

  for (int i=0; i<msg->num_joints; i++) {
    //atlas_command_msg.name.push_back("atlas::" + msg->name[i]); // must use scoped name
    atlas_command_msg.position.push_back(msg->position[i]);
    atlas_command_msg.velocity.push_back(msg->velocity[i]);
    atlas_command_msg.effort.push_back(msg->effort[i]);
    atlas_command_msg.k_effort.push_back(msg->k_effort[i]);

    atlas_command_msg.kp_position.push_back(msg->kp_position[i]);
    atlas_command_msg.kd_position.push_back(msg->kd_position[i]);
  }
  if(ros::ok()) {
    atlas_cmd_pub_.publish(atlas_command_msg);
    
  } 
}  

void LCM2ROS::sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg){
  std::cout << "Got SENSOR_REQUEST setting sensor rates\n";
  if(ros::ok()){
    if (msg->spindle_rpm >=0){
      std_msgs::Float64 spindle_speed_msg;
      spindle_speed_msg.data = msg->spindle_rpm * (2*M_PI)/60; // convert from RPM to rad/sec
      spindle_speed_pub_.publish(spindle_speed_msg);    
      ROS_ERROR("LCM2ROS Setting Spindle RPM: %d", ((int) msg->spindle_rpm) );
      ROS_ERROR("                    rad/sec: %f", spindle_speed_msg.data );
    }else {
      ROS_ERROR("LCM2ROS Ignoring negative Spindle RPM: %d", ((int) msg->spindle_rpm) );
    }
    
    if (msg->head_fps >=0){
      std_msgs::Float64 head_fps_msg;
      head_fps_msg.data = msg->head_fps;
      //head_fps_pub_.publish(head_fps_msg); // disabled:
      multisense_sl_fps_pub_.publish(head_fps_msg);
      ROS_ERROR("LCM2ROS Setting Head Camera FPS: %d [from Gazebo]", ((int) msg->head_fps) );
    }else {
      ROS_ERROR("LCM2ROS Ignoring Negative Head FPS: %d", ((int) msg->head_fps) );
    }
    
    if (msg->hand_fps >=0){
      std_msgs::Float64 hand_fps_msg;
      hand_fps_msg.data = msg->hand_fps;
      hand_fps_pub_.publish(hand_fps_msg);    
      ROS_ERROR("LCM2ROS Setting Hand Camera FPS: %d [discard in Translator]", ((int) msg->hand_fps) );
    }else {
      ROS_ERROR("LCM2ROS Ignoring Negative Hand FPS: %d", ((int) msg->hand_fps) );
    }    
  }
  
  std::cout << "\n";
}


//void LCM2ROS::rot_scan_rate_cmd_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_timed_t* msg){
//  std_msgs::Float64 rot_scan_cmd_msg;
//  rot_scan_cmd_msg.data = msg->angular_velocity.x;
//  }
//}  

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
  
  int counter =0;

void LCM2ROS::bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg){
  geometry_msgs::Twist body_twist_cmd_msg;
  //body_twist_cmd_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  body_twist_cmd_msg.linear.x =  msg->linear_velocity.x;
  body_twist_cmd_msg.linear.y =  msg->linear_velocity.y;
  body_twist_cmd_msg.linear.z =  msg->linear_velocity.z;
  body_twist_cmd_msg.angular.x =  msg->angular_velocity.x;
  body_twist_cmd_msg.angular.y =  msg->angular_velocity.y;
  body_twist_cmd_msg.angular.z =  msg->angular_velocity.z;
  if(ros::ok()){
    body_twist_cmd_pub_.publish(body_twist_cmd_msg);
  }
}


// Transmit an estop to the car - here only for demonstration:
void LCM2ROS::estopHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::nav_goal_timed_t* msg){
  if(ros::ok()){
    std_msgs::Float64 msg;
    msg.data = 1.0;
    brake_pedal_pub_.publish(msg);
    std_msgs::Float64 gasmsg;
    gasmsg.data = 0.0;
    gas_pedal_pub_.publish(gasmsg);
  }
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
  
  
  ROS_ERROR("LCM2ROS Translator Sleeping");
  //handlerObject.setInitialMultisenseRate(5.0);
  sleep(4);
  ROS_ERROR("LCM2ROS Translator Ready");
  //handlerObject.setInitialMultisenseRate(5.0);  
  
  while(0 == lcm->handle());
  return 0;
}
