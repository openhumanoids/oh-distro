#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <osrf_msgs/JointCommands.h>

#include <ConciseArgs>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_,bool synchronized_, bool pause_physics_before_reconfig_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    
    // DRCSIM 2.0 joint command API
    void jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher joint_cmd_pub_;
       
    ros::Publisher spindle_speed_pub_, head_fps_pub_, hand_fps_pub_, multisense_sl_fps_pub_;
    void sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg);
    
    void sandiaLHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    void sandiaRHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher sandia_l_hand_joint_cmd_pub_;
    ros::Publisher sandia_r_hand_joint_cmd_pub_;
    
    // Variables to reconfigure the gazebo simulator in running:
    ros::Publisher pose_pub_;
    ros::Publisher joint_pub_;
    void reconfigCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::robot_state_t* msg);

    bool synchronized_;
    bool pause_physics_before_reconfig_;
    ros::ServiceClient pause_physics_;    
    ros::ServiceClient unpause_physics_;

    // Non-api translations:
    ros::Publisher body_twist_cmd_pub_;
    void actuatorCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg);
    void bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg);
    ros::Publisher gas_pedal_pub_, brake_pedal_pub_;
    void estopHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::nav_goal_timed_t* msg);   
    
    ros::NodeHandle* rosnode;
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, bool synchronized_, bool pause_physics_before_reconfig_): 
           lcm_(lcm_),nh_(nh_), synchronized_(synchronized_), pause_physics_before_reconfig_(pause_physics_before_reconfig_) {

  /// DRCSIM 2.0 joint command API
  lcm_->subscribe("JOINT_COMMANDS",&LCM2ROS::jointCommandHandler,this);  
  joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/atlas/joint_commands",10, true);

  /// Spinning Laser control:
  lcm_->subscribe("SENSOR_REQUEST",&LCM2ROS::sensor_request_Callback,this);
  spindle_speed_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/set_spindle_speed",10);
  head_fps_pub_ = nh_.advertise<std_msgs::Float64>("/mit/set_head_fps",10);
  hand_fps_pub_ = nh_.advertise<std_msgs::Float64>("/mit/set_hand_fps",10);
  
  multisense_sl_fps_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/fps",10);
  
  /// Sandia Hands joint command API
  lcm_->subscribe("L_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaLHandJointCommandHandler,this);  
  sandia_l_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/l_hand/joint_commands",10);
  lcm_->subscribe("R_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaRHandJointCommandHandler,this);  
  sandia_r_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",10); 
  
  /// For reconfiguring Gazebo while running:
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/atlas/set_pose",10);
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/atlas/configuration",10);
  lcm_->subscribe("SET_ROBOT_CONFIG",&LCM2ROS::reconfigCmdHandler,this);
  
  lcm_->subscribe("NAV_CMDS",&LCM2ROS::bodyTwistCmdHandler,this);
  body_twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  /// Experiments with car control:
  lcm_->subscribe("NAV_GOAL_ESTOP",&LCM2ROS::estopHandler,this);
  gas_pedal_pub_ = nh_.advertise<std_msgs::Float64>("drc_vehicle/gas_pedal/cmd", 1000);
  brake_pedal_pub_ = nh_.advertise<std_msgs::Float64>("drc_vehicle/brake_pedal/cmd", 1000);
  
  pause_physics_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  unpause_physics_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

  rosnode = new ros::NodeHandle();
}


void LCM2ROS::jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

  if(synchronized_ && ros::ok()) {
    std_srvs::Empty srv;
    if (!unpause_physics_.call(srv)) {
      ROS_ERROR("LCM2ROS::jointCommandHandler: Failed to unpause gazebo.");
    }  
  }

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
  
//  ros::spinOnce();
}  

void LCM2ROS::sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg){
  std::cout << "Got SENSOR_REQUEST setting sensor rates\n";
  if(ros::ok()){
    if (msg->spindle_rpm >=0){
      std_msgs::Float64 spindle_speed_msg;
      spindle_speed_msg.data = msg->spindle_rpm * (2*M_PI)/60; // convert from RPM to rad/sec
      spindle_speed_pub_.publish(spindle_speed_msg);    
      std::cout << "Setting Spindle rate Rev/min : "<< ((int) msg->spindle_rpm) <<" | ";
      std::cout << "Rad/sec : "<< spindle_speed_msg.data <<"\n";
    }else {
      std::cout << "Ignoring negative spindle rate: "<< ((int) msg->spindle_rpm) <<"\n";
    }
    
    if (msg->head_fps >=0){
      std_msgs::Float64 head_fps_msg;
      head_fps_msg.data = msg->head_fps;
      head_fps_pub_.publish(head_fps_msg);    
      std::cout << "   Setting Head Camera FPS: "<< ((int) msg->head_fps) <<"\n";
      
      //multisense_sl_fps_pub_.publish(head_fps_msg);
      //std::cout << "Also setting the rate this camera is generated by Gazebo\n";
    }else {
      std::cout << "Ignoring Negative Head FPS: "<< ((int) msg->head_fps) <<"\n";
    }
    
    if (msg->hand_fps >=0){
      std_msgs::Float64 hand_fps_msg;
      hand_fps_msg.data = msg->hand_fps;
      hand_fps_pub_.publish(hand_fps_msg);    
      std::cout << "   Setting Hand Camera FPS: "<< ((int) msg->hand_fps) <<"\n";
    }else {
      std::cout << "Ignoring Negative Hand RPS: "<< ((int) msg->hand_fps) <<"\n";
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

  if(synchronized_ && ros::ok()) {
    std_srvs::Empty srv;
    if (!unpause_physics_.call(srv)) {
      ROS_ERROR("LCM2ROS::sandia_l_hand_jointCommandHandler: Failed to unpause gazebo.");
    }  
  }

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

  if(synchronized_ && ros::ok()) {
    std_srvs::Empty srv;
    if (!unpause_physics_.call(srv)) {
      ROS_ERROR("LCM2ROS::sandia_r_hand_jointCommandHandler: Failed to unpause gazebo.");
    }  
  }

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
///////////////// Everything below this is not the in the gazebo API and should eventally be removed ///////////////////
void LCM2ROS::reconfigCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::robot_state_t* msg){
  cout << "Reconfiguration requested..."<< counter<<"\n";
  counter++;
  // pause gazebo beford resetting the state:
  if(pause_physics_before_reconfig_ && ros::ok()) {
    std_srvs::Empty srv;
    if (!pause_physics_.call(srv)) {
      ROS_ERROR("LCM2ROS::reconfigCmdHandler: Failed to pause gazebo.");
    }  
  }

  // set robot pose    
  geometry_msgs::Pose pose_msg;
//  pose_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  pose_msg.position = geometry_msgs::Point();
  pose_msg.position.x = msg->origin_position.translation.x;
  pose_msg.position.y = msg->origin_position.translation.y;
  pose_msg.position.z = msg->origin_position.translation.z;
  
  pose_msg.orientation = geometry_msgs::Quaternion();
  pose_msg.orientation.x = msg->origin_position.rotation.x;
  pose_msg.orientation.y = msg->origin_position.rotation.y;
  pose_msg.orientation.z = msg->origin_position.rotation.z;
  pose_msg.orientation.w = msg->origin_position.rotation.w;
  
  // set robot joint config
  sensor_msgs::JointState joint_msg;
  joint_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  for (int i=0; i<msg->num_joints; i++) {
    joint_msg.name.push_back(msg->joint_name[i]);
    joint_msg.position.push_back(msg->joint_position[i]);
  }
  if(ros::ok()) {
    joint_pub_.publish(joint_msg);      
    pose_pub_.publish(pose_msg);      
  }      
  
  // send zero joint command
  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  joint_command_msg.name.resize(msg->num_joints);
  joint_command_msg.position.resize(msg->num_joints);
  joint_command_msg.velocity.resize(msg->num_joints);
  joint_command_msg.effort.resize(msg->num_joints);
  joint_command_msg.kp_position.resize(msg->num_joints);
  joint_command_msg.ki_position.resize(msg->num_joints);
  joint_command_msg.kd_position.resize(msg->num_joints);
  joint_command_msg.kp_velocity.resize(msg->num_joints);
  joint_command_msg.i_effort_min.resize(msg->num_joints);
  joint_command_msg.i_effort_max.resize(msg->num_joints);

  joint_command_msg.name.resize(msg->num_joints);
  joint_command_msg.position.resize(msg->num_joints);
  joint_command_msg.velocity.resize(msg->num_joints);
  joint_command_msg.effort.resize(msg->num_joints);
  
  
  map<string, int> correct_order;
  correct_order["back_lbz"]= 0 ;
  correct_order["back_mby"]= 1 ;
  correct_order["back_ubx"]= 2 ;
  correct_order["neck_ay"]= 3 ;
  correct_order["l_leg_uhz"]= 4 ;
  correct_order["l_leg_mhx"]= 5 ;
  correct_order["l_leg_lhy"]= 6 ;
  correct_order["l_leg_kny"]= 7 ;
  correct_order["l_leg_uay"]= 8 ;
  correct_order["l_leg_lax"]= 9 ;
  correct_order["r_leg_uhz"]= 10 ;
  correct_order["r_leg_mhx"]= 11 ;
  correct_order["r_leg_lhy"]= 12 ;
  correct_order["r_leg_kny"]= 13 ;
  correct_order["r_leg_uay"]= 14 ;
  correct_order["r_leg_lax"]= 15 ;
  correct_order["l_arm_usy"]= 16 ;
  correct_order["l_arm_shx"]= 17 ;
  correct_order["l_arm_ely"]= 18 ;
  correct_order["l_arm_elx"]= 19 ;
  correct_order["l_arm_uwy"]= 20 ;
  correct_order["l_arm_mwx"]= 21 ;
  correct_order["r_arm_usy"]= 22 ;
  correct_order["r_arm_shx"]= 23 ;
  correct_order["r_arm_ely"]= 24 ;
  correct_order["r_arm_elx"]= 25 ;
  correct_order["r_arm_uwy"]= 26 ;
  correct_order["r_arm_mwx"]= 27 ;
   
  
  for (int i=0; i<msg->num_joints; i++) {
    int j = correct_order.find( msg->joint_name[i] )->second;
    
    joint_command_msg.name[j] = msg->joint_name[i];
    joint_command_msg.position[j] = msg->joint_position[i];
    joint_command_msg.velocity[j] = 0;
    joint_command_msg.effort[j] = 0;
    
//    joint_command_msg.kp_position[j] =0.0;
//    joint_command_msg.kd_position[j] =0.0;
//    joint_command_msg.ki_position[j] =0.0;
//    joint_command_msg.i_effort_max[j]=0.0;

    rosnode->getParam("atlas_controller/gains/" + msg->joint_name[i] + "/p", joint_command_msg.kp_position[j]);
    rosnode->getParam("atlas_controller/gains/" + msg->joint_name[i] + "/d", joint_command_msg.kd_position[j]);
    rosnode->getParam("atlas_controller/gains/" + msg->joint_name[i] + "/i", joint_command_msg.ki_position[j]);
    rosnode->getParam("atlas_controller/gains/" + msg->joint_name[i] + "/i_clamp", joint_command_msg.i_effort_max[j]);
    
    joint_command_msg.i_effort_min[j] = -joint_command_msg.i_effort_max[j];
    
  }
  if(ros::ok()) {
    joint_cmd_pub_.publish(joint_command_msg);
  } 
  
  ros::spinOnce();
  cout << "... Reconfiguration complete\n";
}

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
  ConciseArgs parser(argc, argv, "lcm2ros");
  bool synced = false;
  bool pause_physics_before_reconfig = true; // do we need physics to be paused to reconfig the robot
  parser.add(synced, "s", "synced", "Synchronized joint commands.");
  parser.add(pause_physics_before_reconfig, "p", "pause_physics_before_reconfig", "Pause physics before setting robot config.");
  parser.parse();
  cout << "Synchronized: " << synced << "\n"; 
  cout << "Pause physics before setting robot config: " << pause_physics_before_reconfig << "\n";   
  
  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh, synced, pause_physics_before_reconfig);
  cout << "\nlcm2ros translator ready\n";
  while(0 == lcm->handle());
  return 0;
}
