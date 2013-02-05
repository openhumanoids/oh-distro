#include <cstdlib>
#include <map>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
// MIT Homebrew messages
#include <atlas_gazebo_msgs/RobotState.h>
#include <atlas_gazebo_msgs/ActuatorCmd.h>

#include <ConciseArgs>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_,
      bool reconfig_, bool pause_physics_then_reconfig_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    std::map<std::string, ros::Publisher> pub_map_;
    
    
    void positionCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::joint_angles_t* msg);
    ros::Publisher rot_scan_cmd_pub_;
    void rot_scan_rate_cmd_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_timed_t* msg);
    
    // Variables to reconfigure the gazebo simulator in running:
    ros::Publisher pose_pub_;
    ros::Publisher joint_pub_;
    bool reconfig_;
    ros::ServiceClient pause_physics_;    
    bool pause_physics_then_reconfig_;
    void reconfigCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::robot_state_t* msg);

    // Non-api translations:
    ros::Publisher actuator_cmd_pub_, body_twist_cmd_pub_;
    void actuatorCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg);
    void bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg);
    ros::Publisher gas_pedal_pub_, brake_pedal_pub_;
    void estopHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::nav_goal_timed_t* msg);   
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, 
           bool reconfig_, bool pause_physics_then_reconfig_): 
           lcm_(lcm_),nh_(nh_), 
           reconfig_(reconfig_), pause_physics_then_reconfig_(pause_physics_then_reconfig_) {
             
             
  // Typical Position Control: /////////////////////////////////////////
  vector<string> joints = {"l_arm_elx", "l_arm_ely", "l_arm_mwx", "l_arm_shx", "l_arm_usy", "l_arm_uwy", 
                           "r_arm_elx", "r_arm_ely", "r_arm_mwx", "r_arm_shx", "r_arm_usy", "r_arm_uwy", 
                           "l_leg_kny", "l_leg_lax", "l_leg_lhy", "l_leg_mhx", "l_leg_uay", "l_leg_uhz", 
                           "r_leg_kny", "r_leg_lax", "r_leg_lhy", "r_leg_mhx", "r_leg_uay", "r_leg_uhz", 
                           "neck_ay", "back_lbz", "back_mby", "back_ubx"};
  // sandiaPositionCommandTranslator used to send right_* ... switched to r_*
  vector<string> finger_joints = {"r_f0_j0", "r_f0_j1", "r_f0_j2", 
                           "r_f1_j0", "r_f1_j1", "r_f1_j2", 
                           "r_f2_j0", "r_f2_j1", "r_f2_j2", 
                           "r_f3_j0", "r_f3_j1", "r_f3_j2", 
                           "l_f0_j0", "l_f0_j1", "l_f0_j2", 
                           "l_f1_j0", "l_f1_j1", "l_f1_j2", 
                           "l_f2_j0", "l_f2_j1", "l_f2_j2", 
                           "l_f3_j0", "l_f3_j1", "l_f3_j2",
                           "real_base_x", "real_base_y", "real_base_z",
                           "real_base_roll", "real_base_pitch", "real_base_yaw"};
  joints.insert( joints.end(), finger_joints.begin(), finger_joints.end() );
  cout << joints.size() << " joints available for command\n";
            
  for (size_t i=0; i < joints.size() ; i++){
      pub_map_.insert(std::make_pair( joints[i] ,nh_.advertise<std_msgs::Float64>( string( joints[i] + "_position_controller/command") ,10)));
  }
  lcm_->subscribe("JOINT_POSITION_CMDS",&LCM2ROS::positionCmdHandler,this);  

  /// Spinning Laser control:
  lcm_->subscribe("ROTATING_SCAN_RATE_CMD",&LCM2ROS::rot_scan_rate_cmd_Callback,this);
  rot_scan_cmd_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/set_spindle_speed",10);
  
  /// Reconfiguring Gazebo while running:
  if (reconfig_){
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/atlas/set_pose",10);
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/atlas/configuration",10);
    pause_physics_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    lcm_->subscribe("SET_ROBOT_CONFIG",&LCM2ROS::reconfigCmdHandler,this);
  }
  
  
  /// Older style control of robot via controller_manager:
  lcm_->subscribe("ACTUATOR_CMDS",&LCM2ROS::actuatorCmdHandler,this);
  lcm_->subscribe("NAV_CMDS",&LCM2ROS::bodyTwistCmdHandler,this);
  actuator_cmd_pub_ = nh_.advertise<atlas_gazebo_msgs::ActuatorCmd>("actuator_cmd",10);
  body_twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  /// Experiments with car control:
  lcm_->subscribe("NAV_GOAL_ESTOP",&LCM2ROS::estopHandler,this);
  gas_pedal_pub_ = nh_.advertise<std_msgs::Float64>("mit_golf_cart/gas_pedal/cmd", 1000);
  brake_pedal_pub_ = nh_.advertise<std_msgs::Float64>("mit_golf_cart/brake_pedal/cmd", 1000);
}


void LCM2ROS::positionCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::joint_angles_t* msg){
  std_msgs::Float64 position_command_msg;
  for (int i=0; i<msg->num_joints; i++) {
      // should we instead publish all joints at the same time?
      position_command_msg.data = msg->joint_position[i];
    if(ros::ok()) {
      pub_map_[msg->joint_name[i]].publish(position_command_msg);
    }
  }
}  

void LCM2ROS::rot_scan_rate_cmd_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_timed_t* msg){
  std_msgs::Float64 rot_scan_cmd_msg;
  rot_scan_cmd_msg.data = msg->angular_velocity.x;
  if(ros::ok()){
    rot_scan_cmd_pub_.publish(rot_scan_cmd_msg);
  }
}  
  
///////////////// Everything below this is not the in the gazebo API and should eventally be removed ///////////////////
void LCM2ROS::reconfigCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::robot_state_t* msg){
  cout << "Reconfiguration requested ...\n";
  // pause gazebo beford resetting the state:
  if(pause_physics_then_reconfig_ && ros::ok()) {
    std_srvs::Empty srv;
    if (!pause_physics_.call(srv)) {
      ROS_ERROR("Failed to pause gazebo.");
    }  
  }

  // set robot pose    
  geometry_msgs::Pose pose_msg;
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
  for (int i=0; i<msg->num_joints; i++) {
    joint_msg.name.push_back(msg->joint_name[i]);
    joint_msg.position.push_back(msg->joint_position[i]);
  }
  if(ros::ok()) {
    joint_pub_.publish(joint_msg);      
    pose_pub_.publish(pose_msg);      
  }      
  
  // set PID goals
  std_msgs::Float64 position_command_msg;
  for (int i=0; i<msg->num_joints; i++) {
    position_command_msg.data = msg->joint_position[i];
    if(ros::ok()) {
      pub_map_[msg->joint_name[i]].publish(position_command_msg);
    }
  }
  ros::spinOnce();
  cout << "... Reconfiguration complete\n";
}



void LCM2ROS::actuatorCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg){
  atlas_gazebo_msgs::ActuatorCmd actuator_cmd_msg;
  actuator_cmd_msg.header.stamp.fromNSec( msg->utime*1000 ); // from usec to nsec
  actuator_cmd_msg.robot_name = msg->robot_name;
  for(std::vector<int>::size_type i=0;i!=msg->actuator_name.size();i++){//So varaibles in lcm message are all in vector type
    actuator_cmd_msg.actuator_name.push_back(msg->actuator_name.at(i));
    actuator_cmd_msg.actuator_effort.push_back(msg->actuator_effort.at(i));
    actuator_cmd_msg.effort_duration.push_back(msg->effort_duration.at(i));
  }
  if(ros::ok()){
    actuator_cmd_pub_.publish(actuator_cmd_msg);
  }
}


void LCM2ROS::bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg){
  geometry_msgs::Twist body_twist_cmd_msg;
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
  bool reconfig =false;
  bool pause_physics_then_reconfig = false;// do we need physics to be paused to reconfig the robot
  parser.add(reconfig, "r", "reconfig", "Listen for gazebo reconfig messages");
  parser.add(pause_physics_then_reconfig, "p", "pause_physics_then_reconfig", "Pause physics then reconfig");
  parser.parse();
  cout << "reconfig: " << reconfig << "\n"; 
  cout << "pause_physics_then_reconfig: " << pause_physics_then_reconfig << "\n";   
  
  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh, reconfig, pause_physics_then_reconfig);
  cout << "\nlcm2ros translator ready\n";
  while(0 == lcm->handle());
  return 0;
}