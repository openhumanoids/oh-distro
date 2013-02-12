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
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_,bool reconfig_, bool pause_physics_then_reconfig_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    
    // DRCSIM 2.0 joint command API
    void jointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher joint_cmd_pub_;
        
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
    ros::Publisher body_twist_cmd_pub_;
    void actuatorCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg);
    void bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg);
    ros::Publisher gas_pedal_pub_, brake_pedal_pub_;
    void estopHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::nav_goal_timed_t* msg);   
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, bool reconfig_, bool pause_physics_then_reconfig_): 
           lcm_(lcm_),nh_(nh_), reconfig_(reconfig_), pause_physics_then_reconfig_(pause_physics_then_reconfig_) {

  // DRCSIM 2.0 joint command API
  lcm_->subscribe("JOINT_CMDS",&LCM2ROS::jointCommandHandler,this);  
  joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/atlas/joint_commands",10);

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
  
  lcm_->subscribe("NAV_CMDS",&LCM2ROS::bodyTwistCmdHandler,this);
  body_twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  /// Experiments with car control:
  lcm_->subscribe("NAV_GOAL_ESTOP",&LCM2ROS::estopHandler,this);
  gas_pedal_pub_ = nh_.advertise<std_msgs::Float64>("mit_golf_cart/gas_pedal/cmd", 1000);
  brake_pedal_pub_ = nh_.advertise<std_msgs::Float64>("mit_golf_cart/brake_pedal/cmd", 1000);
}


void LCM2ROS::jointCommandHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::joint_command_t* msg){
  osrf_msgs::JointCommands joint_command_msg;
  for (int i=0; i<msg->num_joints; i++) {
    joint_command_msg.name.push_back(msg->name[i]);
    joint_command_msg.position.push_back(msg->position[i]);
    joint_command_msg.velocity.push_back(msg->velocity[i]);
    joint_command_msg.effort.push_back(msg->effort[i]);

    joint_command_msg.kp_position.push_back(msg->kp_position[i]);
    joint_command_msg.ki_position.push_back(msg->ki_position[i]);
    joint_command_msg.kd_position.push_back(msg->kd_position[i]);
    joint_command_msg.kp_velocity.push_back(msg->kp_velocity[i]);

    joint_command_msg.i_effort_min.push_back(msg->i_effort_min[i]);
    joint_command_msg.i_effort_max.push_back(msg->i_effort_max[i]);
  }
  if(ros::ok()) {
    joint_cmd_pub_.publish(joint_command_msg);
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
  osrf_msgs::JointCommands joint_command_msg;
  for (int i=0; i<msg->num_joints; i++) {
    joint_command_msg.name.push_back(msg->joint_name[i]);
    joint_command_msg.position.push_back(msg->joint_position[i]);
    joint_command_msg.velocity.push_back(0);
    joint_command_msg.effort.push_back(0);
  }
  if(ros::ok()) {
    joint_cmd_pub_.publish(joint_command_msg);
  } 
  
  ros::spinOnce();
  cout << "... Reconfiguration complete\n";
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
