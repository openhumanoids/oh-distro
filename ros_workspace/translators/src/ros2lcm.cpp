// Selective ros2lcm translator
// mfallon aug,sept 2012
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <atlas_msgs/VRCScore.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>


#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/multisense.hpp>

using namespace std;


class BDIJointOrder { 
public:
  BDIJointOrder() {}
  BDIJointOrder(int index, std::string name):
    index(index), name(name){};
  int index;
  string name;
};

class App{
public:
  App(ros::NodeHandle node_, bool send_ground_truth_);
  ~App();

private:
  bool send_ground_truth_; // publish control msgs to LCM
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
	std::map<std::string,BDIJointOrder> jointNameMap;

  bool received_head_joint_states_;
  bool received_l_hand_joint_states_;
  bool received_r_hand_joint_states_;
  bool received_robot_joint_states_;

  // Clock:
  ros::Subscriber clock_sub_;
  void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);
  // VRC Score:
  ros::Subscriber vrc_score_sub_;
  void vrc_score_cb(const atlas_msgs::VRCScoreConstPtr& msg);
  
  // All of this data is mashed down into one LCM message - Robot State ////
  sensor_msgs::JointState l_hand_joint_states_, r_hand_joint_states_, robot_joint_states_, head_joint_states_;  
  ros::Subscriber  joint_states_sub_, l_hand_joint_states_sub_, r_hand_joint_states_sub_, head_joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void l_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void r_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  // Newer version for DRC: (july 2013):
  void appendLimbSensor(drc::force_torque_t& msg_out , atlas_msgs::ForceTorqueSensors msg_in);
  
  void publishRobotState(int64_t utime_in);
  bool init_recd_[2]; // have recived gt [0], robot joints [1]

  ros::Subscriber end_effector_sensors_sub_;  
  void end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg);  
  atlas_msgs::ForceTorqueSensors end_effector_sensors_;
  
  ros::Subscriber ground_truth_odom_sub_;  
  void ground_truth_odom_cb(const nav_msgs::OdometryConstPtr& msg);  
  nav_msgs::Odometry ground_truth_odom_;
  //////////////////////////////////////////////////////////////////////////
  
  // Imus:
  ros::Subscriber torso_imu_sub_;
  void torso_imu_cb(const sensor_msgs::ImuConstPtr& msg);
  void send_imu(const sensor_msgs::ImuConstPtr& msg,string channel ); 
  ros::Subscriber head_imu_sub_;
  void head_imu_cb(const sensor_msgs::ImuConstPtr& msg);
  // Laser:
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);  
  
  ros::Subscriber vrc_Bdown_sub_, vrc_Bup_sub_;
  void vrc_Bdown_cb(const std_msgs::StringConstPtr& msg);
  void vrc_Bup_cb(const std_msgs::StringConstPtr& msg);
  int64_t Bdown_, Bup_;
  
};

App::App(ros::NodeHandle node_, bool send_ground_truth_) :
    node_(node_), send_ground_truth_(send_ground_truth_){
  ROS_INFO("Initializing Translator");

  received_head_joint_states_ = false;
  received_l_hand_joint_states_ = false;
  received_r_hand_joint_states_ = false;
  received_robot_joint_states_ = false;

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  // Clock (in the joint states pub now)
  //   clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this);
  
  // Score:
  vrc_score_sub_ = node_.subscribe(string("/vrc_score"), 100, &App::vrc_score_cb,this);
  // Score from OSRF netshaper:
  vrc_Bup_sub_ = node_.subscribe(string("/vrc/bytes/remaining/uplink"), 100, &App::vrc_Bup_cb,this);
  vrc_Bdown_sub_ = node_.subscribe(string("/vrc/bytes/remaining/downlink"), 100, &App::vrc_Bdown_cb,this);
  Bdown_=0;
  Bup_ =0;
  
  // IMU:
  torso_imu_sub_ = node_.subscribe(string("/atlas/imu"), 100, &App::torso_imu_cb,this);

  // Robot State:
  joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 100, &App::joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
  head_joint_states_sub_ = node_.subscribe(string("/multisense_sl/joint_states"), 100, &App::head_joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
  l_hand_joint_states_sub_ = node_.subscribe(string("/sandia_hands/l_hand/joint_states"), 100, &App::l_hand_joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
  r_hand_joint_states_sub_ = node_.subscribe(string("/sandia_hands/r_hand/joint_states"), 100, &App::r_hand_joint_states_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
  end_effector_sensors_sub_ = node_.subscribe(string("/atlas/force_torque_sensors"), 100, &App::end_effector_sensors_cb,this);

  init_recd_[0]=false; // had ground_truth_odom been received?
  init_recd_[1]=false; // have joint angles been received?
  
  ///////////////////////////// Ground Truth Odom ///////////////////////////////////////
  // initialize with known (but ridiculous) state - to expose any GT data feeding through
  ground_truth_odom_.pose.pose.position.x =0.;
  ground_truth_odom_.pose.pose.position.y =0.;
  ground_truth_odom_.pose.pose.position.z =0.;
  ground_truth_odom_.pose.pose.orientation.w =0.; // upside down (to make it obvious)
  ground_truth_odom_.pose.pose.orientation.x =1.;
  ground_truth_odom_.pose.pose.orientation.y =0.;
  ground_truth_odom_.pose.pose.orientation.z =0.;
  ground_truth_odom_.twist.twist.linear.x = std::numeric_limits<int>::min();
  ground_truth_odom_.twist.twist.linear.y = std::numeric_limits<int>::min();
  ground_truth_odom_.twist.twist.linear.z = std::numeric_limits<int>::min();
  ground_truth_odom_.twist.twist.angular.x = std::numeric_limits<int>::min();
  ground_truth_odom_.twist.twist.angular.y = std::numeric_limits<int>::min();
  ground_truth_odom_.twist.twist.angular.z = std::numeric_limits<int>::min();
  
  if (send_ground_truth_){
    ground_truth_odom_sub_ = node_.subscribe(string("/ground_truth_odom"), 100, &App::ground_truth_odom_cb,this);//, ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
  }else{
    init_recd_[0]=true; // NB: needed to fool this program into believing it has got GTO when we aren't going to provide it
  }

  
  // IMU:
  head_imu_sub_ = node_.subscribe(string("/multisense_sl/imu"), 100, &App::head_imu_cb,this);
  // Laser:
  rotating_scan_sub_ = node_.subscribe(string("/multisense_sl/laser/scan"), 100, &App::rotating_scan_cb,this);

	// maps joint names from sim/VRC format to BDI format
/*
	jointNameMap["back_ubx"] = "back_bkx";
	jointNameMap["back_mby"] = "back_bky";
	jointNameMap["back_lbz"] = "back_bkz";
        jointNameMap["neck_ay"] = "neck_ay";        
	jointNameMap["l_leg_lax"] = "l_leg_akx";
	jointNameMap["l_leg_uay"] = "l_leg_aky";
	jointNameMap["l_leg_kny"] = "l_leg_kny";
	jointNameMap["l_leg_mhx"] = "l_leg_hpx";
	jointNameMap["l_leg_lhy"] = "l_leg_hpy";
	jointNameMap["l_leg_uhz"] = "l_leg_hpz";
	jointNameMap["r_leg_lax"] = "r_leg_akx";
	jointNameMap["r_leg_uay"] = "r_leg_aky";
	jointNameMap["r_leg_kny"] = "r_leg_kny";
	jointNameMap["r_leg_mhx"] = "r_leg_hpx";
	jointNameMap["r_leg_lhy"] = "r_leg_hpy";
	jointNameMap["r_leg_uhz"] = "r_leg_hpz";
	jointNameMap["l_arm_mwx"] = "l_arm_mwx";
	jointNameMap["l_arm_uwy"] = "l_arm_uwy";
	jointNameMap["l_arm_elx"] = "l_arm_elx";
	jointNameMap["l_arm_ely"] = "l_arm_ely";
	jointNameMap["l_arm_shx"] = "l_arm_shx";
	jointNameMap["l_arm_usy"] = "l_arm_usy";
	jointNameMap["r_arm_mwx"] = "r_arm_mwx";
	jointNameMap["r_arm_uwy"] = "r_arm_uwy";
	jointNameMap["r_arm_elx"] = "r_arm_elx";
	jointNameMap["r_arm_ely"] = "r_arm_ely";
	jointNameMap["r_arm_shx"] = "r_arm_shx";
	jointNameMap["r_arm_usy"] = "r_arm_usy";
        */
        
  // Mapping between OSRF joints and BDI's { index and joint_names }:
  jointNameMap["back_ubx"] = { 2, "back_bkx"};
  jointNameMap["back_mby"] = { 1, "back_bky"};
  jointNameMap["back_lbz"] = { 0, "back_bkz"};
  jointNameMap["neck_ay"] = { 3, "neck_ay"};
  jointNameMap["l_leg_lax"] = { 9, "l_leg_akx"};
  jointNameMap["l_leg_uay"] = { 8, "l_leg_aky"};
  jointNameMap["l_leg_kny"] = { 7, "l_leg_kny"};
  jointNameMap["l_leg_mhx"] = { 5, "l_leg_hpx"};
  jointNameMap["l_leg_lhy"] = { 6, "l_leg_hpy"};
  jointNameMap["l_leg_uhz"] = { 4, "l_leg_hpz"};
  jointNameMap["r_leg_lax"] = { 15, "r_leg_akx"};
  jointNameMap["r_leg_uay"] = { 14, "r_leg_aky"};
  jointNameMap["r_leg_kny"] = { 13, "r_leg_kny"};
  jointNameMap["r_leg_mhx"] = { 11, "r_leg_hpx"};
  jointNameMap["r_leg_lhy"] = { 12, "r_leg_hpy"};
  jointNameMap["r_leg_uhz"] = { 10, "r_leg_hpz"};
  jointNameMap["l_arm_mwx"] = { 21, "l_arm_mwx"};
  jointNameMap["l_arm_uwy"] = { 20, "l_arm_uwy"};
  jointNameMap["l_arm_elx"] = { 19, "l_arm_elx"};
  jointNameMap["l_arm_ely"] = { 18, "l_arm_ely"};
  jointNameMap["l_arm_shx"] = { 17, "l_arm_shx"};
  jointNameMap["l_arm_usy"] = { 16, "l_arm_usy"};
  jointNameMap["r_arm_mwx"] = { 27, "r_arm_mwx"};
  jointNameMap["r_arm_uwy"] = { 26, "r_arm_uwy"};
  jointNameMap["r_arm_elx"] = { 25, "r_arm_elx"};
  jointNameMap["r_arm_ely"] = { 24, "r_arm_ely"};
  jointNameMap["r_arm_shx"] = { 23, "r_arm_shx"};
  jointNameMap["r_arm_usy"] = { 22, "r_arm_usy"};        

  
};

App::~App()  {
}

void App::vrc_Bdown_cb(const std_msgs::StringConstPtr& msg){
  Bdown_ = atoi(msg->data.c_str());
}
void App::vrc_Bup_cb(const std_msgs::StringConstPtr& msg){
  Bup_ = atoi(msg->data.c_str());
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


void App::vrc_score_cb(const atlas_msgs::VRCScoreConstPtr& msg){
  //std::cout << "got vrc score\n";  
  drc::score_t score;
  score.utime = _timestamp_now();
  score.wall_time=(int64_t) floor(msg->wall_time.toNSec()/1000);  
  score.sim_time=(int64_t) floor(msg->sim_time.toNSec()/1000);  
  score.wall_time_elapsed=(int64_t) floor(msg->wall_time_elapsed.toNSec()/1000);  
  score.sim_time_elapsed=(int64_t) floor(msg->sim_time_elapsed.toNSec()/1000);  
  score.completion_score = msg->completion_score;
  score.falls = msg->falls;
  score.task_type = msg->task_type;
  score.bytes_downlink_remaining = Bdown_;
  score.bytes_uplink_remaining = Bup_;
  lcm_publish_.publish("VRC_SCORE", &score);

  if (!msg->message.empty() ){
    // any string messages from gazebo come as occasional strings:
    drc::system_status_t m;
    m.utime = (int64_t) floor(msg->sim_time.toNSec()/1000);
    m.system = drc::system_status_t::MESSAGING;
    m.importance = drc::system_status_t::VERY_IMPORTANT;
    m.frequency = drc::system_status_t::LOW_FREQUENCY;

    std::stringstream ss;
    ss << "VRC Score: " 
       <<  msg->wall_time_elapsed.toSec() << " Seconds Elapsed | "
       <<  msg->completion_score << " Completion | "
       <<  msg->falls << " Falls | "
       <<  (int) msg->task_type << " Task";
    m.value = ss.str();
    std::cout << ss.str() << " \n";  
    lcm_publish_.publish("SYSTEM_STATUS", &m); // for simplicity stick this out
    
    m.value = string( "VRC Score: " ) + msg->message;
    lcm_publish_.publish("SYSTEM_STATUS", &m);
    lcm_publish_.publish("VRC_SCORE_MESSAGE", &m); // for convenience
  }
}

void App::send_imu(const sensor_msgs::ImuConstPtr& msg,string channel ){
  drc::imu_t imu_msg;
  imu_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);  
  //imu_msg.frame_id = msg->header->frame_id;
  imu_msg.orientation[0] = msg->orientation.w; // NB: order here is wxyz
  imu_msg.orientation[1] = msg->orientation.x;
  imu_msg.orientation[2] = msg->orientation.y;
  imu_msg.orientation[3] = msg->orientation.z;
  imu_msg.angular_velocity[0] = msg->angular_velocity.x;
  imu_msg.angular_velocity[1] = msg->angular_velocity.y;
  imu_msg.angular_velocity[2] = msg->angular_velocity.z;
  imu_msg.linear_acceleration[0] = msg->linear_acceleration.x;
  imu_msg.linear_acceleration[1] = msg->linear_acceleration.y;
  imu_msg.linear_acceleration[2] = msg->linear_acceleration.z;
  for (size_t i=0;i<9;i++){
    imu_msg.orientation_covariance[i] = msg->orientation_covariance[i];
    imu_msg.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
    imu_msg.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
  }
  lcm_publish_.publish(channel, &imu_msg);
}
void App::torso_imu_cb(const sensor_msgs::ImuConstPtr& msg){
  send_imu(msg,"TORSO_IMU");
}

void App::head_imu_cb(const sensor_msgs::ImuConstPtr& msg){
  send_imu(msg,"HEAD_IMU");
}


int scan_counter=0;
void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  if (scan_counter%80 ==0){
    ROS_ERROR("LSCAN [%d]", scan_counter );
    //std::cout << "SCAN " << scan_counter << "\n";
  }  
  scan_counter++;
  send_lidar(msg, "SCAN");
}
void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities; // currently all identical
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;

  lcm_publish_.publish(channel.c_str(), &scan_out);
}


void App::end_effector_sensors_cb(const atlas_msgs::ForceTorqueSensorsConstPtr& msg){
  end_effector_sensors_ = *msg;
}

int gt_counter =0;
void App::ground_truth_odom_cb(const nav_msgs::OdometryConstPtr& msg){
  if (gt_counter%200 ==0){
    std::cout << "GRTH " << gt_counter << "\n";
  }  
  gt_counter++;

  ground_truth_odom_ = *msg;
  init_recd_[0] =true;
}

/// Locally cache the joint states:
void App::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  
  multisense::state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
    msg_out.joint_name.push_back(msg->name[i]);      
    msg_out.joint_position.push_back(msg->position[i]);      
    msg_out.joint_velocity.push_back(msg->velocity[i]);
    msg_out.joint_effort.push_back( msg->effort[i] );
  }  
  msg_out.num_joints = msg->name.size();
  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
  
  head_joint_states_= *msg;
  received_head_joint_states_ = true;
  
}

void App::l_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  
  drc::hand_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
    msg_out.joint_name.push_back(msg->name[i]);      
    msg_out.joint_position.push_back(msg->position[i]);      
    msg_out.joint_velocity.push_back(msg->velocity[i]);
    msg_out.joint_effort.push_back( msg->effort[i] );
  }  
  msg_out.num_joints = msg->name.size();
  lcm_publish_.publish("SANDIA_LEFT_STATE", &msg_out);  
  
  l_hand_joint_states_= *msg;
  received_l_hand_joint_states_ = true;
}

void App::r_hand_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){

  drc::hand_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
    msg_out.joint_name.push_back(msg->name[i]);      
    msg_out.joint_position.push_back(msg->position[i]);      
    msg_out.joint_velocity.push_back(msg->velocity[i]);
    msg_out.joint_effort.push_back( msg->effort[i] );
  }  
  msg_out.num_joints = msg->name.size();
  lcm_publish_.publish("SANDIA_RIGHT_STATE", &msg_out);  
  
  r_hand_joint_states_= *msg;
  received_r_hand_joint_states_ = true;
}

int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  if (js_counter%500 ==0){
    std::cout << "J ST " << js_counter << "\n";
  }  
  js_counter++;
  
  drc::atlas_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  
  
  
  msg_out.joint_position.assign(msg->name.size() , std::numeric_limits<int>::min()  );
  msg_out.joint_velocity.assign(msg->name.size() , std::numeric_limits<int>::min()  );
  msg_out.joint_effort.assign(msg->name.size() , std::numeric_limits<int>::min()  );
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
    if (jointNameMap.find(msg->name[i]) != jointNameMap.end()){
      // if the OSRF joint name is in the map then take the index and use it for the out going joints
      int index= jointNameMap[msg->name[i]].index;
      msg_out.joint_position [ index ] = msg->position[i];
      msg_out.joint_velocity [ index ] = msg->velocity[i];
      msg_out.joint_effort [ index ] = msg->effort[i];      
    }else{
      ROS_ERROR("OSRF-BDI Joint Matching Failed: Could not find %s", msg->name[i].c_str() );
    }
  }  
  msg_out.num_joints = msg->name.size();
  
  // Append sensor (newer approach)
  drc::force_torque_t force_torque;
  appendLimbSensor(force_torque, end_effector_sensors_);
  msg_out.force_torque = force_torque;
  lcm_publish_.publish("ATLAS_STATE", &msg_out);
  
  drc::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg);
  
  if (1==0){// DISABLED - now published by state_sync
    //////////////////////////////////////////////////////
    robot_joint_states_ = *msg; 
    init_recd_[1] =true; 
    received_robot_joint_states_ = true;
    publishRobotState(joint_utime);
  }
  
}


void App::appendLimbSensor(drc::force_torque_t& msg_out , atlas_msgs::ForceTorqueSensors msg_in){

  msg_out.l_foot_force_z =  msg_in.l_foot.force.z;
  msg_out.l_foot_torque_x = msg_in.l_foot.torque.x; 
  msg_out.l_foot_torque_y = msg_in.l_foot.torque.y;

  msg_out.r_foot_force_z =  msg_in.r_foot.force.z;
  msg_out.r_foot_torque_x = msg_in.r_foot.torque.x; 
  msg_out.r_foot_torque_y = msg_in.r_foot.torque.y;

  msg_out.l_hand_force[0] =  msg_in.l_hand.force.x;
  msg_out.l_hand_force[1] =  msg_in.l_hand.force.y;
  msg_out.l_hand_force[2] =  msg_in.l_hand.force.z;

  msg_out.l_hand_torque[0] =  msg_in.l_hand.torque.x;
  msg_out.l_hand_torque[1] =  msg_in.l_hand.torque.y;
  msg_out.l_hand_torque[2] =  msg_in.l_hand.torque.z;

  msg_out.r_hand_force[0] =  msg_in.r_hand.force.x;
  msg_out.r_hand_force[1] =  msg_in.r_hand.force.y;
  msg_out.r_hand_force[2] =  msg_in.r_hand.force.z;

  msg_out.r_hand_torque[0] =  msg_in.r_hand.torque.x;
  msg_out.r_hand_torque[1] =  msg_in.r_hand.torque.y;
  msg_out.r_hand_torque[2] =  msg_in.r_hand.torque.z;    
  
}



int main(int argc, char **argv){
  bool control_output = true;// by default
  bool send_ground_truth = false;  

  std::string mode_argument;
  if (argc >= 2){
     mode_argument = argv[1];
  }else {
    ROS_ERROR("Need to have another argument in the launch file");
  }

  if (mode_argument.compare("vrc_cheats_enabled") == 0){
    send_ground_truth = true;
  }else if (mode_argument.compare("vrc_cheats_disabled") == 0){
    send_ground_truth = false;    
  }else {
    ROS_ERROR("mode_argument not understood");
    std::cout << mode_argument << " is not understood\n";
    exit(-1);
  }

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  App *app = new App(nh, send_ground_truth);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Control Translator Sleeping: [%s]",  mode_argument.c_str());
  sleep(4);
  ROS_ERROR("ROS2LCM Control Translator Ready: [%s]",  mode_argument.c_str());
  ros::spin();
  return 0;
}
