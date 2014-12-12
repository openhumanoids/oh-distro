// Selective ros2lcm translator
// mfallon
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
//#include <atlas_msgs/ForceTorqueSensors.h>
//#include <trooper_mlc_msgs/FootSensor.h>
//#include <trooper_mlc_msgs/CachedRawIMUData.h>
//#include <trooper_mlc_msgs/RawIMUData.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"

#include "lcmtypes/pronto/multisense_state_t.hpp"
////#include <lcmtypes/multisense.hpp>
////#include "lcmtypes/pronto/imu_t.hpp"

using namespace std;

class App{
public:
  App(ros::NodeHandle node_, bool send_ground_truth_);
  ~App();

private:
  bool send_ground_truth_; // publish control msgs to LCM
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  // Atlas Joints and FT sensor
  ros::Subscriber  joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  //void appendFootSensor(pronto::force_torque_t& msg_out , trooper_mlc_msgs::FootSensor msg_in);
  
  // The position and orientation from BDI's own estimator:
  ros::Subscriber pose_bdi_sub_;
  void pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg);

  // The position and orientation from a vicon system:
  ros::Subscriber pose_vicon_sub_;
  void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

  // Multisense Joint Angles:
  ros::Subscriber  head_joint_states_sub_;  
  void head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg); 
  
  // Laser:
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);  

  // LM:
  //void foot_sensor_cb(const trooper_mlc_msgs::FootSensorConstPtr& msg);  
  //ros::Subscriber foot_sensor_sub_;
  //trooper_mlc_msgs::FootSensor foot_sensor_;

  void imu_cb(const sensor_msgs::ImuConstPtr& msg);
  ros::Subscriber imu_sub_;
  sensor_msgs::Imu imu_msg_;

  geometry_msgs::PoseStamped pose_msg_;

  //void imu_batch_cb(const trooper_mlc_msgs::CachedRawIMUDataConstPtr& msg);
  //ros::Subscriber imu_batch_sub_;
  //sensor_msgs::Imu imu_batch_msg_;

  void behavior_cb(const std_msgs::Int32ConstPtr& msg);
  ros::Subscriber behavior_sub_;

  void sendMultisenseState(int64_t utime, float position, float velocity);
  
  int64_t last_joint_state_utime_;
  bool verbose_;
};

App::App(ros::NodeHandle node_, bool send_ground_truth_) :
    send_ground_truth_(send_ground_truth_), node_(node_){
  ROS_INFO("Initializing Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Atlas Joints and FT sensor
  joint_states_sub_ = node_.subscribe(string("/atlas/outputs/joint_states"), 100, &App::joint_states_cb,this);

  // The position and orientation from BDI's own estimator (or GT from Gazebo):
  if (send_ground_truth_){
    pose_bdi_sub_ = node_.subscribe(string("/ground_truth_odom"), 100, &App::pose_bdi_cb,this);
  }else{
    //pose_bdi_sub_ = node_.subscribe(string("/controller_mgr/bdi_odom"), 100, &App::pose_bdi_cb,this);
    pose_bdi_sub_ = node_.subscribe(string("/robot_pose_service/odom"), 100, &App::pose_bdi_cb,this);
  }
  pose_vicon_sub_ = node_.subscribe(string("/atlas/outputs/rootPose"), 100, &App::pose_cb,this);


  // Multisense Joint Angles:
  head_joint_states_sub_ = node_.subscribe(string("/spindle_state"), 100, &App::head_joint_states_cb,this);

  // Laser:
  rotating_scan_sub_ = node_.subscribe(string("/multisense/lidar_scan"), 100, &App::rotating_scan_cb,this);

  // LM:
  //foot_sensor_sub_ = node_.subscribe(string("/foot_contact_service/foot_sensor"), 100, &App::foot_sensor_cb,this);
  imu_sub_ = node_.subscribe(string("/imu_publisher_service/imu"), 100, &App::imu_cb,this);
  //imu_batch_sub_ = node_.subscribe(string("/imu_publisher_service/raw_imu"), 100, &App::imu_batch_cb,this);

  behavior_sub_ = node_.subscribe(string("/current_behavior"), 100, &App::behavior_cb,this);
  verbose_ = false;
};

App::~App()  {
}


//void App::foot_sensor_cb(const trooper_mlc_msgs::FootSensorConstPtr& msg){
//  foot_sensor_ = *msg;
//}



void App::behavior_cb(const std_msgs::Int32ConstPtr& msg){
//  ROS_ERROR("BHER %d", msg->data );
  pronto::atlas_behavior_t msg_out;

  msg_out.utime = last_joint_state_utime_;
  int temp = (int) msg->data;
  msg_out.behavior = (int) temp;
//  std::cout << msg_out.behavior << " out\n";
  lcm_publish_.publish("ATLAS_BEHAVIOR", &msg_out);
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
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcm_publish_.publish(channel.c_str(), &scan_out);

}


int gt_counter =0;
void App::pose_bdi_cb(const nav_msgs::OdometryConstPtr& msg){
  if (gt_counter%1000 ==0){
    ROS_ERROR("BDI  [%d]", gt_counter );
  }  
  gt_counter++;

  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);

  if (verbose_)
    std::cout <<"                                                            " << pose_msg.utime << " bdi\n";

  pose_msg.pos[0] = msg->pose.pose.position.x;
  pose_msg.pos[1] = msg->pose.pose.position.y;
  pose_msg.pos[2] = msg->pose.pose.position.z;
  // what about orientation in imu msg?
  pose_msg.orientation[0] =  msg->pose.pose.orientation.w;
  pose_msg.orientation[1] =  msg->pose.pose.orientation.x;
  pose_msg.orientation[2] =  msg->pose.pose.orientation.y;
  pose_msg.orientation[3] =  msg->pose.pose.orientation.z;

  // This transformation is NOT correct for Trooper
  // April 2014: added conversion to body frame - so that both rates are in body frame
  Eigen::Matrix3d R = Eigen::Matrix3d( Eigen::Quaterniond( msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                                           msg->pose.pose.orientation.y, msg->pose.pose.orientation.z ));
  Eigen::Vector3d lin_body_vel  = R*Eigen::Vector3d ( msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                                      msg->twist.twist.linear.z );
  pose_msg.vel[0] = lin_body_vel[0];
  pose_msg.vel[1] = lin_body_vel[1];
  pose_msg.vel[2] = lin_body_vel[2];


  // this is the body frame rate
  pose_msg.rotation_rate[0] = msg->twist.twist.angular.x;
  pose_msg.rotation_rate[1] = msg->twist.twist.angular.y;
  pose_msg.rotation_rate[2] = msg->twist.twist.angular.z;
  // prefer to take all the info from one source
//  pose_msg.rotation_rate[0] = imu_msg_.angular_velocity.x;
//  pose_msg.rotation_rate[1] = imu_msg_.angular_velocity.y;
//  pose_msg.rotation_rate[2] = imu_msg_.angular_velocity.z;
  
  // Frame?
  pose_msg.accel[0] = imu_msg_.linear_acceleration.x;
  pose_msg.accel[1] = imu_msg_.linear_acceleration.y;
  pose_msg.accel[2] = imu_msg_.linear_acceleration.z;

  lcm_publish_.publish("POSE_BDI", &pose_msg);   
  lcm_publish_.publish("POSE_BODY", &pose_msg);    // for now
}

void App::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg){
  pose_msg_ = *msg;

  bot_core::pose_t pose_msg;
  pose_msg.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  pose_msg.pos[0] = msg->pose.position.x;
  pose_msg.pos[1] = msg->pose.position.y;
  pose_msg.pos[2] = msg->pose.position.z;
  pose_msg.orientation[0] =  msg->pose.orientation.w;
  pose_msg.orientation[1] =  msg->pose.orientation.x;
  pose_msg.orientation[2] =  msg->pose.orientation.y;
  pose_msg.orientation[3] =  msg->pose.orientation.z;

  lcm_publish_.publish("POSE_BODY", &pose_msg);
}




void App::imu_cb(const sensor_msgs::ImuConstPtr& msg){
  imu_msg_ = *msg;

}

/*
void App::imu_batch_cb(const trooper_mlc_msgs::CachedRawIMUDataConstPtr& msg){

  pronto::atlas_raw_imu_batch_t imu;
  imu.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);

  if (verbose_)
    std::cout <<"                              " << imu.utime << " imu\n";

  imu.num_packets = 15;
  for (size_t i=0; i < 15 ; i++){
    
    //std::cout << i
    //  << " | " <<  msg->data[i].imu_timestamp
    //  << " | " <<  msg->data[i].packet_count
    //  << " | " <<  msg->data[i].dax << " " << msg->data[i].day << " " << msg->data[i].daz
    //  << " | " <<  msg->data[i].ddx << " " << msg->data[i].ddy << " " << msg->data[i].ddz << "\n";
    
    pronto::atlas_raw_imu_t raw;
    raw.utime = msg->data[i].imu_timestamp;
    raw.packet_count = msg->data[i].packet_count;
    raw.delta_rotation[0] = msg->data[i].dax;
    raw.delta_rotation[1] = msg->data[i].day;
    raw.delta_rotation[2] = msg->data[i].daz;
    
    raw.linear_acceleration[0] = msg->data[i].ddx;
    raw.linear_acceleration[1] = msg->data[i].ddy;
    raw.linear_acceleration[2] = msg->data[i].ddz;
    imu.raw_imu.push_back( raw );
  }
  lcm_publish_.publish( ("ATLAS_IMU_BATCH") , &imu);
}
*/


void App::head_joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  int64_t utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  float position = msg->position[0];
  float velocity = msg->velocity[0];
  sendMultisenseState(utime, position, velocity);
}

void App::sendMultisenseState(int64_t utime, float position, float velocity){
  pronto::multisense_state_t msg_out;
  msg_out.utime = utime;
  for (std::vector<int>::size_type i = 0; i < 13; i++)  {
    msg_out.joint_name.push_back("z");      
    msg_out.joint_position.push_back(0);      
    msg_out.joint_velocity.push_back(0);
    msg_out.joint_effort.push_back(0);
  }  
  msg_out.num_joints = 13;

  msg_out.joint_position[0] = position;
  msg_out.joint_velocity[0] = velocity;
  msg_out.joint_name[0] = "hokuyo_joint";

  msg_out.joint_name[1] = "pre_spindle_cal_x_joint";
  msg_out.joint_name[2] = "pre_spindle_cal_y_joint";
  msg_out.joint_name[3] = "pre_spindle_cal_z_joint";

  msg_out.joint_name[4] = "pre_spindle_cal_roll_joint";
  msg_out.joint_name[5] = "pre_spindle_cal_pitch_joint";
  msg_out.joint_name[6] = "pre_spindle_cal_yaw_joint";

  msg_out.joint_name[7] = "post_spindle_cal_x_joint";
  msg_out.joint_name[8] = "post_spindle_cal_x_joint";
  msg_out.joint_name[9] = "post_spindle_cal_x_joint";

  msg_out.joint_name[10] = "post_spindle_cal_roll_joint";
  msg_out.joint_name[11] = "post_spindle_cal_pitch_joint";
  msg_out.joint_name[12] = "post_spindle_cal_yaw_joint";

  lcm_publish_.publish("MULTISENSE_STATE", &msg_out);  
}


inline int  getIndex(const std::vector<std::string> &vec, const std::string &str) {
  return std::find(vec.begin(), vec.end(), str) - vec.begin();
}

int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){

  int n_joints = msg->position.size();
  
  pronto::robot_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  

  if (verbose_)
    std::cout << msg_out.utime << " jnt\n";

  double elapsed_utime = (msg_out.utime - last_joint_state_utime_)*1E-6;
  if (elapsed_utime>0.004)
    std::cout << elapsed_utime << "   is elapsed_utime in sec\n";

  msg_out.joint_position.assign(n_joints , 0  );
  msg_out.joint_velocity.assign(n_joints , 0  );
  msg_out.joint_effort.assign(n_joints , 0  );
  msg_out.num_joints = n_joints;
  msg_out.joint_name= msg->name;
  for (int i = 0; i < n_joints; i++)  {
    msg_out.joint_position[ i ] = msg->position[ i ];      
    msg_out.joint_velocity[ i ] =0;// (double) msg->velocity[ i ];
    msg_out.joint_effort[ i ] = 0;//msg->effort[i];
  }

  for (int i = 0; i < n_joints; i++)  {
    if (msg_out.joint_name[i] == "l_arm_shz"){
      msg_out.joint_name[i] = "l_arm_usy";
    }
    if (msg_out.joint_name[i] == "l_arm_wry"){
      msg_out.joint_name[i] = "l_arm_uwy";
    }
    if (msg_out.joint_name[i] == "l_arm_wrx"){
      msg_out.joint_name[i] = "l_arm_mwx";
    }


    if (msg_out.joint_name[i] == "r_arm_shz"){
      msg_out.joint_name[i] = "r_arm_usy";
    }
    if (msg_out.joint_name[i] == "r_arm_wry"){
      msg_out.joint_name[i] = "r_arm_uwy";
    }
    if (msg_out.joint_name[i] == "r_arm_wrx"){
      msg_out.joint_name[i] = "r_arm_mwx";
    }

    if (msg_out.joint_name[i] == "neck_ry"){
      msg_out.joint_name[i] = "neck_ay";
    }

    if (msg_out.joint_name[i] == "hokuyo_joint"){
      //double output = remainderf( msg_out.joint_position[i] , M_PI);
      //std::cout << (msg_out.joint_position[i]) << " "  << output << "\n";
      //msg_out.joint_position[i] = output;
      //msg_out.joint_name[i] = "hokuyo_link";
    }

  }

  // Append FT sensor info
  pronto::force_torque_t force_torque;
//  appendFootSensor(force_torque, foot_sensor_);
  msg_out.force_torque = force_torque;


  msg_out.pose.translation.x = pose_msg_.pose.position.x;
  msg_out.pose.translation.y = pose_msg_.pose.position.y;
  msg_out.pose.translation.z = pose_msg_.pose.position.z;

  msg_out.pose.rotation.w = pose_msg_.pose.orientation.w;
  msg_out.pose.rotation.x = pose_msg_.pose.orientation.x;
  msg_out.pose.rotation.y = pose_msg_.pose.orientation.y;
  msg_out.pose.rotation.z = pose_msg_.pose.orientation.z;

  lcm_publish_.publish("EST_ROBOT_STATE", &msg_out);
  
  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcm_publish_.publish("ROBOT_UTIME", &utime_msg); 

  //sendMultisenseState(joint_utime, msg->position[22], 0);

  last_joint_state_utime_ = joint_utime;
}


/*void App::appendFootSensor(pronto::force_torque_t& msg_out , trooper_mlc_msgs::FootSensor msg_in){
  msg_out.l_foot_force_z  =  msg_in.left_fz;
  msg_out.l_foot_torque_x =  msg_in.left_mx;
  msg_out.l_foot_torque_y  =  msg_in.left_my;
  msg_out.r_foot_force_z  =  msg_in.right_fz;
  msg_out.r_foot_torque_x  =  msg_in.right_mx;
  msg_out.r_foot_torque_y  =  msg_in.right_my;

  msg_out.l_hand_force[0] =  0;
  msg_out.l_hand_force[1] =  0;
  msg_out.l_hand_force[2] =  0;
  msg_out.l_hand_torque[0] = 0;
  msg_out.l_hand_torque[1] = 0;
  msg_out.l_hand_torque[2] = 0;
  msg_out.r_hand_force[0] =  0;
  msg_out.r_hand_force[1] =  0;
  msg_out.r_hand_force[2] =  0;
  msg_out.r_hand_torque[0] =  0;
  msg_out.r_hand_torque[1] =  0;
  msg_out.r_hand_torque[2] =  0;
}
*/

int main(int argc, char **argv){
  bool send_ground_truth = false;  

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh, send_ground_truth);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Translator Ready");
  ros::spin();
  return 0;
}
