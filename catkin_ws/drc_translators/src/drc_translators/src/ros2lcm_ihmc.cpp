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

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

using namespace std;

int width =800;//1024; // hardcoded
int height =800;//544; // hardcoded
uint8_t* singleimage_data = new uint8_t [2*2* width*height]; // 1 color scale image

class App{
public:
  App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  ros::Subscriber  joint_states_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  

  // The position and orientation:
  ros::Subscriber pose_robot_sub_;
  void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
  geometry_msgs::PoseStamped pose_msg_;
  int64_t last_joint_state_utime_;

  // Laser:
  void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);  
  void sendMultisenseState(int64_t utime, float position, float velocity);

  ros::Subscriber head_l_image_sub_;
  void head_l_image_cb(const sensor_msgs::ImageConstPtr& msg);
  void send_image(const sensor_msgs::ImageConstPtr& msg,string channel );

  bool verbose_;
  // Image Compression
  image_io_utils*  imgutils_;  
};

App::App(ros::NodeHandle node_) :
    node_(node_){
  ROS_INFO("Initializing Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  joint_states_sub_ = node_.subscribe(string("/atlas/outputs/joint_states"), 100, &App::joint_states_cb,this);
  pose_robot_sub_ = node_.subscribe(string("/atlas/outputs/rootPose"), 100, &App::pose_cb,this);
  rotating_scan_sub_ = node_.subscribe(string("/multisense/lidar_scan"), 100, &App::rotating_scan_cb,this);
  head_l_image_sub_ = node_.subscribe("/multisense/left/image_rect_color/compressed", 1, &App::head_l_image_cb,this);

  imgutils_ = new image_io_utils( lcm_publish_.getUnderlyingLCM(), width, height );  
  verbose_ = false;
};

App::~App()  {
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

  last_joint_state_utime_ = joint_utime;
}

int head_l_image_counter=0;
void App::head_l_image_cb(const sensor_msgs::ImageConstPtr& msg){
  if (scan_counter%80 ==0){
    ROS_ERROR("CAMLT [%d]", head_l_image_counter );
  }  
  head_l_image_counter++;
  send_image(msg,"CAMERA_LEFT");
}

void App::send_image(const sensor_msgs::ImageConstPtr& msg,string channel ){
  int64_t current_utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  int  n_colors=3;
  int isize = msg->data.size();
  bot_core::image_t lcm_img;
  lcm_img.utime =current_utime;
  lcm_img.width =msg->width;
  lcm_img.height =msg->height;
  lcm_img.nmetadata =0;
  lcm_img.row_stride=n_colors*msg->width;
  lcm_img.pixelformat =bot_core::image_t::PIXEL_FORMAT_RGB;
  lcm_img.data.assign(msg->data.begin(), msg->data.end());
  lcm_img.size = isize;
  lcm_publish_.publish(channel.c_str(), &lcm_img);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh);
  std::cout << "ros2lcm translator ready\n";
  ROS_ERROR("ROS2LCM Translator Ready");
  ros::spin();
  return 0;
}
