// Selective ros2lcm translator for Edinburgh Kuka Arm
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
#include <trajectory_msgs/JointTrajectory.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/joint_state_t.hpp"

using namespace std;

class App{
public:
  App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;

  ros::Subscriber  joint_states_sub_;
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);

};

App::App(ros::NodeHandle node_) :
    node_(node_){
  ROS_INFO("Initializing KUKA LWR Joint State Translator");
  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  joint_states_sub_ = node_.subscribe(string("/joint_states"), 100, &App::joint_states_cb,this);

};

App::~App()  {
}


void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  int n_joints = msg->position.size();

  drc::joint_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  
  msg_out.joint_position.assign(n_joints , 0  );
  msg_out.joint_velocity.assign(n_joints , 0  );
  msg_out.joint_effort.assign(n_joints , 0  );
  msg_out.num_joints = n_joints;
  msg_out.joint_name= msg->name;
  for (int i = 0; i < n_joints; i++)  {
    msg_out.joint_position[ i ] = msg->position[ i ];
    msg_out.joint_velocity[ i ] = msg->velocity[ i ];
    msg_out.joint_effort[ i ] = msg->effort[i];
  }

  lcm_publish_.publish("KUKA_STATE", &msg_out);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh);
  std::cout << "ros2lcm KUKA lwr translator ready\n";
  ROS_ERROR("ROS2LCM KUKA LWR Joint State Translator Ready");
  ros::spin();
  return 0;
}
