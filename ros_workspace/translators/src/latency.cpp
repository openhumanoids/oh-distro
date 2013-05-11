#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>


#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/stereo_camera_model.h>

#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>


#include <ros/callback_queue.h>

#include <latency/latency.hpp>


#define VERBOSE false
using namespace std;
class App{
public:
  App(ros::NodeHandle node_);
  ~App();

private:
  ros::NodeHandle node_;
  
  ros::Subscriber  joint_states_sub_, joint_commands_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void joint_commands_cb(const osrf_msgs::JointCommandsConstPtr& msg);  
  Latency* latency_;  
  std::string message_;
};

App::App(ros::NodeHandle node_){
  ROS_INFO("Initializing Latency");
  latency_ = new Latency();  

  joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 1000, &App::joint_states_cb,this);
  joint_commands_sub_ = node_.subscribe(string("/atlas/joint_commands"), 1000, &App::joint_commands_cb,this);
  message_ = "ROS ";

};

App::~App()  {
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  latency_->add_from( (int64_t) floor(msg->header.stamp.toNSec()/1000)   , _timestamp_now() );
}

void App::joint_commands_cb(const osrf_msgs::JointCommandsConstPtr& msg){
  latency_->add_to(  (int64_t) floor(msg->header.stamp.toNSec()/1000)   , _timestamp_now(), message_ );
}


int main(int argc, char **argv){
  ros::init(argc, argv, "latency");
  
  ros::CallbackQueue local_callback_queue;
  ros::NodeHandle nh;
  nh.setCallbackQueue(&local_callback_queue);
  
  App *app = new App(nh );
  std::cout << "ros latency ready\n";
  while (ros::ok()){
    local_callback_queue.callAvailable(ros::WallDuration(0.01));
  }  
  return 0;
}
