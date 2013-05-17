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
#include <rosgraph_msgs/Clock.h>

#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <sensor_msgs/LaserScan.h>


#include <ros/callback_queue.h>

#include <frequency/frequency.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm,ros::NodeHandle node_,bool verbose);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleUtime(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const  drc::utime_t * msg);
  void handleAllMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan);
  
  int mode_;
  std::string message_;
  
  
  ros::Subscriber  joint_states_sub_, joint_commands_sub_;  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  void joint_commands_cb(const osrf_msgs::JointCommandsConstPtr& msg);  
  
  ros::Subscriber left_image_sub_;  
  void left_image_cb(const sensor_msgs::ImageConstPtr& msg);
  
  ros::Subscriber rotating_scan_sub_;
  void rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
  
  ///////// 
  ros::Subscriber clock_sub_;
  void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);  
  
private:
  
  
  ros::NodeHandle node_;
  Frequency* frequency_;  
  
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, ros::NodeHandle node_, bool verbose): _lcm(_lcm){
  frequency_ = new Frequency(0.5, verbose,"ROS");  
  //frequency_->readChannels("lcm");

  joint_states_sub_ = node_.subscribe(string("/atlas/joint_states"), 1000, &App::joint_states_cb,this);
  frequency_->addCounter("/atlas/joint_states", "JSTA");
  joint_commands_sub_ = node_.subscribe(string("/atlas/joint_commands"), 1000, &App::joint_commands_cb,this);
  frequency_->addCounter("/atlas/joint_commands", "JCOM");
  left_image_sub_ = node_.subscribe( "/multisense_sl/camera/left/image_raw", 10, &App::left_image_cb,this);
  frequency_->addCounter("/multisense_sl/camera/left/image_raw", "LCAM");
  rotating_scan_sub_ = node_.subscribe(string("/multisense_sl/laser/scan"), 10, &App::rotating_scan_cb,this);
  frequency_->addCounter("/multisense_sl/laser/scan", "SCAN");
  
  
  //////////////////////////////////////////////////////
  clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this);
  
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void App::clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  int64_t utime = (int64_t) floor(msg->clock.toNSec()/1000);
  frequency_->setUtime( utime );
}


////////////////////////////////////////////////////////////////////////////////////////////
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  frequency_->incrementCounter( "/atlas/joint_states" );
}
void App::joint_commands_cb(const osrf_msgs::JointCommandsConstPtr& msg){
  frequency_->incrementCounter( "/atlas/joint_commands" );
}

void App::left_image_cb(const sensor_msgs::ImageConstPtr& msg){
  frequency_->incrementCounter( "/multisense_sl/camera/left/image_raw" );
}

void App::rotating_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  frequency_->incrementCounter( "/multisense_sl/laser/scan" );
}

int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "frequency");
  int mode=0;
  bool verbose=true;
  string name="frequency";
  
  parser.add(mode, "m", "mode", "Mode [0,1]");
  parser.add(verbose, "v", "verbose", "Verbose");
  parser.add(name, "n", "name", "Name of ros process");
  parser.parse();

  cout << "name is: " << name << "\n";   
  cout << "mode is: " << mode << "\n"; 
  cout << "verbose is: " << verbose << "\n"; 
  
  ros::init(argc, argv, name);
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;
  
  
  ros::NodeHandle nh;
  App *app = new App(lcm, nh, verbose );
  std::cout << "launching frequency as " << name <<"\n";
  ros::spin();
  
  return 0;
}