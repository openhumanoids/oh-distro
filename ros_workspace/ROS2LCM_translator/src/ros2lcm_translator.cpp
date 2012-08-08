// Selective ros2lcm translator.
//
// As of August 2012:
// - Translates ROS laser msgs and part of tf to LCM
//
// mfallon aug 2012
#include <ros/ros.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/message_filter.h>

#include <lcm/lcm-cpp.hpp>
//#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.hpp>

#define VERBOSE false
using namespace std;

class App{
  public:
    App();
    ~App();

    lcm_t* get_lcmref(){return lcmref_;}

  private:
    lcm_t* lcmref_ ;

    ros::NodeHandle node_;
    tf::Transformer transformer;

    ros::Subscriber tilt_scan_sub_, base_scan_sub_, odom_sub_, tf_sub_;
    void odom_cb(const nav_msgs::OdometryConstPtr& msg);
    void tf_cb(const tf::tfMessageConstPtr& msg);
    void base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
    void tilt_scan_cb(const sensor_msgs::LaserScanConstPtr& msg);
    ros::Subscriber clock_sub_;
    void clock_cb(const rosgraph_msgs::ClockConstPtr& msg);

    void send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
    void send_rigid_transform(tf::StampedTransform& transform,string channel );
};

App::App(){
  lcmref_ = lcm_create(NULL);

  tf_sub_ = node_.subscribe(string("/tf"), 10, &App::tf_cb,this);
  odom_sub_ = node_.subscribe(string("/base_odometry/odom"), 10, &App::odom_cb,this);
  base_scan_sub_ = node_.subscribe(string("/base_scan"), 10, &App::base_scan_cb,this);
  tilt_scan_sub_ = node_.subscribe(string("/tilt_scan"), 10, &App::tilt_scan_cb,this);

  clock_sub_ = node_.subscribe(string("/clock"), 10, &App::clock_cb,this);
};

App::~App()  {
}

void App::send_lidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core_planar_lidar_t scan_out;

  float range_line[msg->ranges.size()];
  for (size_t i=0; i < msg->ranges.size(); i++){
    range_line[i] = msg->ranges[i];
  }
  scan_out.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  scan_out.nranges =msg->ranges.size();
  scan_out.ranges=range_line;
  scan_out.nintensities=0;
  scan_out.intensities=NULL;
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  bot_core_planar_lidar_t_publish(lcmref_,channel.c_str(),&scan_out);
  if (VERBOSE) cout << channel << ": "<< scan_out.utime << "\n";
}

void App::clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  int64_t clock_utime = (int64_t) floor(msg->clock.toSec()  * 1E6);
  if (VERBOSE) cout << "got clock msg: " << clock_utime << "\n";
}

void App::tilt_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "TILT_SCAN");
}

void App::base_scan_cb(const sensor_msgs::LaserScanConstPtr& msg){
  send_lidar(msg, "BASE_SCAN");
}

void App::odom_cb(const nav_msgs::OdometryConstPtr& msg){
  if (VERBOSE) cout << "Odom\n";
  bot_core_pose_t pose;
  pose.utime = (int64_t) floor(msg->header.stamp.toSec()  * 1E6);
  pose.pos[0] = msg->pose.pose.position.x;
  pose.pos[1] = msg->pose.pose.position.y;
  pose.pos[2] = msg->pose.pose.position.z;
  pose.orientation[0] = msg->pose.pose.orientation.w;
  pose.orientation[1] = msg->pose.pose.orientation.x;
  pose.orientation[2] = msg->pose.pose.orientation.y;
  pose.orientation[3] = msg->pose.pose.orientation.z;
  bot_core_pose_t_publish(lcmref_,"POSE",&pose);
}

void App::tf_cb(const tf::tfMessageConstPtr& msg){
  //cout << "tf n: " << msg->transforms.size() << "\n";

  for (unsigned int i = 0; i < msg->transforms.size(); ++i){
    tf::StampedTransform transform;
    tf::transformStampedMsgToTF(msg->transforms[i], transform);

    try{
      transformer.setTransform(transform);
    }
    catch (tf::TransformException& ex){
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n",
          msg->transforms[i].child_frame_id.c_str(),
          msg->transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }

  // Lookup a transform
  try {
    tf::StampedTransform transform;
    tf::Quaternion t_quat;
    tf::Vector3 t;

    /////////////////////////
    // Faulty time stamp - do not republish for now:
    //transformer.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
    //send_rigid_transform(transform, "BODY_TO_BASE_SCAN");

    transformer.lookupTransform("/base_link", "/laser_tilt_link", ros::Time(0), transform);
    send_rigid_transform(transform, "BODY_TO_TILT_SCAN");

    transformer.lookupTransform("/base_link", "/narrow_stereo_link", ros::Time(0), transform);
    send_rigid_transform(transform, "BODY_TO_NARROW_STEREO");

    //transformer.lookupTransform("/odom_combined", "/base_link", ros::Time(0), transform);
    //send_rigid_transform(transform, "BODY_TO_NARROW_STEREO");

    //int64_t current_utime = (int64_t)  floor(transform.stamp_.toSec()  * 1E6);
    //cout << current_utime << " is current_utime\n";

  }
  catch (tf::TransformException ex)
  {
     ROS_ERROR("lookupTransform failed: %s", ex.what());
  }
}

void App::send_rigid_transform(tf::StampedTransform& transform,string channel ){
  tf::Quaternion t_quat= transform.getRotation();
  tf::Vector3 t= transform.getOrigin();

  bot_core_rigid_transform_t tf_out;
  tf_out.utime = (int64_t)  floor(transform.stamp_.toSec()  * 1E6);
  tf_out.trans[0] =t.x();
  tf_out.trans[1] =t.y();
  tf_out.trans[2] =t.z();
  tf_out.quat[0] =t_quat.w();
  tf_out.quat[1] =t_quat.x();
  tf_out.quat[2] =t_quat.y();
  tf_out.quat[3] =t_quat.z();
  bot_core_rigid_transform_t_publish(lcmref_,channel.c_str(),&tf_out);

  if (VERBOSE){
    std::cout << channel << ": "<< tf_out.utime << "\n";
    std::cout << "                ->                 : " << t.x() << " " << t.y() << " " << t.z() << std::endl;
    std::cout << "       "  << t_quat.w() << " " << t_quat.x() << " " << t_quat.y() << " " << t_quat.z() << std::endl;
  }
}

int main(int argc, char **argv){
  std::cout << "ros2lcm_translator launched\n";
  ros::init(argc, argv, "ros2lcm_translator");
  App *app = new App();
  ros::spin();
  return 0;
}
