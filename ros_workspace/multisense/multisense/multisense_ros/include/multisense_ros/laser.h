#ifndef MULTISENSE_ROS_LASER_H
#define MULTISENSE_ROS_LASER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_driver/multisense_driver.h>
#include <multisense_ros/tools.h>
#include <multisense_ros/state_publisher.h>
#include <multisense_ros/function_caller.h>
#include <LibSensorPodCommunications/LidarDataMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <LibSensorPodCommunications/LidarStartScanAckMessage.h>
#include <LibSensorPodCommunications/LidarStopScanAckMessage.h>

#include <lcmtypes/bot_core.hpp>
#include <lcm/lcm-cpp.hpp>

namespace multisense_ros
{

class Laser
{
public:
  Laser(multisense_driver::MultisenseDriver* driver);
  static const float EXPECTED_RATE;


private:
  // start or stop laser scan stream
  void startStream();
  void stopStream();

  void startAck(const boost::shared_ptr<const LidarStartScanAckMessage>& ack);
  void stopAck(const boost::shared_ptr<const LidarStopScanAckMessage>& ack);

  // driver callbacks
  void scanCB(const boost::shared_ptr<const LidarDataMessage>& msg);
  void processScan(const boost::shared_ptr<const LidarDataMessage>& msg);

  // ros topic callbacks
  void connectScanCB();
  void disconnectScanCB();
  void connectCloudCB();
  void disconnectCloudCB();

  multisense_driver::MultisenseDriver* driver_;
  ros::Publisher scan_pub_;
  std::string frame_id_;
  multisense_driver::MultisenseSubscriber<LidarDataMessage> scan_sub_;
  boost::shared_ptr<multisense_ros::CommandBase> command_;
  multisense_ros::FunctionCaller scan_caller_;

  multisense_ros::StatePublisher diagnostics_;

  // store in class for efficiency
  mutable sensor_msgs::LaserScan laser_msg_;

  // LCM stuff:
  lcm::LCM lcm_publish_ ;
  bot_core::planar_lidar_t msg_out_;

}; // class

}// namespace


#endif
