#ifndef MULTISENSE_ROS_STATE_PUBLISHER_H
#define MULTISENSE_ROS_STATE_PUBLISHER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <multisense_ros/SensorDiagnostics.h>
#include <multisense_ros/SensorStatus.h>
#include <ros/ros.h>

namespace multisense_ros
{
class StatePublisher
{
public:
  StatePublisher(ros::NodeHandle nh, const std::string& topic, float expected_rate);

  void countStream();
  void publish(const uint8_t state);
  void setExpectedRate(float expected_rate);

private:
  void publishScanRate(const ros::TimerEvent& t);
  void setCounter(int expected_rate);

  multisense_ros::SensorDiagnostics diagnostics_;
  ros::Publisher diagnostics_pub_;

  boost::mutex mutex_;
  std::vector<int> scan_counter_;
  int scan_counter_index_;
  float expected_rate_;
  ros::Timer timer_;
};

}

#endif
