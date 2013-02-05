#ifndef MULTISENSE_ROS_LED_H
#define MULTISENSE_ROS_LED_H
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_driver/multisense_driver.h>
#include <multisense_ros/tools.h>
#include <actionlib/server/simple_action_server.h>

#include <LibSensorPodCommunications/LedSetMessage.h>
#include <LibSensorPodCommunications/LedSetAckMessage.h>
#include <multisense_msgs/LedControlAction.h>

namespace multisense_ros
{

class Led
{
 public:
  Led(multisense_driver::MultisenseDriver* driver);

 private:
  //action callbacks
  void ledControlGoal();
  void ledControlResult(const boost::shared_ptr<const LedSetAckMessage>& res);

  multisense_driver::MultisenseDriver* driver_;
  boost::mutex as_mutex_;

  //led control
  typedef Command<LedSetMessage, LedSetAckMessage> LedCommand;
  boost::shared_ptr<LedCommand> led_commander_;
  boost::shared_ptr<const multisense_msgs::LedControlGoal> led_goal_;
  typedef actionlib::SimpleActionServer<multisense_msgs::LedControlAction> LedAS;
  boost::shared_ptr<LedAS> as_;
};

}

#endif
