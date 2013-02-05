#ifndef MULTISENSE_ROS_LASER_JOINT_H
#define MULTISENSE_ROS_LASER_JOINT_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <multisense_driver/multisense_driver.h>
#include <multisense_ros/tools.h>
#include <LibSensorPodCommunications/LidarDataMessage.h>
#include <LibSensorPodCommunications/LidarSetMotorMessage.h>
#include <LibSensorPodCommunications/LidarSetMotorAckMessage.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <multisense_msgs/SpindleControlAction.h>
#include <multisense_ros/JointDiagnostics.h>


#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcm/lcm-cpp.hpp>

namespace multisense_ros
{

class LaserJoint
{
public:
  LaserJoint(multisense_driver::MultisenseDriver* driver);


private:
  // driver callbacks
  void scanCB(const boost::shared_ptr<const LidarDataMessage>& msg);

  // action callbacks
  void spindleControlGoal();
  void spindleControlResult(const boost::shared_ptr<const LidarSetMotorAckMessage>& res);

  multisense_driver::MultisenseDriver* driver_;
  boost::mutex as_mutex_;

  // joint state
  ros::Publisher js_pub_, diagnostics_pub_;
  multisense_driver::MultisenseSubscriber<LidarDataMessage> scan_sub_;
  sensor_msgs::JointState js_msg_;

  // splindle control
  typedef Command<LidarSetMotorMessage, LidarSetMotorAckMessage> SpindleCommand;
  boost::shared_ptr<SpindleCommand> spindle_commander_;
  boost::shared_ptr<const multisense_msgs::SpindleControlGoal> spindle_goal_;
  typedef actionlib::SimpleActionServer<multisense_msgs::SpindleControlAction> SpindleAS;
  typedef actionlib::SimpleActionClient<multisense_msgs::SpindleControlAction> SpindleAC;
  boost::shared_ptr<SpindleAS> as_;
  JointDiagnostics diagnostics_;


  // LCM stuff:
  lcm::LCM lcm_publish_ ;
  drc::robot_state_t msg_out_;

}; // class

}// namespace


#endif
