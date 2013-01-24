#include <ros/ros.h>
/*#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/ApplyJointEffort.h>*/
#include <urdf/model.h>
#include <urdf_interface/joint.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <atlas_gazebo_msgs/RobotState.h>

#include <rosgraph_msgs/Clock.h>

#include <lcm/lcm-cpp.hpp>
//#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <ros/callback_queue.h>

//double getTime_now()
//{
//    struct timeval tv;
//    gettimeofday (&tv, NULL);
//    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; 
//}

void clock_cb(const rosgraph_msgs::ClockConstPtr& msg){
  drc::utime_t utime_msg;
  utime_msg.utime = (int64_t) floor(msg->clock.toNSec()/1000);
  lcm::LCM lcm;
  if(lcm.good())
    lcm.publish("ROBOT_UTIME", &utime_msg);
  
}

void true_robot_state_Callback(const atlas_gazebo_msgs::RobotState::ConstPtr& msg)
{
drc::robot_state_t robot_state_msg;
//std::cout << msg->header.stamp.toNSec() << std::endl;
robot_state_msg.utime = msg->header.stamp.toNSec()/1000; //  from nsec to usec
robot_state_msg.robot_name = msg->robot_name;
robot_state_msg.origin_position.translation.x = msg->body_pose.position.x;
robot_state_msg.origin_position.translation.y = msg->body_pose.position.y;
robot_state_msg.origin_position.translation.z = msg->body_pose.position.z;
robot_state_msg.origin_position.rotation.x = msg->body_pose.orientation.x;
robot_state_msg.origin_position.rotation.y = msg->body_pose.orientation.y;
robot_state_msg.origin_position.rotation.z = msg->body_pose.orientation.z;
robot_state_msg.origin_position.rotation.w = msg->body_pose.orientation.w;

robot_state_msg.origin_twist.linear_velocity.x =msg->body_twist.linear.x;
robot_state_msg.origin_twist.linear_velocity.y =msg->body_twist.linear.y;
robot_state_msg.origin_twist.linear_velocity.z =msg->body_twist.linear.z;
robot_state_msg.origin_twist.angular_velocity.x =msg->body_twist.angular.x;
robot_state_msg.origin_twist.angular_velocity.y =msg->body_twist.angular.y;
robot_state_msg.origin_twist.angular_velocity.z =msg->body_twist.angular.z;

int i,j;
for(i = 0; i < 6; i++)  {
   for(j = 0; j < 6; j++) {
         robot_state_msg.origin_cov.position_cov[i][j] = 0;
	 robot_state_msg.origin_cov.twist_cov[i][j] = 0;
}
}

robot_state_msg.num_joints = msg->joint_name.size();
robot_state_msg.joint_name = msg->joint_name;

drc::joint_covariance_t j_cov;
j_cov.variance = 0;

for (std::vector<int>::size_type i = 0; i != robot_state_msg.joint_name.size(); i++)  {
robot_state_msg.joint_position.push_back(msg->joint_position[i]);
robot_state_msg.joint_velocity.push_back(msg->joint_velocity[i]);
robot_state_msg.measured_effort.push_back(0);
robot_state_msg.joint_cov.push_back(j_cov);
}


// dummy ground contact states
robot_state_msg.contacts.num_contacts =8;
robot_state_msg.contacts.id.push_back("LFootToeIn");
robot_state_msg.contacts.id.push_back("LFootToeOut");
robot_state_msg.contacts.id.push_back("LFootHeelIn");
robot_state_msg.contacts.id.push_back("LFootHeelOut");
robot_state_msg.contacts.id.push_back("RFootToeIn");
robot_state_msg.contacts.id.push_back("RFootToeOut");
robot_state_msg.contacts.id.push_back("RFootHeelIn");
robot_state_msg.contacts.id.push_back("RFootHeelOut");
for (int i=0; i< robot_state_msg.contacts.num_contacts; i++){
    robot_state_msg.contacts.inContact.push_back(0);
    drc::vector_3d_t f_zero;
    f_zero.x = 0;f_zero.y = 0;f_zero.z = 0;
    robot_state_msg.contacts.contactForce.push_back(f_zero);
}

lcm::LCM lcm;
if(lcm.good())
 lcm.publish("TRUE_ROBOT_STATE", &robot_state_msg);
}

int main(int argc, char **argv)
{
  ros::CallbackQueue local_callback_queue;
  ros::init(argc, argv, "true_robot_state_publisher");
  ros::NodeHandle n;
  n.setCallbackQueue(&local_callback_queue);
 //ros::Subscriber sub = n.subscribe("true_robot_state", 1000, true_robot_state_Callback);
  ros::Subscriber sub = n.subscribe("true_robot_state", 1000, true_robot_state_Callback,ros::TransportHints().unreliable().maxDatagramSize(1000).tcpNoDelay());
  // create subscriptions to contact sensors.

  ros::Subscriber sub2 = n.subscribe("/clock", 1000, clock_cb);
  
    //ros::spin();
   while (ros::ok())
   {
    local_callback_queue.callAvailable(ros::WallDuration(0.01));
    //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001)); // dont use global call back queue; spawn a local thread
   }
  return 0;
}


