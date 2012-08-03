#include <ros/ros.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <urdf/model.h>
#include <urdf_interface/joint.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <atlas_gazebo_msgs/RobotState.h>

#include <lcm/lcm-cpp.hpp>
//#include <lcmtypes/bot_core.h>
#include <lcmtypes/drc_lcmtypes.hpp>

double getTime_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; 
}

void true_robot_state_Callback(const atlas_gazebo_msgs::RobotState::ConstPtr& msg)
{
drc::robot_state_t robot_state_msg;
robot_state_msg.timestamp = msg->header.stamp.toNSec()/1000; // from nsec to usec
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
robot_state_msg.angular_position.push_back(msg->joint_position[i]);
robot_state_msg.angular_velocity.push_back(msg->joint_velocity[i]);
robot_state_msg.measured_torque.push_back(0);
robot_state_msg.joint_cov.push_back(j_cov);
}

// dummy ground contact state
robot_state_msg.ground_contacts.num_contacts =2;
robot_state_msg.ground_contacts.id.push_back("Left_Foot");
robot_state_msg.ground_contacts.id.push_back("Right_Foot");
robot_state_msg.ground_contacts.onGround.push_back(0);
robot_state_msg.ground_contacts.onGround.push_back(0);

lcm::LCM lcm;
if(lcm.good())
 lcm.publish("TRUE_ROBOT_STATE", &robot_state_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "true_robot_state_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("true_robot_state", 10, true_robot_state_Callback);


  ros::spin();
  return 0;
}


// Code for Service based comms. Foudn to be too slow. but iwll be useful if you want to spawn models 
// programmatically
 /*ros::service::waitForService("/gazebo/get_joint_properties");
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

 std::string urdf_param_name = "robot_description";
 urdf::Model urdf_model;
  if (!urdf_model.initParam(urdf_param_name))
    ROS_ERROR("unable to get urdf xml from param %s",urdf_param_name.c_str());
  // get gazebo model from urdf model name
  ROS_INFO("urdf model name [%s]",urdf_model.getName().c_str());
std::cout<< urdf_model.getName() <<std::endl;
std::cout<< urdf_model.joints_.size()<<std::endl;


std::vector<std::string> joint_names_;
  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType;
  for(  joints_mapType::const_iterator it = urdf_model.joints_.begin(); it!=urdf_model.joints_.end(); it++)
  { 
	if(it->second->type!=6)
	{
                std::cout << it->first << std::endl;
                joint_names_.push_back(it->first);
	}
  }

  gazebo_msgs::GetJointProperties srv;


double tic, toc;
tic = getTime_now();
ros::WallTime start_time (ros::WallTime::now());
  for( std::vector<std::string>::const_iterator it =joint_names_.begin(); it!=joint_names_.end(); it++)
 {
    
    srv.request.joint_name = *it;
      //srv.request.joint_name = "BackRoll";
     std::cout << srv.request.joint_name << std::endl;
      if (client.call(srv))
      {

        for (unsigned int i=0; i<srv.response.position.size();i++)
         {
           //std::cout<< srv.response.position[i]<<std::endl;
           ROS_INFO("Position: %.16f",srv.response.position[i]);
   	   ROS_INFO("Rate: %.16f",srv.response.rate[i]);
         }
       }
       else
       {
          ROS_ERROR("Failed to call service get_joint_properties");
          return 1;
       }
  
   }
toc = getTime_now();
ros::WallDuration elapsed_time (ros::WallTime::now() - start_time);
ROS_INFO("elapsed: %.16f seconds", elapsed_time.toSec());
std::cout << (toc-tic)*0.001 << " msec elapsed."<< std::endl;
*/


