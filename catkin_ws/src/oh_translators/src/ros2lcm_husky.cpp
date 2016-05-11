// ### Boost
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

// ### ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

// ### Standard includes
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <string>

#include <lcmtypes/bot_core.hpp>
#include <lcm/lcm-cpp.hpp>


class App
{
public:
  explicit App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

  ros::Subscriber sick_lidar_sub_;
  void sick_lidar_cb(const sensor_msgs::LaserScanConstPtr& msg);
  void publishLidar(const sensor_msgs::LaserScanConstPtr& msg, std::string channel);

  ros::Subscriber ekf_odom_sub_;
  void ekf_odom_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  ros::Subscriber imuSensorSub_;
  void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);

};

App::App(ros::NodeHandle node_)
{
  ROS_INFO("Initializing Sick Translator");
  if (!lcmPublish_.good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  sick_lidar_sub_ = node_.subscribe(std::string("/sick_scan"), 100, &App::sick_lidar_cb,
                                      this);
  ekf_odom_sub_ = node_.subscribe(std::string("/robot_pose_ekf/odom_combined"), 100, &App::ekf_odom_cb,
                                    this);

  imuSensorSub_ = node_.subscribe(std::string("/imu/data"), 100,
                                    &App::imuSensorCallback, this);
}

App::~App()
{
}



void App::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg)
{
  bot_core::ins_t imu;
  imu.utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
  imu.device_time = imu.utime;
  imu.gyro[0] = msg->angular_velocity.x;
  imu.gyro[1] = msg->angular_velocity.y;
  imu.gyro[2] = msg->angular_velocity.z;
  imu.mag[0] = 0;
  imu.mag[1] = 0;
  imu.mag[2] = 0;
  imu.accel[0] = msg->linear_acceleration.x;
  imu.accel[1] = msg->linear_acceleration.y;
  imu.accel[2] = msg->linear_acceleration.z;
  imu.quat[0] = msg->orientation.w;
  imu.quat[1] = msg->orientation.x;
  imu.quat[2] = msg->orientation.y;
  imu.quat[3] = msg->orientation.z;
  imu.pressure = 0;
  imu.rel_alt = 0;

  lcmPublish_.publish( "IMU_MICROSTRAIN", &imu);
}


void App::ekf_odom_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  ROS_ERROR_STREAM("ekfcp");
/*
    position: 
      x: -0.473553686959
      y: 59.9936536045
      z: 0.0
    orientation: 
      x: 0.0138799797993
      y: -0.014872795039
      z: 0.607424085735
      w: 0.794117199283
*/
  bot_core::pose_t lcm_pose_msg;
  lcm_pose_msg.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
  lcm_pose_msg.pos[0] = msg->pose.pose.position.x;
  lcm_pose_msg.pos[1] = msg->pose.pose.position.y;
  lcm_pose_msg.pos[2] = msg->pose.pose.position.z;
  lcm_pose_msg.orientation[0] = msg->pose.pose.orientation.w;
  lcm_pose_msg.orientation[1] = msg->pose.pose.orientation.x;
  lcm_pose_msg.orientation[2] = msg->pose.pose.orientation.y;
  lcm_pose_msg.orientation[3] = msg->pose.pose.orientation.z;
  lcmPublish_.publish("POSE_BODY", &lcm_pose_msg);

  if (1==1){ // publish EST_ROBOT_STATE (used to draw the robot)
    bot_core::robot_state_t est_robot_state_;
    est_robot_state_.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec
    est_robot_state_.num_joints = 4;
    est_robot_state_.pose.translation.x = msg->pose.pose.position.x;
    est_robot_state_.pose.translation.y = msg->pose.pose.position.y;
    est_robot_state_.pose.translation.z = msg->pose.pose.position.z;
    est_robot_state_.pose.rotation.w = msg->pose.pose.orientation.w;
    est_robot_state_.pose.rotation.x = msg->pose.pose.orientation.x;
    est_robot_state_.pose.rotation.y = msg->pose.pose.orientation.y;
    est_robot_state_.pose.rotation.z = msg->pose.pose.orientation.z;
    est_robot_state_.twist.linear_velocity.x = 0.0;
    est_robot_state_.twist.linear_velocity.y = 0.0;
    est_robot_state_.twist.linear_velocity.z = 0.0;
    est_robot_state_.twist.angular_velocity.x = 0.0;
    est_robot_state_.twist.angular_velocity.y = 0.0;
    est_robot_state_.twist.angular_velocity.z = 0.0;


    est_robot_state_.joint_name.assign(est_robot_state_.num_joints, "");
    est_robot_state_.joint_position.assign(est_robot_state_.num_joints, (const float &) 0.);
    est_robot_state_.joint_velocity.assign(est_robot_state_.num_joints, (const float &) 0.);
    est_robot_state_.joint_effort.assign(est_robot_state_.num_joints, (const float &) 0.);
    est_robot_state_.joint_name[0] = "front_left_wheel";
    est_robot_state_.joint_name[1] = "front_right_wheel";
    est_robot_state_.joint_name[2] = "rear_left_wheel";
    est_robot_state_.joint_name[3] = "rear_right_wheel";

    // Initialise F/T sensors in est_robot_state_
    est_robot_state_.force_torque.l_foot_force_z = 0.0;
    est_robot_state_.force_torque.l_foot_torque_x = 0.0;
    est_robot_state_.force_torque.l_foot_torque_y = 0.0;
    est_robot_state_.force_torque.r_foot_force_z = 0.0;
    est_robot_state_.force_torque.r_foot_torque_x = 0.0;
    est_robot_state_.force_torque.r_foot_torque_y = 0.0;

    for (unsigned int i = 0; i < 3; i++) {
        est_robot_state_.force_torque.l_hand_force[i] = 0.0;
        est_robot_state_.force_torque.l_hand_torque[i] = 0.0;
        est_robot_state_.force_torque.r_hand_force[i] = 0.0;
        est_robot_state_.force_torque.r_hand_torque[i] = 0.0;
    }

    lcmPublish_.publish("EST_ROBOT_STATE", &est_robot_state_);
  }

}


void App::sick_lidar_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
  publishLidar(msg, "SICK_SCAN");
}

void App::publishLidar(const sensor_msgs::LaserScanConstPtr& msg, std::string channel)
{
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
  scan_out.nranges = msg->ranges.size();
  scan_out.nintensities = msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcmPublish_.publish(channel.c_str(), &scan_out);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm_husky");
  ros::NodeHandle nh;
  new App(nh);
  ROS_INFO_STREAM("ros2lcm_husky translator ready");
  ROS_ERROR_STREAM("ros2lcm_husky translator ready");
  ros::spin();
  return 0;
}
