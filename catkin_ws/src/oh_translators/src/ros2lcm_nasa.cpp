// Copyright 2015 Maurice Fallon, Vladimir Ivan

// Selective ros2lcm translator for NASA

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <val_hardware_msgs/valImuSensor.h>
#include <val_hardware_msgs/valAtiSensor.h>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc/plan_status_t.hpp"

struct Joints
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<std::string> name;
};

class App
{
public:
  App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;
  bool verbose_;

  void appendSensors(bot_core::six_axis_force_torque_array_t& msg_out, geometry_msgs::WrenchStamped l_foot_sensor,
                         geometry_msgs::WrenchStamped r_foot_sensor, geometry_msgs::WrenchStamped l_hand_sensor, geometry_msgs::WrenchStamped r_hand_sensor);

  geometry_msgs::WrenchStamped lastLeftFootSensorMsg_;
  geometry_msgs::WrenchStamped lastRightFootSensorMsg_;
  geometry_msgs::WrenchStamped lastLeftHandSensorMsg_;
  geometry_msgs::WrenchStamped lastRightHandSensorMsg_;

  // NASA Data
  ros::Subscriber jointCommandsNasaSub_, jointStatesNasaSub_, imuSensorNasaSub_, footSensorNasaSub_;
  void jointCommandsNasaCallback(const sensor_msgs::JointStateConstPtr& msg);
  void jointStatesNasaCallback(const sensor_msgs::JointStateConstPtr& msg);
  void imuSensorNasaCallback(const val_hardware_msgs::valImuSensorConstPtr& msg);
  void footSensorNasaCallback(const val_hardware_msgs::valAtiSensorConstPtr& msg);

};

App::App(ros::NodeHandle node_in) :
    node_(node_in)
{
  ROS_INFO("Initializing Translator");
  if (!lcmPublish_.good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  verbose_ = false;


  // setting a queue of 1 reduces clumping of messages [desired for state estimation] but significantly reduces frequency
  // this is true even for just 2 subscriptions
  // setting to 100 means no messages are dropped during translation but imu data tends to be clumped into 50 message blocks
  int queue_size = 100;

  jointStatesNasaSub_ = node_.subscribe(std::string("/joint_states"), queue_size,
                                    &App::jointStatesNasaCallback, this);
  jointCommandsNasaSub_ = node_.subscribe(std::string("/joint_commands"), queue_size,
                                    &App::jointCommandsNasaCallback, this);
  imuSensorNasaSub_ = node_.subscribe(std::string("/imu_states"), queue_size,
                                    &App::imuSensorNasaCallback, this);
  footSensorNasaSub_ = node_.subscribe(std::string("/force_torque_states"), queue_size,
                                    &App::footSensorNasaCallback, this);
}

App::~App()
{
}



void App::appendSensors(bot_core::six_axis_force_torque_array_t& msg_out, geometry_msgs::WrenchStamped l_foot_sensor,
                            geometry_msgs::WrenchStamped r_foot_sensor, geometry_msgs::WrenchStamped l_hand_sensor, geometry_msgs::WrenchStamped r_hand_sensor)
{
  int num_sensors = 4;

  msg_out.utime = (int64_t)l_foot_sensor.header.stamp.toNSec() / 1000; 
  msg_out.num_sensors = num_sensors;

  std::vector<std::string> names;
  names.push_back("l_foot");
  names.push_back("r_foot");
  names.push_back("l_hand");
  names.push_back("r_hand");

  msg_out.names = names;

  std::vector<bot_core::six_axis_force_torque_t> sensors;
  bot_core::six_axis_force_torque_t l_foot;
  bot_core::six_axis_force_torque_t r_foot;
  bot_core::six_axis_force_torque_t l_hand; 
  bot_core::six_axis_force_torque_t r_hand;
  
  l_foot.utime = (int64_t)l_foot_sensor.header.stamp.toNSec() / 1000;
  l_foot.force[0] = l_foot_sensor.wrench.force.x;
  l_foot.force[1] = l_foot_sensor.wrench.force.y;
  l_foot.force[2] = l_foot_sensor.wrench.force.z;
  l_foot.moment[0] = l_foot_sensor.wrench.torque.x;
  l_foot.moment[1] = l_foot_sensor.wrench.torque.y;
  l_foot.moment[2] = l_foot_sensor.wrench.torque.z;

  r_foot.utime = (int64_t)r_foot_sensor.header.stamp.toNSec() / 1000;
  r_foot.force[0] = r_foot_sensor.wrench.force.x;
  r_foot.force[1] = r_foot_sensor.wrench.force.y;
  r_foot.force[2] = r_foot_sensor.wrench.force.z;
  r_foot.moment[0] = r_foot_sensor.wrench.torque.x;
  r_foot.moment[1] = r_foot_sensor.wrench.torque.y;
  r_foot.moment[2] = r_foot_sensor.wrench.torque.z;

  l_hand.utime = (int64_t)l_hand_sensor.header.stamp.toNSec() / 1000;
  l_hand.force[0] = l_hand_sensor.wrench.force.x;
  l_hand.force[1] = l_hand_sensor.wrench.force.y;
  l_hand.force[2] = l_hand_sensor.wrench.force.z;
  l_hand.moment[0] = l_hand_sensor.wrench.torque.x;
  l_hand.moment[1] = l_hand_sensor.wrench.torque.y;
  l_hand.moment[2] = l_hand_sensor.wrench.torque.z;

  r_hand.utime = (int64_t)r_hand_sensor.header.stamp.toNSec() / 1000;
  r_hand.force[0] = r_hand_sensor.wrench.force.x;
  r_hand.force[1] = r_hand_sensor.wrench.force.y;
  r_hand.force[2] = r_hand_sensor.wrench.force.z;
  r_hand.moment[0] = r_hand_sensor.wrench.torque.x;
  r_hand.moment[1] = r_hand_sensor.wrench.torque.y;
  r_hand.moment[2] = r_hand_sensor.wrench.torque.z;
 
  sensors.push_back(l_foot);
  sensors.push_back(r_foot);
  sensors.push_back(l_hand);
  sensors.push_back(r_hand);

  msg_out.sensors = sensors;
}


////////////////////// NASA Originating Data ////////////////////////
void App::jointStatesNasaCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    bot_core::joint_state_t amsg;
    amsg.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec

    for (int i = 0; i < msg->position.size(); i++)
    {
        amsg.joint_name.push_back(msg->name[i]);
        amsg.joint_position.push_back(msg->position[i]);
        amsg.joint_velocity.push_back(msg->velocity[i]);
        amsg.joint_effort.push_back(msg->effort[i]);
    }
    amsg.num_joints = amsg.joint_name.size();
    lcmPublish_.publish("VAL_CORE_ROBOT_STATE", &amsg);
}

void App::jointCommandsNasaCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    bot_core::joint_state_t amsg;
    amsg.utime = (int64_t)msg->header.stamp.toNSec() / 1000;  // from nsec to usec

    for (int i = 0; i < msg->position.size(); i++)
    {
        amsg.joint_name.push_back(msg->name[i]);
        amsg.joint_position.push_back(msg->position[i]);
        amsg.joint_velocity.push_back(msg->effort[i]);
        amsg.joint_effort.push_back(msg->effort[i]);
    }
    amsg.num_joints = amsg.joint_name.size();
    lcmPublish_.publish("VAL_COMMAND_FEEDBACK", &amsg);
}

void App::imuSensorNasaCallback(const val_hardware_msgs::valImuSensorConstPtr& msg)
{

  for (int i=0; i < msg->name.size(); i++){
    bot_core::ins_t imu;
    imu.utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
    imu.device_time = imu.utime;
    imu.gyro[0] = msg->angularVelocity[i].x;
    imu.gyro[1] = msg->angularVelocity[i].y;
    imu.gyro[2] = msg->angularVelocity[i].z;
    imu.mag[0] = 0;
    imu.mag[1] = 0;
    imu.mag[2] = 0;
    imu.accel[0] = msg->linearAcceleration[i].x;
    imu.accel[1] = msg->linearAcceleration[i].y;
    imu.accel[2] = msg->linearAcceleration[i].z;
    imu.quat[0] = msg->orientation[i].w;
    imu.quat[1] = msg->orientation[i].x;
    imu.quat[2] = msg->orientation[i].y;
    imu.quat[3] = msg->orientation[i].z;
    imu.pressure = 0;
    imu.rel_alt = 0;

    std::string output_channel = "VAL_IMU_" + msg->name[i];
    lcmPublish_.publish(output_channel, &imu);
  }

}

void App::footSensorNasaCallback(const val_hardware_msgs::valAtiSensorConstPtr& msg)
{

  lastLeftFootSensorMsg_.wrench = (msg->forceTorque[0]);
  lastLeftFootSensorMsg_.header = msg->header;
  lastRightFootSensorMsg_.wrench = (msg->forceTorque[1]);
  lastRightFootSensorMsg_.header = msg->header;

  bot_core::six_axis_force_torque_array_t six_axis_force_torque_array;
  appendSensors(six_axis_force_torque_array, lastLeftFootSensorMsg_, lastRightFootSensorMsg_, lastLeftHandSensorMsg_, lastRightHandSensorMsg_);
  lcmPublish_.publish("VAL_FORCE_TORQUE", &six_axis_force_torque_array);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros2lcm_nasa");
  ros::NodeHandle nh;
  new App(nh);
  ROS_ERROR("ROS2LCM NASA Translator Ready");
  ros::spin();
  return 0;
}
