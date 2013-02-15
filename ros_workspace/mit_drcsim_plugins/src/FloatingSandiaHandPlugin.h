/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_FLOATING_SANDIA_HAND_PLUGIN_HH
#define GAZEBO_FLOATING_SANDIA_HAND_PLUGIN_HH

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <osrf_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  class FloatingSandiaHandPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: FloatingSandiaHandPlugin();

    /// \brief Destructor
    public: virtual ~FloatingSandiaHandPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();
    private: void CopyVectorIfValid(const std::vector<double> &from,
                                    std::vector<double> &to,
                                    const unsigned joint_offset);

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    /// Throttle update rate
    private: double lastStatusTime;
    private: double updateRate;

    // IMU sensor
    private: common::Time lastImuTime;
    private: std::string leftImuLinkName;
    private: physics::LinkPtr leftImuLink;
    private: math::Pose leftImuReferencePose;
    private: math::Vector3 leftImuLastLinearVel;
    private: ros::Publisher pubLeftImu;
    private: std::string rightImuLinkName;
    private: physics::LinkPtr rightImuLink;
    private: math::Pose rightImuReferencePose;
    private: math::Vector3 rightImuLastLinearVel;
    private: ros::Publisher pubRightImu;

    // deferred loading in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;
    private: ros::Publisher pubLeftJointStates;
    private: ros::Publisher pubRightJointStates;

    private: ros::Subscriber subJointCommands[2];
    private: void SetJointCommands(
      const osrf_msgs::JointCommands::ConstPtr &_msg,
      const unsigned jointOffset);  // to handle left/right hands

    private: std::vector<std::string> jointNames;
    private: physics::Joint_V joints;
    private: class ErrorTerms
      {
        double q_p;
        double d_q_p_dt;
        double q_i;
        double qd_p;
        friend class FloatingSandiaHandPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: osrf_msgs::JointCommands jointCommands;
    private: sensor_msgs::JointState leftJointStates;
    private: sensor_msgs::JointState rightJointStates;

    // Controls stuff
    private: common::Time lastControllerUpdateTime;
  };
/** \} */
/// @}
}
#endif
