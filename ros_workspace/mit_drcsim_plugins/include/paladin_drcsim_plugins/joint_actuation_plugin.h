/*  Adapted from diffdrive gazebo plugin.
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef JOINT_ACTUATION_PLUGIN_H
#define JOINT_ACTUATION_PLUGIN_H

#include <map>

#include <common/common.hh>
#include <physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <atlas_gazebo_msgs/ActuatorCmd.h>

namespace gazebo
{
class Joint;
class Entity;

struct ActuationCmdStruc
{
  double value;
  double start_time;
  double duration;
  bool  on_flag;
};


class JointActuationPlugin: public ModelPlugin
{
  public: JointActuationPlugin();
  public: ~JointActuationPlugin();
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected: virtual void UpdateChild();
 
private:

  physics::WorldPtr world;
  physics::ModelPtr parent;

  event::ConnectionPtr updateConnection;

  std::map<std::string, ActuationCmdStruc> _joint_actuation_values; // Actuation cmd buffer

  physics::JointPtr joint;
  physics::PhysicsEnginePtr physicsEngine;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Subscriber sub_;


  boost::mutex lock;

  std::string robotNamespace;
  std::string topicName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  void actuationCmdCallback(const atlas_gazebo_msgs::ActuatorCmd::ConstPtr& cmd_msg);

  bool alive_;
};

}

#endif


