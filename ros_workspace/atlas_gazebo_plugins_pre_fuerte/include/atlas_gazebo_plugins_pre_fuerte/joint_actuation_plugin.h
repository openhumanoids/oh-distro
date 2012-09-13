/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
#ifndef JOINT_ACTUATION_PLUGIN_H
#define JOINT_ACTUATION_PLUGIN_H

#include <map>

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

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

class JointActuationPlugin : public Controller
{

public:
  JointActuationPlugin(Entity *parent);
  virtual ~JointActuationPlugin();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  void SaveChild(std::string &prefix, std::ostream &stream);
  virtual void InitChild();
  void ResetChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  libgazebo::PositionIface *pos_iface_;
  Model *parent_;

  // Simulation time of the last update
  Time prevUpdateTime;

  std::map<std::string, ActuationCmdStruc> _joint_actuation_values; // Actuation cmd buffer

  Joint *joint;
  PhysicsEngine *physicsEngine;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Subscriber sub_;


  boost::mutex lock;

  ParamT<std::string> *robotNamespaceP;
  std::string robotNamespace;

  ParamT<std::string> *topicNameP;
  std::string topicName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread* callback_queue_thread_;
  void QueueThread();

  void actuationCmdCallback(const atlas_gazebo_msgs::ActuatorCmd::ConstPtr& cmd_msg);

  bool alive_;
};

}

#endif


