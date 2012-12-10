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
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef GAZEBO_DRC_ROBOT_PLUGIN_HH
#define GAZEBO_DRC_ROBOT_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>

#include "math/Vector3.hh"
#include "physics/physics.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include "boost/thread/mutex.hpp"

namespace gazebo
{
  class DRCRobotPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: DRCRobotPlugin();

    /// \brief Destructor
    public: virtual ~DRCRobotPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void UpdateStates();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    private: boost::mutex update_mutex;

    /// Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// Sets DRC Robot feet placement
    /// No reachability checking here.
    public: void SetFeetPose(math::Pose _lPose, math::Pose _rPose);

    /// Sets DRC Robot planar navigational command velocity
    /// _cmd is a Vector3, where:
    ///   - x is the desired forward linear velocity, positive is robot-forward
    ///     and negative is robot-back.
    ///   - y is the desired lateral linear velocity, positive is robot-left
    ///     and negative is robot-right.
    ///   - z is the desired heading angular velocity, positive makes
    ///     the robot turn left, and negative makes the robot turn right
    public: void SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd);
    public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);
    public: void SetPluginModeTopic(const std_msgs::String::ConstPtr &_str);
    public: void SetPluginMode(const std::string &_str);

    /// Move the robot's pinned joint to a certain location in the world.
    public: void WarpDRCRobot(math::Pose _pose);

    /// \brief add a constraint between 2 links
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower);

    /// \brief Remove a joint
    private: void RemoveJoint(physics::JointPtr &_joint);

    // \brief attach a model's link to the gripper with relative pose
    private: void GrabLink(std::string _modelName, std::string _linkName,
                           std::string _gripperName, math::Pose _pose);

    private: physics::LinkPtr fixedLink;
    private: physics::JointPtr fixedJoint;
    private: physics::JointPtr grabJoint;
    private: math::Vector3 anchorPose;
    private: bool warpRobot;

    /// \brief keep initial pose of robot to prevent z-drifting when
    /// teleporting the robot.
    private: math::Pose initialPose;

    private: bool harnessed;

    private: double lastUpdateTime;
    private: geometry_msgs::Twist cmdVel;

    // ros stuff
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;
    private: ros::Subscriber trajectory_sub_;
    private: ros::Subscriber pose_sub_;
    private: ros::Subscriber mode_sub_;
  };
/** \} */
/// @}
}
#endif
