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

#include <algorithm>
#include <assert.h>

#include <atlas_gazebo_plugins_pre_fuerte/fourWheeled_diffdrive_plugin.h>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/PhysicsEngine.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("diffdrive_plugin", FourWheeledDiffDrivePlugin);

enum
{
  FRONT_RIGHT,
  FRONT_LEFT,
  REAR_RIGHT,
  REAR_LEFT,
};

// Constructor
FourWheeledDiffDrivePlugin::FourWheeledDiffDrivePlugin(Entity *parent) :
  Controller(parent)
{
 this->parent_ = dynamic_cast<Model*>(this->parent);
 // parent_ = dynamic_cast<Model*> (parent);

  if (!parent_)
    gzthrow("Differential_Position2d controller requires a Model as its parent");

  enableMotors = true;

  wheelSpeed[FRONT_RIGHT] = 0;
  wheelSpeed[FRONT_LEFT] = 0;
  wheelSpeed[REAR_RIGHT] = 0;
  wheelSpeed[REAR_LEFT] = 0;

  prevUpdateTime = Simulator::Instance()->GetSimTime();

  Param::Begin(&parameters);
  frontleftJointNameP = new ParamT<std::string> ("frontleftJoint", "", 1);
  frontrightJointNameP = new ParamT<std::string> ("frontrightJoint", "", 1);
  rearleftJointNameP = new ParamT<std::string> ("rearleftJoint", "", 1);
  rearrightJointNameP = new ParamT<std::string> ("rearrightJoint", "", 1);
  wheelSepP = new ParamT<float> ("wheelSeparation", 0.47, 1);
  wheelDiamP = new ParamT<float> ("wheelDiameter", 0.15, 1);
  torqueP = new ParamT<float> ("torque", 25.0, 1);
  robotNamespaceP = new ParamT<std::string> ("robotNamespace", "/", 0);
  topicNameP = new ParamT<std::string> ("topicName", "", 1);
  Param::End();

  x_ = 0;
  rot_ = 0;
  alive_ = true;
}

// Destructor
FourWheeledDiffDrivePlugin::~FourWheeledDiffDrivePlugin()
{
  delete frontleftJointNameP;
  delete frontrightJointNameP;
  delete rearleftJointNameP;
  delete rearrightJointNameP;
  delete wheelSepP;
  delete wheelDiamP;
  delete torqueP;
  delete robotNamespaceP;
  delete topicNameP;
  delete callback_queue_thread_;
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void FourWheeledDiffDrivePlugin::LoadChild(XMLConfigNode *node)
{
 
  pos_iface_ = dynamic_cast<libgazebo::PositionIface*> (GetIface("position"));

  // the defaults are from pioneer2dx
  wheelSepP->Load(node);
  wheelDiamP->Load(node);
  torqueP->Load(node);

  frontleftJointNameP->Load(node);
  frontrightJointNameP->Load(node);
  rearleftJointNameP->Load(node);
  rearrightJointNameP->Load(node);

  joints[FRONT_LEFT] = parent_->GetJoint(**frontleftJointNameP);
  joints[FRONT_RIGHT] = parent_->GetJoint(**frontrightJointNameP);
  joints[REAR_LEFT] = parent_->GetJoint(**rearleftJointNameP);
  joints[REAR_RIGHT] = parent_->GetJoint(**rearrightJointNameP);

  if (!joints[FRONT_LEFT])
    gzthrow("The controller couldn't get front left hinge joint");

  if (!joints[FRONT_RIGHT])
    gzthrow("The controller couldn't get front right hinge joint");
  
    if (!joints[REAR_LEFT])
    gzthrow("The controller couldn't get rear left hinge joint");

  if (!joints[REAR_RIGHT])
    gzthrow("The controller couldn't get rear right hinge joint");

  // Initialize the ROS node and subscribe to cmd_vel

  robotNamespaceP->Load(node);
  robotNamespace = robotNamespaceP->GetValue();

    int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "diffdrive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(robotNamespace);

  ROS_INFO("starting four_wheeled_diffdrive_plugin in ns: %s", this->robotNamespace.c_str());

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  topicNameP->Load(node);
  topicName = topicNameP->GetValue();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                          boost::bind(&FourWheeledDiffDrivePlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

}

// Initialize the controller
void FourWheeledDiffDrivePlugin::InitChild()
{
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  callback_queue_thread_ = new boost::thread(boost::bind(&FourWheeledDiffDrivePlugin::QueueThread, this));
}

// Load the controller
void FourWheeledDiffDrivePlugin::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(frontleftJointNameP) << "\n";
  stream << prefix << *(frontrightJointNameP) << "\n";
  stream << prefix << *(rearleftJointNameP) << "\n";
  stream << prefix << *(rearrightJointNameP) << "\n";
  stream << prefix << *(torqueP) << "\n";
  stream << prefix << *(wheelDiamP) << "\n";
  stream << prefix << *(wheelSepP) << "\n";
}

// Reset
void FourWheeledDiffDrivePlugin::ResetChild()
{
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;
}

// Update the controller
void FourWheeledDiffDrivePlugin::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time stepTime;

  //myIface->Lock(1);

  GetPositionCmd();

  wd = **(wheelDiamP);
  ws = **(wheelSepP);

  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = Simulator::Instance()->GetSimTime() - prevUpdateTime;
  prevUpdateTime = Simulator::Instance()->GetSimTime();

  // Distance travelled by front wheels
  d1 = stepTime.Double() * wd / 2 * joints[FRONT_LEFT]->GetVelocity(0);
  d2 = stepTime.Double() * wd / 2 * joints[FRONT_RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

  // Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  // Compute odometric instantaneous velocity
  odomVel[0] = dr / stepTime.Double();
  odomVel[1] = 0.0;
  odomVel[2] = da / stepTime.Double();

       
  if (enableMotors)
  {
    joints[FRONT_LEFT]->SetVelocity(0, wheelSpeed[FRONT_LEFT] / (**(wheelDiamP) / 2.0));
    joints[FRONT_RIGHT]->SetVelocity(0, wheelSpeed[FRONT_RIGHT] / (**(wheelDiamP) / 2.0));
    joints[REAR_LEFT]->SetVelocity(0, wheelSpeed[REAR_LEFT] / (**(wheelDiamP) / 2.0));
    joints[REAR_RIGHT]->SetVelocity(0, wheelSpeed[REAR_RIGHT] / (**(wheelDiamP) / 2.0));

    joints[FRONT_LEFT]->SetMaxForce(0, **(torqueP));
    joints[FRONT_RIGHT]->SetMaxForce(0, **(torqueP));
    joints[REAR_LEFT]->SetMaxForce(0, **(torqueP));
    joints[REAR_RIGHT]->SetMaxForce(0, **(torqueP));

  }

  write_position_data();
  publish_odometry();

  //myIface->Unlock();
}

// Finalize the controller
void FourWheeledDiffDrivePlugin::FiniChild()
{
  //std::cout << "ENTERING FINALIZE\n";
  alive_ = false;
  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_->join();
  //std::cout << "EXITING FINALIZE\n";
}

// NEW: Now uses the target velocities from the ROS message, not the Iface 
void FourWheeledDiffDrivePlugin::GetPositionCmd()
{
  lock.lock();

  double vr, va;

  vr = x_; //myIface->data->cmdVelocity.pos.x;
  va = rot_; //myIface->data->cmdVelocity.yaw;

  //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; //myIface->data->cmdEnableMotors > 0;

  //std::cout << enableMotors << std::endl;

  wheelSpeed[FRONT_LEFT] = vr + va * **(wheelSepP) / 2;
  wheelSpeed[FRONT_RIGHT] = vr - va * **(wheelSepP) / 2;
  wheelSpeed[REAR_LEFT] = vr + va * **(wheelSepP) / 2;
  wheelSpeed[REAR_RIGHT] = vr - va * **(wheelSepP) / 2;
  
  lock.unlock();
}

// NEW: Store the velocities from the ROS message
void FourWheeledDiffDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  //std::cout << "BEGIN CALLBACK\n";

  lock.lock();

  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;

  lock.unlock();

 // std::cout << "END CALLBACK\n";
}

// NEW: custom callback queue thread
void FourWheeledDiffDrivePlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// NEW: Update this to publish odometry topic
void FourWheeledDiffDrivePlugin::publish_odometry()
{
  // get current time
  ros::Time current_time_((Simulator::Instance()->GetSimTime()).sec, (Simulator::Instance()->GetSimTime()).nsec); 

  // getting data for base_footprint to odom transform
  btQuaternion qt;
  // TODO: Is there something wrong here? RVIZ has a problem?
  qt.setEulerZYX(pos_iface_->data->pose.yaw, pos_iface_->data->pose.pitch, pos_iface_->data->pose.roll);
  btVector3 vt(pos_iface_->data->pose.pos.x, pos_iface_->data->pose.pos.y, pos_iface_->data->pose.pos.z);
  tf::Transform base_footprint_to_odom(qt, vt);

  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                            current_time_,
                                                            "odom",
                                                            "base_footprint"));

  // publish odom topic
  odom_.pose.pose.position.x = pos_iface_->data->pose.pos.x;
  odom_.pose.pose.position.y = pos_iface_->data->pose.pos.y;

  gazebo::Quatern rot;
  rot.SetFromEuler(gazebo::Vector3(pos_iface_->data->pose.roll, pos_iface_->data->pose.pitch, pos_iface_->data->pose.yaw));

  odom_.pose.pose.orientation.x = rot.x;
  odom_.pose.pose.orientation.y = rot.y;
  odom_.pose.pose.orientation.z = rot.z;
  odom_.pose.pose.orientation.w = rot.u;

  odom_.twist.twist.linear.x = pos_iface_->data->velocity.pos.x;
  odom_.twist.twist.linear.y = pos_iface_->data->velocity.pos.y;
  odom_.twist.twist.angular.z = pos_iface_->data->velocity.yaw;

  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
  odom_.child_frame_id = "base_footprint";

  //odom_.header.stamp = current_time_;
  odom_.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  odom_.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  pub_.publish(odom_);
}

// Update the data in the interface
void FourWheeledDiffDrivePlugin::write_position_data()
{
  // TODO: Data timestamp
  pos_iface_->data->head.time = Simulator::Instance()->GetSimTime().Double();

  pos_iface_->data->pose.pos.x = odomPose[0];
  pos_iface_->data->pose.pos.y = odomPose[1];
  pos_iface_->data->pose.yaw = NORMALIZE(odomPose[2]);

  pos_iface_->data->velocity.pos.x = odomVel[0];
  pos_iface_->data->velocity.yaw = odomVel[2];

  // TODO
  pos_iface_->data->stall = 0;
}

