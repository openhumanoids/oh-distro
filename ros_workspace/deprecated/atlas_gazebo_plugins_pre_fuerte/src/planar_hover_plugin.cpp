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

#include <atlas_gazebo_plugins_pre_fuerte/planar_hover_plugin.h>

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

GZ_REGISTER_DYNAMIC_CONTROLLER("planar_hover_plugin", PlanarHoverPlugin);

enum
{
  X,
  Y,
  THETA,
};

// Constructor
PlanarHoverPlugin::PlanarHoverPlugin(Entity *parent) :
  Controller(parent)
{
 this->parent_ = dynamic_cast<Model*>(this->parent);
 // parent_ = dynamic_cast<Model*> (parent);

  if (!parent_)
    gzthrow("Differential_Position2d controller requires a Model as its parent");

  enableMotors = true;

  planarSpeed[X] = 0;
  planarSpeed[Y] = 0;
  planarSpeed[THETA] = 0;

  prevUpdateTime = Simulator::Instance()->GetSimTime();

  Param::Begin(&parameters);

  XJointNameP = new ParamT<std::string> ("XJoint", "", 1);
  YJointNameP = new ParamT<std::string> ("YJoint", "", 1);
  thetaJointNameP = new ParamT<std::string> ("ThetaJoint", "", 1);
  movingBaseNameP = new ParamT<std::string> ("MovingBase", "", 1);
  torqueP = new ParamT<float> ("torque", 25.0, 1);
  robotNamespaceP = new ParamT<std::string> ("robotNamespace", "/", 0);
  topicNameP = new ParamT<std::string> ("topicName", "", 1);
  Param::End();

  x_ = 0;
  rot_ = 0;
  alive_ = true;
}

// Destructor
PlanarHoverPlugin::~PlanarHoverPlugin()
{
  delete XJointNameP;
  delete YJointNameP;
  delete thetaJointNameP;
  delete movingBaseNameP;
  delete torqueP;
  delete robotNamespaceP;
  delete topicNameP;
  delete callback_queue_thread_;
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void PlanarHoverPlugin::LoadChild(XMLConfigNode *node)
{
 
  pos_iface_ = dynamic_cast<libgazebo::PositionIface*> (GetIface("position"));


  torqueP->Load(node);

  XJointNameP->Load(node);
  YJointNameP->Load(node);
  thetaJointNameP->Load(node);
  movingBaseNameP->Load(node);

  joints[X] = parent_->GetJoint(**XJointNameP); 
  joints[Y] = parent_->GetJoint(**YJointNameP); 
  joints[THETA] = parent_->GetJoint(**thetaJointNameP);
  movingBase =  parent_->GetBody(**movingBaseNameP);
  if (!joints[X])  { gzthrow("The controller couldn't get  prismatic base joint"); }
  if (!joints[Y])  { gzthrow("The controller couldn't get  prismatic base joint"); }
  if (!joints[THETA])  { gzthrow("The controller couldn't get rotable base joint"); }
  if (!movingBase)  { gzthrow("The controller couldn't get moving Base Link"); }

  // Initialize the ROS node and subscribe to cmd_vel

  robotNamespaceP->Load(node);
  robotNamespace = robotNamespaceP->GetValue();

    int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "planar_hover_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(robotNamespace);

  ROS_INFO("starting planar hover plugin in ns: %s", this->robotNamespace.c_str());

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  topicNameP->Load(node);
  topicName = topicNameP->GetValue();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                          boost::bind(&PlanarHoverPlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

}

// Initialize the controller
void PlanarHoverPlugin::InitChild()
{
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  callback_queue_thread_ = new boost::thread(boost::bind(&PlanarHoverPlugin::QueueThread, this));
}

// Load the controller
void PlanarHoverPlugin::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(XJointNameP) << "\n";
  stream << prefix << *(YJointNameP) << "\n";
  stream << prefix << *(thetaJointNameP) << "\n";
  stream << prefix << *(torqueP) << "\n";

}

// Reset
void PlanarHoverPlugin::ResetChild()
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
void PlanarHoverPlugin::UpdateChild()
{

  Time stepTime;

  //myIface->Lock(1);

  GetPositionCmd();


  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = Simulator::Instance()->GetSimTime() - prevUpdateTime;
  prevUpdateTime = Simulator::Instance()->GetSimTime();

       
  if (enableMotors)
  {

   joints[X]->SetVelocity(0, planarSpeed[X]);
   joints[Y]->SetVelocity(0, planarSpeed[Y]);
   joints[THETA]->SetVelocity(0, planarSpeed[THETA]);

    joints[X]->SetMaxForce(0, **(torqueP));
    joints[Y]->SetMaxForce(0, **(torqueP));
    joints[THETA]->SetMaxForce(0, **(torqueP));
  }

  //myIface->Unlock();
}

// Finalize the controller
void PlanarHoverPlugin::FiniChild()
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
void PlanarHoverPlugin::GetPositionCmd()
{
  lock.lock();

   //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;
   //std::cout <<(**movingBaseNameP) << std::endl;
  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; //myIface->data->cmdEnableMotors > 0;
  //Pose3d pose = parent_->GetWorldPose();
  Pose3d pose = movingBase->GetModelRelativePose();
  current_yaw =  pose.rot.GetYaw(); //global yaw
 // current_yaw =  pos_iface_->data->pose.yaw; //global yaw
  //std::cout << "current_yaw: [" << current_yaw << "]" << std::endl; 
  planarSpeed[THETA] = rot_;
  planarSpeed[X] =  x_*cos(current_yaw);
  planarSpeed[Y] =  x_*sin(current_yaw);
  
  lock.unlock();
}

// NEW: Store the velocities from the ROS message
void PlanarHoverPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  //std::cout << "BEGIN CALLBACK\n";

  lock.lock();

  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;

  lock.unlock();

 // std::cout << "END CALLBACK\n";
}

// NEW: custom callback queue thread
void PlanarHoverPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// NEW: Update this to publish odometry topic
void PlanarHoverPlugin::publish_odometry()
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
void PlanarHoverPlugin::write_position_data()
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

