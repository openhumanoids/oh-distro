/*
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


#include <algorithm>
#include <assert.h>

#include <paladin_drcsim_plugins/planar_hover_plugin.h>

#include <common/common.hh>
#include <math/gzmath.hh>
#include <physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

namespace gazebo
{

enum
{
  X,
  Y,
  THETA,
};

// Constructor
PlanarHoverPlugin::PlanarHoverPlugin()
{
}

// Destructor
PlanarHoverPlugin::~PlanarHoverPlugin()
{
  // Finalize the controller
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
  
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void PlanarHoverPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();

  gzdbg << "plugin parent sensor name: " << parent->GetName() << "\n";

  if (!this->parent) { gzthrow("Differential_Position2d controller requires a Model as its parent"); }

  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
  }

  if (!_sdf->HasElement("XJoint"))
  {
    ROS_WARN("Differential Drive plugin missing <XJoint>, defaults to X_Joint");
    this->XJointName = "X_Joint";  
  }
  else
  {
    this->XJointName = _sdf->GetElement("XJoint")->GetValueString();
  }  
  
    if (!_sdf->HasElement("YJoint"))
  {
    ROS_WARN("Differential Drive plugin missing <YJoint>, defaults to Y_Joint");
    this->YJointName = "Y_Joint";
  }
  else
  {
    this->YJointName = _sdf->GetElement("YJoint")->GetValueString();
  }  
  if (!_sdf->HasElement("ThetaJoint"))
  {
    ROS_WARN("Differential Drive plugin missing <ThetaJoint>, defaults to Theta_Joint");
    this->thetaJointName = "Theta_Joint";
  }
  else
  {
    this->thetaJointName = _sdf->GetElement("ThetaJoint")->GetValueString();
  }
  if (!_sdf->HasElement("MovingBase"))
  {
    ROS_WARN("Differential Drive plugin missing <MovingBase>, defaults to base");
    this->movingBaseName = "base";
  }
  else
  {
    this->movingBaseName = _sdf->GetElement("MovingBase")->GetValueString();
  }

  if (!_sdf->HasElement("torque"))
  {
    ROS_WARN("Differential Drive plugin missing <torque>, defaults to 5.0");
    this->torque = 5.0;
  }
  else
  {
    this->torque = _sdf->GetElement("torque")->GetValueDouble();
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_WARN("Differential Drive plugin missing <topicName>, defaults to cmd_vel");
    this->topicName = "cmd_vel";
  }
  else
  {
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
  }

  
  planarSpeed[X] = 0;
  planarSpeed[Y] = 0;
  planarSpeed[THETA] = 0;

  x_ = 0;
  rot_ = 0;
  alive_ = true;
  
  joints[X] = this->parent->GetJoint(XJointName); 
  if (!joints[X])  { gzthrow("The controller couldn't get  prismatic base joint"); }
  
  joints[Y] = this->parent->GetJoint(YJointName);
  if (!joints[Y])  { gzthrow("The controller couldn't get  prismatic base joint"); }

  joints[THETA] = this->parent->GetJoint(thetaJointName);
  if (!joints[THETA])  { gzthrow("The controller couldn't get rotable base joint"); }

 
  // Initialize the ROS node and subscribe to cmd_vel
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "planar_hover_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

  ROS_INFO("starting planar hover plugin in ns: %s", this->robotNamespace.c_str());

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                          boost::bind(&PlanarHoverPlugin::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  // Initialize the controller
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  // start custom queue for diff drive
  this->callback_queue_thread_ = boost::thread(boost::bind(&PlanarHoverPlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&PlanarHoverPlugin::UpdateChild, this));
}

// Update the controller
void PlanarHoverPlugin::UpdateChild()
{

  GetPositionCmd();


//  math::Vector3 v = this->parent->GetLink(MovableBaseLinkName)->GetRelativeAngularVel();
//   da = stepTime * v.z;
//movableBase->AddForce(math::Vector3(100, 0, 0)); // does not work?
// movableBase->SetAngularVel(math::Vector3(0, 0, 1));

  joints[X]->SetVelocity(0, planarSpeed[X]);
  joints[Y]->SetVelocity(0, planarSpeed[Y]);
  joints[THETA]->SetVelocity(0, planarSpeed[THETA]);


//this->parent->GetLink("base")->SetLinearVel(math::Vector3(planarSpeed[X], planarSpeed[Y], 0));
//this->parent->GetLink("base")->SetAngularVel(math::Vector3(0, 0, planarSpeed[THETA]));
   //  joints[X]->SetForce(0, 50 );
   //joints[Y]->SetForce(0, 50);
   //joints[THETA]->SetForce(0, 100);
  //this->parent->GetJoint("Theta_Joint")->SetForce(0, -10 );
   //this->parent->GetJoint("X_Joint")->SetForce(0, 100);
   // this->parent->GetJoint("Theta_Joint")->SetForce(0, 100);
  //this->parent->SetLinearVel(math::Vector3(.03, 0, 0));

  joints[X]->SetMaxForce(0, torque);
  joints[Y]->SetMaxForce(0, torque);
  joints[THETA]->SetMaxForce(0, torque);

}


void PlanarHoverPlugin::GetPositionCmd()
{
  lock.lock();

  //math::Pose pose = this->parent->GetState().GetPose();
  math::Pose pose = this->parent->GetLink(movingBaseName)->GetWorldPose();
  current_yaw =  pose.rot.GetYaw(); //global yaw
  //std::cout << "current_yaw: [" << current_yaw << "]" << std::endl; 
  planarSpeed[THETA] = rot_;
  planarSpeed[X] =  x_*cos(current_yaw);
  planarSpeed[Y] =  x_*sin(current_yaw);
  lock.unlock();
}

void PlanarHoverPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  lock.lock();

  x_ = 4*cmd_msg->linear.x;
  rot_ = 2*cmd_msg->angular.z;
 // std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;
 //  std::cout << "joints[Y]: [" << joints[Y]->GetVelocity(0) << "] joints[THETA]: [" << joints[THETA]->GetVelocity(0)  << "]" << std::endl;
  
  lock.unlock();
}

void PlanarHoverPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}


GZ_REGISTER_MODEL_PLUGIN(PlanarHoverPlugin)
}
