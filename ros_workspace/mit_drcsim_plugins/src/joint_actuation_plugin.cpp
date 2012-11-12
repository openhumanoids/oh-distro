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


#include <algorithm>
#include <assert.h>

#include <paladin_drcsim_plugins/joint_actuation_plugin.h>

#include <common/common.h>
#include <math/gzmath.h>
#include <physics/physics.h>
#include <sdf/sdf.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

namespace gazebo
{


// Constructor
JointActuationPlugin::JointActuationPlugin()
{
}

// Destructor
JointActuationPlugin::~JointActuationPlugin()
{
  // Finalize the controller
  alive_ = false;
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();
  
  delete rosnode_;
}

// Load the controller
void JointActuationPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  this->world = _parent->GetWorld();

  gzdbg << "plugin parent sensor name: " << parent->GetName() << "\n";

  if (!this->parent) { gzthrow("Joint Actuation controller requires a Model as its parent"); }

  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_WARN("Joint Actuation plugin missing <topicName>, defaults to ACTUATOR_CMDS");
    this->topicName = "ACTUATOR_CMDS";
  }
  else
  {
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
  }

  alive_ = true;
 
  ActuationCmdStruc zero_cmd;
  zero_cmd.value = 0;
  zero_cmd.start_time = 0;
  zero_cmd.duration = 0;
  zero_cmd.on_flag = false;
     
  for (unsigned int i = 0; i < this->parent->GetJointCount(); i ++)
  {
     joint = this->parent->GetJoint(i);
     std::string joint_name = joint->GetName();
     _joint_actuation_values.insert(make_pair(joint_name, zero_cmd));
  }
 

  // Initialize the ROS node and subscribe to joint actuation command topic
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "joint_actuation_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(this->robotNamespace);

  ROS_INFO("starting joint_actuation_plugin in ns: %s", this->robotNamespace.c_str());


  // ROS: Subscribe to the joint actuation command topic
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<atlas_gazebo_msgs::ActuatorCmd>(topicName, 1,
                                                          boost::bind(&JointActuationPlugin::actuationCmdCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);

  // start custom queue for diff drive
  this->callback_queue_thread_ = boost::thread(boost::bind(&JointActuationPlugin::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&JointActuationPlugin::UpdateChild, this));
}

// Update the controller
void JointActuationPlugin::UpdateChild()
{

  //update forces of all joints that are active

  // Loop through all Joints
  std::map<std::string, ActuationCmdStruc>::iterator it;

  for (unsigned int i = 0; i < this->parent->GetJointCount(); i ++)
  {
    std::string joint_name = this->parent->GetJoint(i)->GetName();


    it=_joint_actuation_values.find(joint_name);

    // both time_now, duration and start_time must also in simTime.  
    double time_now = this->world->GetSimTime().Double();
    lock.lock();	
    double duration = it->second.duration; // in seconds
    double start_time =  it->second.start_time; 
    bool on_flag = it->second.on_flag;
    lock.unlock();

    if(on_flag==true) // if flag is on
    {
      if (time_now <= start_time + duration)  
      { 
        this->parent->GetJoint(i)->SetForce(0,it->second.value);
      }
      else  // duration has expired, so clear forces. (duration is reset if a new torque command is received)
      { 
        //clear force
        this->parent->GetJoint(i)->SetForce(0,0);
        lock.lock();	
        it->second.on_flag = false;
        lock.unlock();	
      }
    }
  } // end for loop
      
}// end update child.



void JointActuationPlugin::actuationCmdCallback(const atlas_gazebo_msgs::ActuatorCmd::ConstPtr& cmd_msg)
{

  //Double check that the msg is for the correct robot name (this->parent->getName()). 
  if(cmd_msg->robot_name == this->parent->GetName())
  {
    lock.lock();
    int64_t t   =  cmd_msg->header.stamp.toNSec(); 
    double msg_start_time  = t*1e-9; // from nsec to sec 
    //double msg_start_time  = this->world->GetSimTime().Double(); // All timing is done relative to sim time.
    //for all joints in message update ActuationCmd Buffer.      
    for (std::vector<int>::size_type i = 0; i != cmd_msg->actuator_name.size(); i++)
    {
      std::string joint_name = cmd_msg->actuator_name[i];
      std::map<std::string, ActuationCmdStruc>::iterator it;
      it=_joint_actuation_values.find(joint_name);	
      ActuationCmdStruc act_cmd;
      act_cmd.start_time = msg_start_time; // sim time when msg was received.
      act_cmd.value = cmd_msg->actuator_effort[i];
      act_cmd.duration = cmd_msg->effort_duration[i];
      act_cmd.on_flag = true;
      it->second = act_cmd;
    }// end for
    lock.unlock();
  }
  
}

void JointActuationPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}


GZ_REGISTER_MODEL_PLUGIN(JointActuationPlugin)
}

