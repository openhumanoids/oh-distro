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

#include <atlas_gazebo_plugins_pre_fuerte/joint_actuation_plugin.h>

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

GZ_REGISTER_DYNAMIC_CONTROLLER("joint_actuation_plugin", JointActuationPlugin);


// Constructor
JointActuationPlugin::JointActuationPlugin(Entity *parent) :
  Controller(parent)
{
 this->parent_ = dynamic_cast<Model*>(this->parent);
 // parent_ = dynamic_cast<Model*> (parent);

  if (!parent_)
    gzthrow("joint actuation plugin requires a Model as its parent");

  prevUpdateTime = Simulator::Instance()->GetSimTime();

  Param::Begin(&parameters);

  robotNamespaceP = new ParamT<std::string> ("robotNamespace", "/", 0);
  topicNameP = new ParamT<std::string> ("topicName", "", 1);
  Param::End();

  alive_ = true;

}

// Destructor
JointActuationPlugin::~JointActuationPlugin()
{
  delete robotNamespaceP;
  delete topicNameP;
  delete callback_queue_thread_;
  delete rosnode_;
}

// Load the controller
void JointActuationPlugin::LoadChild(XMLConfigNode *node)
{

  pos_iface_ = dynamic_cast<libgazebo::PositionIface*> (GetIface("position"));

ActuationCmdStruc zero_cmd;
  zero_cmd.value = 0;
  zero_cmd.start_time = 0;
  zero_cmd.duration = 0;
  zero_cmd.on_flag = false;
     
  for (unsigned int i = 0; i < this->parent_->GetJointCount(); i ++)
  {
     joint = this->parent_->GetJoint(i);
     std::string joint_name = joint->GetName();
     _joint_actuation_values.insert(make_pair(joint_name, zero_cmd));
  }

  // Initialize the ROS node and subscribe to cmd_vel

  robotNamespaceP->Load(node);
  robotNamespace = robotNamespaceP->GetValue();

    int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "joint_actuation_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(robotNamespace);

  ROS_INFO("starting joint_actuation_plugin in ns: %s", this->robotNamespace.c_str());

  topicNameP->Load(node);
  topicName = topicNameP->GetValue();

  // ROS: Subscribe to the joint actuation command topic
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<atlas_gazebo_msgs::ActuatorCmd>(topicName, 1,
                                                          boost::bind(&JointActuationPlugin::actuationCmdCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);

}

// Initialize the controller
void JointActuationPlugin::InitChild()
{
  callback_queue_thread_ = new boost::thread(boost::bind(&JointActuationPlugin::QueueThread, this));
}

// Load the controller
void JointActuationPlugin::SaveChild(std::string &prefix, std::ostream &stream)
{

}

// Reset
void JointActuationPlugin::ResetChild()
{

}

// Update the controller
void JointActuationPlugin::UpdateChild()
{

  Time stepTime;
  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = Simulator::Instance()->GetSimTime() - prevUpdateTime;
  prevUpdateTime = Simulator::Instance()->GetSimTime();

// Loop through all Joints
std::map<std::string, ActuationCmdStruc>::iterator it;

 for (unsigned int i = 0; i < this->parent_->GetJointCount(); i ++)
 {
        std::string joint_name = this->parent_->GetJoint(i)->GetName();
	
        
	it=_joint_actuation_values.find(joint_name);
	
	// both time_now, duration and start_time must also in simTime.  
	double time_now = Simulator::Instance()->GetSimTime().Double();
	lock.lock();	
	double duration = it->second.duration; // in seconds
	double start_time =  it->second.start_time; 
	bool on_flag = it->second.on_flag;
	lock.unlock();
	
	if(on_flag==true) // if flag is on
	{
	  if (time_now <= start_time + duration)  
	  { 
	    //Joint *hj = this->parent_->GetJoint(i);
	   //std::cout << joint_name << ": " << it->second.value << std::endl;
	    (this->parent_->GetJoint(i))->SetForce(0,35*it->second.value);
	    // this->parent_->GetJoint(i)->SetTorque(it->second.value);
	  }
	  else  // duration has expired, so clear forces. (duration is reset if a new torque command is received)
	  { 
	        //clear force
		(this->parent_->GetJoint(i))->SetForce(0,0);
		lock.lock();	
		it->second.on_flag = false;
		lock.unlock();	
	  }
        }
 } // end for loop

  //myIface->Unlock();
}// end update child.


// Finalize the controller
void JointActuationPlugin::FiniChild()
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

void JointActuationPlugin::actuationCmdCallback(const atlas_gazebo_msgs::ActuatorCmd::ConstPtr& cmd_msg)
{

//Double check that the msg is for the correct robot name (this->parent_->getName()). 
  if(cmd_msg->robot_name == this->parent_->GetName())
  {

      lock.lock();
     //int64_t t   =  cmd_msg->header.stamp.toNSec(); // Not Used     
      double msg_start_time  = Simulator::Instance()->GetSimTime().Double(); // All timing is done relative to sim time.
      //for all joints in message update ActuationCmd Buffer.      
      for (std::vector<int>::size_type i = 0; i != cmd_msg->actuator_name.size(); i++)
      {
	std::string joint_name = cmd_msg->actuator_name[i];
	std::map<std::string, ActuationCmdStruc>::iterator it;
	it=_joint_actuation_values.find(joint_name);	
	  ActuationCmdStruc act_cmd;
	  act_cmd.start_time = msg_start_time; // sim time when msg was received.
	  act_cmd.value = cmd_msg->actuator_effort[i];
	  
	   //std::cout << joint_name << ": " << act_cmd.value << std::endl;
	  act_cmd.duration = cmd_msg->effort_duration[i];
	  act_cmd.on_flag = true;
	  it->second = act_cmd;
      }// end for
        lock.unlock();
   }
  
}

// NEW: custom callback queue thread
void JointActuationPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}


