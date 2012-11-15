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



#include <algorithm>
#include <assert.h>

#include "atlas_gazebo_plugins_pre_fuerte/gazebo_ros_pub_robot_state.h"

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <boost/bind.hpp>


using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_pub_robot_state", GazeboRosPubRobotState);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosPubRobotState::GazeboRosPubRobotState(Entity *parent)
    : Controller(parent)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboRosPubRobotState controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->frameNameP = new ParamT<std::string>("frameName", "base_link", 0);
//this->frameNameP = new ParamT<std::string>("frameName", "world", 0);
  Param::End();

  this->robotStateConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosPubRobotState::~GazeboRosPubRobotState()
{
  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosPubRobotState::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  // Custom Callback Queue
  /*ros::AdvertiseOptions p3d_ao = ros::AdvertiseOptions::create<nav_msgs::Odometry>(
    this->topicName,1,
    boost::bind( &GazeboRosP3D::P3DConnect,this),
    boost::bind( &GazeboRosP3D::P3DDisconnect,this), ros::VoidPtr(), &this->p3d_queue_);
  this->pub_ = this->rosnode_->advertise(p3d_ao);*/


  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<atlas_gazebo_msgs::RobotState>(
    this->topicName,1,
    boost::bind( &GazeboRosPubRobotState::RobotStateConnect,this),
    boost::bind( &GazeboRosPubRobotState::RobotStateDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);

}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosPubRobotState::RobotStateConnect()
{
  this->robotStateConnectCount++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosPubRobotState::RobotStateDisconnect()
{
  this->robotStateConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosPubRobotState::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosPubRobotState::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosPubRobotState::UpdateChild()
{
  /***************************************************************/
  /*                                                             */
  /*  this is called at every update simulation step             */
  /*                                                             */
  /***************************************************************/
  if (this->robotStateConnectCount == 0)
    return;

  /***************************************************************/
  /*                                                             */
  /*  publish                                                    */
  /*                                                             */
  /***************************************************************/
  Time cur_time = Simulator::Instance()->GetSimTime();
  // std::cout << "Simtime" << cur_time.Double() <<std::endl;
  // construct world state message

    this->lock.lock();

    // compose robotStateMsg
    this->robotStateMsg.header.frame_id = this->frameName;
    this->robotStateMsg.header.stamp.fromSec(cur_time.Double());

     // get name
     // this->robotStateMsg.name.push_back(std::string(biter->second->GetName()));
	this->robotStateMsg.robot_name = std::string(this->parent_model_->GetName());
      // set pose
      // get pose from simulator
      Pose3d pose;
      Quatern rot;
      Vector3 pos;

      // Get Pose/Orientation 
       pose = this->parent_model_->GetWorldPose();
      // apply xyz offsets and get position and rotation components
      pos = pose.pos; // (add if there's offset) + this->xyzOffsets;
      rot = pose.rot;
      // apply rpy offsets
      /* add if there's offsets
      Quatern qOffsets;
      qOffsets.SetFromEuler(this->rpyOffsets);
      rot = qOffsets*rot;
      rot.Normalize();
      */
      geometry_msgs::Pose geom_pose; 
      geom_pose.position.x    = pos.x;
      geom_pose.position.y    = pos.y;
      geom_pose.position.z    = pos.z;
      geom_pose.orientation.x = rot.x;
      geom_pose.orientation.y = rot.y;
      geom_pose.orientation.z = rot.z;
      geom_pose.orientation.w = rot.u;     
      this->robotStateMsg.body_pose =geom_pose;
    

      // set velocities
      // get Rates 
      Vector3 vpos = this->parent_model_->GetWorldLinearVel();
      Vector3 veul = this->parent_model_->GetWorldAngularVel();

      // pass linear rates
      geometry_msgs::Twist geom_twist;
      geom_twist.linear.x        = vpos.x;
      geom_twist.linear.y        = vpos.y;
      geom_twist.linear.z        = vpos.z;
      // pass euler angular rates
      geom_twist.angular.x    = veul.x;
      geom_twist.angular.y    = veul.y;
      geom_twist.angular.z    = veul.z;
      this->robotStateMsg.body_twist=geom_twist;


      int joint_count = this->parent_model_->GetJointCount();

	/*for (int i = 0; i < joint_count ; i++)
	{
	gazebo::Joint* joint = this->parent_model_->GetJoint(i);
	std::string name =  joint->GetName();//joint name
	double current_position = joint->GetAngle(0).GetAsRadian(); // joint position
	double current_velocity = joint->GetVelocity(0); // joint velocity
	this->robotStateMsg.joint_name.push_back(name);
	this->robotStateMsg.joint_position.push_back(current_position);
	this->robotStateMsg.joint_velocity.push_back(current_velocity);
//std::cout << "    name[" << joint->GetName() << "] pos[" << joint->GetAngle(0).GetAsRadian() << "] rate [" << joint->GetVelocity(0) << "]\n";
	}*/

        //Get pointers to all joints in the model and store them in a map (which performs and intrinsic 	 alphabetical sort on joint name).
        //
	if(this->joints_.empty()) {
	  for (int i = 0; i < joint_count ; i++)
	  {
	    gazebo::Joint* joint = this->parent_model_->GetJoint(i);
            this->joints_.insert(make_pair(joint->GetName(), joint));
		//populate joints_ once	
	  }
	}

 	 typedef std::map<std::string, gazebo::Joint* > joints_mapType;
        for( joints_mapType::const_iterator it = this->joints_.begin(); it!=this->joints_.end(); it++)
        { 

	  std::string name =  it->second->GetName();//joint name
	  double current_position = it->second->GetAngle(0).GetAsRadian(); // joint position
	  double current_velocity = it->second->GetVelocity(0); // joint velocity
	  this->robotStateMsg.joint_name.push_back(name);
	  this->robotStateMsg.joint_position.push_back(current_position);
	  this->robotStateMsg.joint_velocity.push_back(current_velocity);

//std::cout << "    name[" << name << "] pos[" << current_position << "] rate [" << current_velocity<< "]\n";
        }

    this->pub_.publish(this->robotStateMsg);   
    this->robotStateMsg.joint_name.clear();
    this->robotStateMsg.joint_position.clear();
    this->robotStateMsg.joint_velocity.clear();
   
    this->lock.unlock();


}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosPubRobotState::FiniChild()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}


// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosPubRobotState::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}


