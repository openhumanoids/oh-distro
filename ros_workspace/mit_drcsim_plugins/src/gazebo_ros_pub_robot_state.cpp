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



#include "paladin_drcsim_plugins/gazebo_ros_pub_robot_state.h"



namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosPubRobotState::GazeboRosPubRobotState()
{
  this->robotStateConnectCount = 0;
  this->last_update_time_ = common::Time(0);

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosPubRobotState::~GazeboRosPubRobotState()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  // Finalize the controller
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosPubRobotState::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  
    // Get the world name.
  this->world = _parent->GetWorld();
  this->parent_model_ = _parent;

  
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_WARN("Laser plugin missing <frameName>, defaults to /world");
    this->frameName = "/world";
  }
  else
    this->frameName = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_WARN("Laser plugin missing <topicName>, defaults to /world");
    this->topicName = "/true_robot_state";
  }
  else
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
  
    if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO("Camera plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();
  
  
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);


  // Custom Callback Queue
  /*ros::AdvertiseOptions p3d_ao = ros::AdvertiseOptions::create<nav_msgs::Odometry>(
    this->topicName,1,
    boost::bind( &GazeboRosP3D::P3DConnect,this),
    boost::bind( &GazeboRosP3D::P3DDisconnect,this), ros::VoidPtr(), &this->p3d_queue_);
  this->pub_ = this->rosnode_->advertise(p3d_ao);*/

  if (this->topicName != "")
  {
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<atlas_gazebo_msgs::RobotState>(
    this->topicName,1,
    boost::bind( &GazeboRosPubRobotState::RobotStateConnect,this),
    boost::bind( &GazeboRosPubRobotState::RobotStateDisconnect,this), ros::VoidPtr(), &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);
  }
  
    // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosPubRobotState::QueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosPubRobotState::UpdateChild, this));
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
  //Time cur_time = Simulator::Instance()->GetSimTime();
  common::Time cur_time = this->world->GetSimTime();
  // std::cout << "Simtime" << cur_time.Double() <<std::endl;
  // construct world state message
  if (cur_time - this->last_update_time_ >= this->update_period_)
  {
        
    this->lock.lock();

    // compose robotStateMsg
    this->robotStateMsg.header.frame_id = this->frameName;
    this->robotStateMsg.header.stamp.fromSec(cur_time.Double());

     // get name
     // this->robotStateMsg.name.push_back(std::string(biter->second->GetName()));
      this->robotStateMsg.robot_name = std::string(this->parent_model_->GetName());
      // set pose
      // get pose from simulator
      math::Pose pose;
      math::Quaternion rot;
      math::Vector3 pos;

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
      geom_pose.orientation.w = rot.w;     
      this->robotStateMsg.body_pose =geom_pose;
    

      // set velocities
      // get Rates 
      math::Vector3 vpos = this->parent_model_->GetWorldLinearVel();
      math::Vector3 veul = this->parent_model_->GetWorldAngularVel();

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
	    physics::JointPtr joint = this->parent_model_->GetJoint(i);
            this->joints_.insert(make_pair(joint->GetName(), joint));
		//populate joints_ once	
	  }
	}

 	 typedef std::map<std::string, physics::JointPtr > joints_mapType;
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

    this->last_update_time_ = cur_time;
  }

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


GZ_REGISTER_MODEL_PLUGIN(GazeboRosPubRobotState);
} //end namespace
