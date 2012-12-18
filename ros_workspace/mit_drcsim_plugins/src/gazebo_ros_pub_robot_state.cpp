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
#include <sys/time.h>
#include <time.h>


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
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
#else
  this->ros_spinner_thread_.join();
#endif
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
    ROS_WARN(" plugin missing <frameName>, defaults to /world");
    this->frameName = "/world";
  }
  else
    this->frameName = _sdf->GetElement("frameName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    ROS_WARN(" plugin missing <topicName>, defaults to /world");
    this->topicName = "/true_robot_state";
  }
  else
    this->topicName = _sdf->GetElement("topicName")->GetValueString();
    
  
  if (!_sdf->HasElement("synchronization"))
  {
    ROS_WARN(" plugin missing <topicName>, defaults to false");
    this->synchronization = "false";
  }
  else
    this->synchronization = _sdf->GetElement("synchronization")->GetValueString();
    //For some reason it parses "true" as "1" but does not like GetValueBool().
  
    if (!_sdf->GetElement("updateRate"))
  {
    ROS_INFO(" plugin missing <updateRate>, defaults to 0");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();
  
  
  if (this->update_rate_ > 0.0)
    this->update_period_ = 1.0/this->update_rate_;
  else
    this->update_period_ = 0.0;

   // Init ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init( argc, argv, "gazebo", ros::init_options::NoSigintHandler); //ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    gzwarn << "should start ros::init in simulation by using the system plugin\n";
  }
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);
  
  if (this->topicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<atlas_gazebo_msgs::RobotState>(
      this->topicName,1,
      boost::bind( &GazeboRosPubRobotState::RobotStateConnect,this),
      boost::bind( &GazeboRosPubRobotState::RobotStateDisconnect,this), ros::VoidPtr(), &this->queue_);
    this->pub_ = this->rosnode_->advertise(ao);
#else
    this->pub_ = this->rosnode_->advertise<atlas_gazebo_msgs::RobotState>(this->topicName, 1);
#endif
  }

  
#ifdef USE_CBQ
    // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosPubRobotState::QueueThread,this ) );
#else
  this->ros_spinner_thread_ = boost::thread( boost::bind( &GazeboRosPubRobotState::RosSpinnerThread,this ) );
#endif

usleep(500000); // 0.5 sec delay to make sure that the subscribes find its publishers.

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

  if (this->world->IsPaused()) return;
  /***************************************************************/
  /*                                                             */
  /*  this is called at every update simulation step             */
  /*                                                             */
  /***************************************************************/
 #ifdef USE_CBQ
  //if (this->robotStateConnectCount == 0)
  //  return;
 #endif
  /***************************************************************/
  /*                                                             */
  /*  publish                                                    */
  /*                                                             */
  /***************************************************************/
  //Time cur_time = Simulator::Instance()->GetSimTime();
  common::Time cur_time = this->world->GetSimTime();
//  std::cout << "Simtime" << cur_time.Double() <<std::endl;
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
          current_position = fmod(current_position, 2*M_PI );
          // if out of bounds, then correct: (added due to Gazebo bug in dec 2012, mfallon
          // their angles arent bound checked
          if (current_position > 2*M_PI){
            current_position =  current_position- 2*M_PI;
          }else if (current_position < 0){
            current_position =  current_position+ 2*M_PI;
          }
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
    if(this->synchronization=="1")
    {
     // gzerr <<this->synchronization  << " ok!"<< std::endl;
     this->world->SetPaused(true);
    }
  
  }

}
#ifdef USE_CBQ
// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosPubRobotState::QueueThread()
{

  static const double timeout = 0.01;
  //gzwarn << "In GazeboRosPubRobotState::QueueThread()\n";
  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#else
void GazeboRosPubRobotState::RosSpinnerThread()
{
//  static const double timeout = 0.001;
//  while (this->rosnode_->ok())
//  {
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(timeout));
//  }
  while (this->rosnode_->ok())
  {
    usleep(1000);
    ros::spinOnce();
  }
}
#endif
GZ_REGISTER_MODEL_PLUGIN(GazeboRosPubRobotState);
} //end namespace
