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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include "MITRobotPlugin.hh"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
MITRobotPlugin::MITRobotPlugin()
{
  /// initial anchor pose
  this->anchorPose = math::Vector3(0, 0, 0);
  this->warpRobot = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
MITRobotPlugin::~MITRobotPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void MITRobotPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  // initialize ros
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",
      ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  // ros stuff
  this->rosnode_ = new ros::NodeHandle("~");

  // Get the world name.
  this->world = _parent->GetWorld();
  this->model = _parent;
  this->world->EnablePhysicsEngine(true);

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->cmdVel = geometry_msgs::Twist();

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->fixedLink = this->model->GetLink("pelvis");
  // if (!this->fixedJoint)
  //   this->fixedJoint = this->AddJoint(this->world, this->model,
  //                                     physics::LinkPtr(), this->fixedLink,
  //                                     "revolute",
  //                                     math::Vector3(0, 0, 0),
  //                                     math::Vector3(0, 0, 1),
  //                                     0.0, 0.0);

  // On startup, simulate "virtual harness" by turning gravity off
  // allowing the controllers can initialize without the robot falling
  this->SetPluginMode("feet");
  this->harnessed = true;
  ROS_WARN("Start robot with gravity turned off for all links.");
  ROS_WARN("  rostopic pub /mode std_msgs/String '{data: \"nominal\"}'");
  ROS_WARN("To re-engage.");

  this->initialPose = this->fixedLink->GetWorldPose();

  // ros subscription
  std::string trajectory_topic_name = "/cmd_vel";
  ros::SubscribeOptions trajectory_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
    trajectory_topic_name, 100,
    boost::bind( &MITRobotPlugin::SetRobotCmdVel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->trajectory_sub_ = this->rosnode_->subscribe(trajectory_so);

  std::string pose_topic_name = "/pose";
  ros::SubscribeOptions pose_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    pose_topic_name, 100,
    boost::bind( &MITRobotPlugin::SetRobotPose,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->pose_sub_ = this->rosnode_->subscribe(pose_so);

  std::string mode_topic_name = "/mode";
  ros::SubscribeOptions mode_so =
    ros::SubscribeOptions::create<std_msgs::String>(
    mode_topic_name, 100,
    boost::bind( &MITRobotPlugin::SetPluginModeTopic,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->mode_sub_ = this->rosnode_->subscribe(mode_so);

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind( &MITRobotPlugin::QueueThread,this ) );

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&MITRobotPlugin::UpdateStates, this));
}

void MITRobotPlugin::SetPluginModeTopic(const std_msgs::String::ConstPtr &_str)
{
  this->SetPluginMode(_str->data);
}

void MITRobotPlugin::SetPluginMode(const std::string &_str)
{
  if (_str == "no_gravity")
  {
    // stop warping robot
    this->warpRobot = false;
    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
    if (this->fixedJoint)
      this->RemoveJoint(this->fixedJoint);
  }
  else if (_str == "feet")
  {
    // stop warping robot
    this->warpRobot = false;
    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetName() == "l_foot" || links[i]->GetName() == "r_foot")
        links[i]->SetGravityMode(true);
      else
        links[i]->SetGravityMode(false);
    }
    if (this->fixedJoint)
      this->RemoveJoint(this->fixedJoint);
  }
  else if (_str == "pinned")
  {
    // reinitialize pinning
    if (!this->fixedJoint)
      this->fixedJoint = this->AddJoint(this->world, this->model,
                                        physics::LinkPtr(), this->fixedLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->initialPose = this->fixedLink->GetWorldPose();

    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
  }
  else if (_str == "nominal")
  {
    // reinitialize pinning
    physics::Link_V links = this->model->GetAllLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    if (this->fixedJoint)
      this->RemoveJoint(this->fixedJoint);
  }
  else if (_str == "grab_fire_hose")
  {
    this->GrabLink("fire_hose", "coupling", "r_hand",
      math::Pose(math::Vector3(0, -0.3, -0.1),
                 math::Quaternion(0, 0, 0)));
  }
  else if (_str == "release_fire_hose")
  {
    this->RemoveJoint(this->grabJoint);
    // this->grabJoint.reset();
  }
  else
  {
    ROS_INFO("available modes:no_gravity, feet, pinned, nominal");
  }

}

void MITRobotPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warpRobot = false;
  }
  else
  {
    this->cmdVel = *_cmd;
    this->warpRobot = true;
    this->lastUpdateTime = this->world->GetSimTime().Double();
  }
}

void MITRobotPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose)
{
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z),
                  math::Quaternion(_pose->orientation.w,
                                   _pose->orientation.x,
                                   _pose->orientation.y,
                                   _pose->orientation.z));
  this->model->SetWorldPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
// dynamically add joint between 2 links
physics::JointPtr MITRobotPlugin::AddJoint(physics::WorldPtr _world,
                                           physics::ModelPtr _model,
                                           physics::LinkPtr _link1,
                                           physics::LinkPtr _link2,
                                           std::string _type,
                                           math::Vector3 _anchor,
                                           math::Vector3 _axis,
                                           double _upper, double _lower)
{
  physics::JointPtr joint = _world->GetPhysicsEngine()->CreateJoint(
    _type, _model);
  joint->Attach(_link1, _link2);
  // load adds the joint to a vector of shared pointers kept
  // in parent and child links, preventing joint from being destroyed.
  joint->Load(_link1, _link2, math::Pose(_anchor, math::Quaternion()));
  // joint->SetAnchor(0, _anchor);
  joint->SetAxis(0, _axis);
  joint->SetHighStop(0, _upper);
  joint->SetLowStop(0, _lower);

  if (_link1)
    joint->SetName(_link1->GetName() + std::string("_") +
                              _link2->GetName() + std::string("_joint"));
  else
    joint->SetName(std::string("world_") +
                              _link2->GetName() + std::string("_joint"));
  joint->Init();

  // disable collision between the link pair
  if (_link1)
    _link1->SetCollideMode("fixed");
  _link2->SetCollideMode("fixed");
  return joint;
}

////////////////////////////////////////////////////////////////////////////////
// attach a model to gripper
void MITRobotPlugin::GrabLink(std::string _modelName, std::string _linkName,
                              std::string _gripperName, math::Pose _pose)
{
  physics::ModelPtr grabModel = this->world->GetModel(_modelName);
  if (grabModel)
  {
    physics::LinkPtr grabLink = grabModel->GetLink(_linkName);
    if (grabLink)
    {
      physics::LinkPtr gripperLink = this->model->GetLink(_gripperName);
      if (gripperLink)
      {
        math::Pose pose = _pose + gripperLink->GetWorldPose();
        grabModel->SetLinkWorldPose(pose, grabLink);
        if (!this->grabJoint)
          this->grabJoint = this->AddJoint(this->world, this->model,
                                           gripperLink, grabLink,
                                           "revolute",
                                           math::Vector3(0, 0, 0),
                                           math::Vector3(0, 0, 1),
                                           0.0, 0.0);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// remove a joint
void MITRobotPlugin::RemoveJoint(physics::JointPtr &_joint)
{
  if (_joint)
  {
    // reenable collision between the link pair
    physics::LinkPtr parent = _joint->GetParent();
    physics::LinkPtr child = _joint->GetChild();
    parent->SetCollideMode("all");
    child->SetCollideMode("all");

    _joint->Detach();
    _joint.reset();
  }
}

void MITRobotPlugin::WarpDRCRobot(math::Pose _pose)
{
  // two ways, both requres to be done in a single step, is pause reliable? or do we
  // need some mutex.
  //   1. update poses, set the joint anchor offset properties,
  //      this requires introducing a new SetAnchor call with the
  //      new joint agnle
  //      will not do for now.
  //      using: this->fixedJoint->SetAnchor(0, _pose);
  //   or,
  //   2. less ideally, pause, break joint, update pose,
  //       create new joint, unpause

  // try 2. here
  bool p = this->world->IsPaused();
  bool e = this->world->GetEnablePhysicsEngine();
  this->world->EnablePhysicsEngine(false);
  this->world->SetPaused(true);
  if (this->fixedJoint)
    this->RemoveJoint(this->fixedJoint);
  this->model->SetLinkWorldPose(_pose, this->fixedLink);
  if (!this->fixedJoint)
    this->fixedJoint = this->AddJoint(this->world, this->model,
                                      physics::LinkPtr(), this->fixedLink,
                                      "revolute",
                                      math::Vector3(0, 0, 0),
                                      math::Vector3(0, 0, 1),
                                      0.0, 0.0);
  this->world->SetPaused(p);
  this->world->EnablePhysicsEngine(e);
}

// Set DRC Robot feet placement
void MITRobotPlugin::SetFeetPose(math::Pose _lPose, math::Pose _rPose)
{
  physics::LinkPtr l_foot = this->model->GetLink("l_foot");
  physics::LinkPtr r_foot = this->model->GetLink("r_foot");
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void MITRobotPlugin::UpdateStates()
{
  double cur_time = this->world->GetSimTime().Double();

  if (this->harnessed && cur_time > 10)
  {
    this->SetPluginMode("pinned");
    this->harnessed = false;
  }

  if (this->warpRobot && cur_time - this->lastUpdateTime >= 0)
  {
    double dt = cur_time - this->lastUpdateTime;
    if (dt > 0)
    {
      this->lastUpdateTime = cur_time;
      math::Pose cur_pose = this->fixedLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->cmdVel.linear.x, this->cmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->cmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the fixed joint
      this->WarpDRCRobot(new_pose);
    }
  }
}

void MITRobotPlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(MITRobotPlugin)
}
