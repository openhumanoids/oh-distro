/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <map>
#include <string>

#include "VRCPlugin.h"

namespace gazebo
{
GZ_REGISTER_WORLD_PLUGIN(VRCPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
VRCPlugin::VRCPlugin()
{
  /// initial anchor pose
  this->warpRobotWithCmdVel = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
VRCPlugin::~VRCPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueueThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void VRCPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->world = _parent;
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&VRCPlugin::DeferredLoad, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void VRCPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading vrc plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // load VRC ROS API
  this->LoadVRCROSAPI();

  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->lastUpdateTime = this->world->GetSimTime().Double();
  this->robotCmdVel = geometry_msgs::Twist();

  // Load Robot
  this->atlas.Load(this->world, this->sdf);

  // Load Vehicle
  this->drcVehicle.Load(this->world, this->sdf);

  // Load fire hose and standpipe
  this->drcFireHose.Load(this->world, this->sdf);

  // Setup ROS interfaces for robot
  this->LoadRobotROSAPI();

  // Harness the Robot
  // On startup, simulate "virtual harness" by turning gravity off
  // allowing the controllers can initialize without the robot falling
  if (this->atlas.isInitialized)
  {
    this->SetRobotMode("pinned");
    this->atlas.startupHarness = true;
    ROS_INFO("Start robot with gravity turned off and harnessed.");
    ROS_INFO("Resume to nominal mode after 10 seconds.");
  }

  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
    boost::bind(&VRCPlugin::ROSQueueThread, this));

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&VRCPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
{
  this->SetRobotMode(_str->data);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotMode(const std::string &_str)
{
  if (_str == "no_gravity")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
  }
  else if (_str == "feet")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetName() == "l_foot" || links[i]->GetName() == "r_foot")
        links[i]->SetGravityMode(true);
      else
        links[i]->SetGravityMode(false);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
  }
  else if (_str == "pinned")
  {
    // pinning robot, and turning off effect of gravity
    if (!this->atlas.pinJoint)
      this->atlas.pinJoint = this->AddJoint(this->world,
                                        this->atlas.model,
                                        physics::LinkPtr(),
                                        this->atlas.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->atlas.initialPose = this->atlas.pinLink->GetWorldPose();

    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
  }
  else if (_str == "nominal")
  {
    // reinitialize pinning
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->atlas.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    if (this->atlas.pinJoint)
      this->RemoveJoint(this->atlas.pinJoint);
  }
  else
  {
    ROS_INFO("available modes:no_gravity, feet, pinned, nominal");
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warpRobotWithCmdVel = false;
  }
  else
  {
    this->robotCmdVel = *_cmd;
    this->warpRobotWithCmdVel = true;
    this->lastUpdateTime = this->world->GetSimTime().Double();
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose)
{
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z),
                  math::Quaternion(_pose->orientation.w,
                                   _pose->orientation.x,
                                   _pose->orientation.y,
                                   _pose->orientation.z));
  this->atlas.model->SetWorldPose(pose);
  
  // also set body velocity to zero, added by sk
  this->atlas.model->SetWorldTwist(math::Vector3(0,0,0), math::Vector3(0,0,0));  
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotGrabFireHose(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  /// \todo: get these from incoming message
  std::string modelName = "fire_hose";
  std::string linkName = "coupling";
  std::string gripperName = "r_hand";
  math::Pose relPose(math::Vector3(0, -0.3, -0.1),
               math::Quaternion(0, 0, 0));

  physics::ModelPtr grabModel = this->world->GetModel(modelName);
  if (grabModel)
  {
    physics::LinkPtr object = grabModel->GetLink(linkName);
    if (object)
    {
      physics::LinkPtr gripper = this->atlas.model->GetLink(gripperName);
      if (gripper)
      {
        // teleports the object being attached together
        math::Pose pose = relPose + gripper->GetWorldPose();
        grabModel->SetLinkWorldPose(pose, object);

        if (!this->grabJoint)
          this->grabJoint = this->AddJoint(this->world, this->atlas.model,
                                           gripper, object,
                                           "revolute",
                                           math::Vector3(0, 0, 0),
                                           math::Vector3(0, 0, 1),
                                           0.0, 0.0);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &/*_cmd*/)
{
  this->RemoveJoint(this->grabJoint);
}

////////////////////////////////////////////////////////////////////////////////
// dynamically add joint between 2 links
physics::JointPtr VRCPlugin::AddJoint(physics::WorldPtr _world,
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

/*
  // disable collision between the link pair
  if (_link1)
    _link1->SetCollideMode("fixed");
  if (_link2)
    _link2->SetCollideMode("fixed");
*/
  return joint;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotEnterCar(const geometry_msgs::Pose::ConstPtr &_pose)
{
  // Check if drcVehicle.model is loaded
  if (!this->drcVehicle.model)
  {
    ROS_ERROR("drc_vehicle model not found, cannot enter car.");
    return;
  }
  math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                    _pose->orientation.y, _pose->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z), q);
  if (this->atlas.pinJoint)
    this->RemoveJoint(this->atlas.pinJoint);

  this->atlas.vehicleRelPose = math::Pose(math::Vector3(0.52, 0.5, 2),
                                              math::Quaternion());

  this->atlas.model->SetLinkWorldPose(pose +
    this->atlas.vehicleRelPose + this->drcVehicle.model->GetWorldPose(),
    this->atlas.pinLink);

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drcVehicle.model,
                                       this->drcVehicle.seatLink,
                                       this->atlas.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
/*
  std::map<std::string, double> jointPositions;
  jointPositions["atlas::back_lbz" ] =  0.00;
  jointPositions["atlas::back_mby" ] =  0.00;
  jointPositions["atlas::back_ubx" ] =  0.00;
  jointPositions["atlas::neck_ay"  ] =  0.00;
  jointPositions["atlas::l_leg_uhz"] =  0.00;
  jointPositions["atlas::l_leg_mhx"] =  0.00;
  jointPositions["atlas::l_leg_lhy"] = -1.80;
  jointPositions["atlas::l_leg_kny"] =  1.80;
  jointPositions["atlas::l_leg_uay"] =  0.00;
  jointPositions["atlas::l_leg_lax"] =  0.00;
  jointPositions["atlas::r_leg_uhz"] =  0.00;
  jointPositions["atlas::r_leg_mhx"] =  0.00;
  jointPositions["atlas::r_leg_lhy"] = -1.80;
  jointPositions["atlas::r_leg_kny"] =  1.80;
  jointPositions["atlas::r_leg_uay"] =  0.00;
  jointPositions["atlas::r_leg_lax"] =  0.00;
  jointPositions["atlas::l_arm_elx"] =  0.00;
  jointPositions["atlas::l_arm_ely"] =  0.00;
  jointPositions["atlas::l_arm_mwx"] =  0.00;
  jointPositions["atlas::l_arm_shx"] =  0.00;
  jointPositions["atlas::l_arm_usy"] = -1.60;
  jointPositions["atlas::l_arm_uwy"] =  0.00;
  jointPositions["atlas::r_arm_elx"] =  0.00;
  jointPositions["atlas::r_arm_ely"] =  0.00;
  jointPositions["atlas::r_arm_mwx"] =  0.00;
  jointPositions["atlas::r_arm_shx"] =  0.00;
  jointPositions["atlas::r_arm_usy"] =  1.60;
  jointPositions["atlas::r_arm_uwy"] =  0.00;
  this->atlas.model->SetJointPositions(jointPositions);
*/

  // wait for action server to come up
  while (!this->jointTrajectoryController.clientTraj->waitForServer(
    ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  this->jointTrajectoryController.sendTrajectory(
    this->jointTrajectoryController.seatingConfiguration());

  // Wait for trajectory completion
  while (!jointTrajectoryController.getState().isDone() && ros::ok())
  {
    ros::spinOnce();
    usleep(50000);
  }
  ROS_INFO("set configuration done");

  this->atlas.vehicleRelPose = math::Pose(math::Vector3(0.52, 0.5, 1.27),
                                              math::Quaternion());

  this->RemoveJoint(this->vehicleRobotJoint);

  this->atlas.model->SetLinkWorldPose(pose +
    this->atlas.vehicleRelPose + this->drcVehicle.model->GetWorldPose(),
    this->atlas.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drcVehicle.model,
                                       this->drcVehicle.seatLink,
                                       this->atlas.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::RobotExitCar(const geometry_msgs::Pose::ConstPtr &_pose)
{
  // Check if drcVehicle.model is loaded
  if (!this->drcVehicle.model)
  {
    ROS_ERROR("drc_vehicle model not found, cannot exit car.");
    return;
  }
  math::Quaternion q(_pose->orientation.w, _pose->orientation.x,
                    _pose->orientation.y, _pose->orientation.z);
  q.Normalize();
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z), q);

  if (this->atlas.pinJoint)
    this->RemoveJoint(this->atlas.pinJoint);

  this->atlas.vehicleRelPose = math::Pose(math::Vector3(0.52, 1.7, 1.20),
                                              math::Quaternion());

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);

  this->atlas.model->SetLinkWorldPose(pose +
    this->atlas.vehicleRelPose + this->drcVehicle.model->GetWorldPose(),
    this->atlas.pinLink);

  if (!this->vehicleRobotJoint)
    this->vehicleRobotJoint = this->AddJoint(this->world,
                                       this->drcVehicle.model,
                                       this->drcVehicle.seatLink,
                                       this->atlas.pinLink,
                                       "revolute",
                                       math::Vector3(0, 0, 0),
                                       math::Vector3(0, 0, 1),
                                       0.0, 0.0);

  // wait for action server to come up
  while (!this->jointTrajectoryController.clientTraj->waitForServer(
    ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  this->jointTrajectoryController.sendTrajectory(
    this->jointTrajectoryController.standingConfiguration());

  // Wait for trajectory completion
  while (!jointTrajectoryController.getState().isDone() && ros::ok())
  {
    ros::spinOnce();
    usleep(50000);
  }
  ROS_INFO("set configuration done");

  if (this->vehicleRobotJoint)
    this->RemoveJoint(this->vehicleRobotJoint);
}


////////////////////////////////////////////////////////////////////////////////
// remove a joint
void VRCPlugin::RemoveJoint(physics::JointPtr &_joint)
{
  if (_joint)
  {
    // reenable collision between the link pair
    physics::LinkPtr parent = _joint->GetParent();
    physics::LinkPtr child = _joint->GetChild();
    if (parent)
      parent->SetCollideMode("all");
    if (child)
      child->SetCollideMode("all");

    _joint->Detach();
    _joint.reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose,
                         const std::map<std::string, double> &/*_jp*/)
{
  this->Teleport(_pinLink, _pinJoint, _pose);
  /// \todo: use _jp to set robot configuration
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose)
{
  // pause, break joint, update pose, create new joint, unpause
  bool p = this->world->IsPaused();
  bool e = this->world->GetEnablePhysicsEngine();
  this->world->EnablePhysicsEngine(false);
  this->world->SetPaused(true);
  if (_pinJoint)
    this->RemoveJoint(_pinJoint);
  _pinLink->GetModel()->SetLinkWorldPose(_pose, _pinLink);
  if (!_pinJoint)
    _pinJoint = this->AddJoint(this->world,
                               _pinLink->GetModel(),
                               physics::LinkPtr(),
                               this->atlas.pinLink,
                               "revolute",
                               math::Vector3(0, 0, 0),
                               math::Vector3(0, 0, 1),
                               0.0, 0.0);
  this->world->SetPaused(p);
  this->world->EnablePhysicsEngine(e);
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void VRCPlugin::UpdateStates()
{
  double curTime = this->world->GetSimTime().Double();

  if (this->atlas.isInitialized &&
      this->atlas.startupHarness && curTime > 10)
  {
    this->SetRobotMode("nominal");
    this->atlas.startupHarness = false;
  }

  if (curTime > this->lastUpdateTime)
  {
    this->CheckThreadStart();

    double dt = curTime - this->lastUpdateTime;

    if (this->warpRobotWithCmdVel)
    {
      this->lastUpdateTime = curTime;
      math::Pose cur_pose = this->atlas.pinLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->robotCmdVel.linear.x,
                        this->robotCmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->atlas.initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->robotCmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the pin joint
      this->Teleport(this->atlas.pinLink,
                     this->atlas.pinJoint,
                     new_pose);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::ROSQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::FireHose::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;

  sdf::ElementPtr sdf = _sdf->GetElement("drc_fire_hose");
  // Get special coupling links (on the firehose side)
  std::string fireHoseModelName = sdf->GetValueString("fire_hose_model");
  this->fireHoseModel = _world->GetModel(fireHoseModelName);
  if (!this->fireHoseModel)
  {
    ROS_INFO("fire_hose_model [%s] not found", fireHoseModelName.c_str());
    return;
  }
  this->initialFireHosePose = this->fireHoseModel->GetWorldPose();

  // Get coupling link
  std::string couplingLinkName = sdf->GetValueString("coupling_link");
  this->couplingLink = this->fireHoseModel->GetLink(couplingLinkName);
  if (!this->couplingLink)
  {
    ROS_ERROR("coupling link [%s] not found", couplingLinkName.c_str());
    return;
  }

  // Get joints
  this->fireHoseJoints = this->fireHoseModel->GetJoints();

  // Get links
  this->fireHoseLinks = this->fireHoseModel->GetLinks();

  // Get special links from the standpipe
  std::string standpipeModelName = sdf->GetValueString("standpipe_model");
  this->standpipeModel = _world->GetModel(standpipeModelName);
  if (!this->standpipeModel)
  {
    ROS_ERROR("standpipe model [%s] not found", standpipeModelName.c_str());
    return;
  }

  // Get spout link
  std::string spoutLinkName = sdf->GetValueString("spout_link");
  this->spoutLink = this->standpipeModel->GetLink(spoutLinkName);
  if (!this->spoutLink)
  {
    ROS_ERROR("spout link [%s] not found", spoutLinkName.c_str());
    return;
  }

  this->threadPitch = sdf->GetValueDouble("thread_pitch");

  this->couplingRelativePose = sdf->GetValuePose("coupling_relative_pose");

  // Set initial configuration
  this->SetInitialConfiguration();

  this->isInitialized = true;
}

void VRCPlugin::CheckThreadStart()
{
  if (!this->drcFireHose.isInitialized)
    return;

  // gzerr << "coupling [" << this->couplingLink->GetWorldPose() << "]\n";
  // gzerr << "spout [" << this->spoutLink->GetWorldPose() << "]\n"
  math::Pose connectPose(this->drcFireHose.couplingRelativePose);
  math::Pose relativePose = this->drcFireHose.couplingLink->GetWorldPose() -
                            this->drcFireHose.spoutLink->GetWorldPose();

  math::Pose connectOffset = relativePose - connectPose;

  double posErr = (relativePose.pos - connectPose.pos).GetLength();
  double rotErr = (relativePose.rot.GetZAxis() -
                   connectPose.rot.GetZAxis()).GetLength();

  // gzdbg << "connect offset [" << connectOffset
  //       << "] xyz [" << posErr
  //       << "] rpy [" << rotErr
  //       << "]\n";

  if (!this->drcFireHose.screwJoint)
  {
    if (posErr < 0.01 && rotErr < 0.01)
    {
      this->drcFireHose.screwJoint =
        this->AddJoint(this->world, this->drcFireHose.fireHoseModel,
                       this->drcFireHose.spoutLink,
                       this->drcFireHose.couplingLink,
                       "screw",
                       math::Vector3(0, 0, 0),
                       math::Vector3(0, 0, 1),
                       20.0/1000, -0.5/1000);
                       // 20.0, -0.5); // recover threadPitch
    }
  }
  else
  {
    // check joint position to disconnect
    double position = this->drcFireHose.screwJoint->GetAngle(0).Radian();
    // gzerr << "position " << position << "\n";
    if (position < -0.0003)
      this->RemoveJoint(this->drcFireHose.screwJoint);
  }
}


////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Vehicle::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;
  // load parameters
  if (_sdf->HasElement("drc_vehicle") &&
      _sdf->GetElement("drc_vehicle")->HasElement("model_name"))
  {
    this->model = _world->GetModel(_sdf->GetElement("drc_vehicle")
                        ->GetValueString("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <drc_vehicle><model_name> blocks. using default.");
    this->model = _world->GetModel("drc_vehicle");
  }

  if (!this->model)
  {
    ROS_INFO("drc vehicle not found.");
    return;
  }

  if (_sdf->HasElement("drc_vehicle") &&
      _sdf->GetElement("drc_vehicle")->HasElement("seat_link"))
  {
    this->seatLink = this->model->GetLink(_sdf->GetElement("drc_vehicle")
                        ->GetValueString("seat_link"));
  }
  else
  {
    ROS_INFO("Can't find <drc_vehicle><seat_link> blocks, using default.");
    this->seatLink = this->model->GetLink("chassis");
  }

  if (!this->seatLink)
  {
    ROS_ERROR("drc vehicle seat link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->seatLink->GetWorldPose();
  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::Robot::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;

  // load parameters
  if (_sdf->HasElement("atlas") &&
      _sdf->GetElement("atlas")->HasElement("model_name"))
  {
    this->model = _world->GetModel(_sdf->GetElement("atlas")
                        ->GetValueString("model_name"));
  }
  else
  {
    ROS_INFO("Can't find <atlas><model_name> blocks. using default.");
    this->model = _world->GetModel("atlas");
  }

  if (!this->model)
  {
    ROS_ERROR("atlas model not found.");
    return;
  }

  if (_sdf->HasElement("atlas") &&
      _sdf->GetElement("atlas")->HasElement("pin_link"))
  {
    this->pinLink = this->model->GetLink(_sdf->GetElement("atlas")
                        ->GetValueString("pin_link"));
  }
  else
  {
    ROS_INFO("Can't find <atlas><pin_link> blocks, using default.");
    this->pinLink = this->model->GetLink("utorso");
  }

  if (!this->pinLink)
  {
    ROS_ERROR("atlas robot pin link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->pinLink->GetWorldPose();
  this->isInitialized = true;
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadVRCROSAPI()
{
  // ros subscription
  std::string robot_enter_car_topic_name = "drc_world/robot_enter_car";
  ros::SubscribeOptions robot_enter_car_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_enter_car_topic_name, 100,
    boost::bind(&VRCPlugin::RobotEnterCar, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subRobotEnterCar = this->rosNode->subscribe(robot_enter_car_so);

  std::string robot_exit_car_topic_name = "drc_world/robot_exit_car";
  ros::SubscribeOptions robot_exit_car_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_exit_car_topic_name, 100,
    boost::bind(&VRCPlugin::RobotExitCar, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subRobotExitCar = this->rosNode->subscribe(robot_exit_car_so);

  std::string robot_grab_topic_name = "drc_world/robot_grab_link";
  ros::SubscribeOptions robot_grab_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_grab_topic_name, 100,
    boost::bind(&VRCPlugin::RobotGrabFireHose, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subRobotGrab = this->rosNode->subscribe(robot_grab_so);

  std::string robot_release_topic_name = "drc_world/robot_release_link";
  ros::SubscribeOptions robot_release_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    robot_release_topic_name, 100,
    boost::bind(&VRCPlugin::RobotReleaseLink, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->subRobotRelease = this->rosNode->subscribe(robot_release_so);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::LoadRobotROSAPI()
{
  // ros subscription
  std::string trajectory_topic_name = "atlas/cmd_vel";
  ros::SubscribeOptions trajectory_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
    trajectory_topic_name, 100,
    boost::bind(&VRCPlugin::SetRobotCmdVel, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->atlas.subTrajectory = this->rosNode->subscribe(trajectory_so);

  std::string pose_topic_name = "atlas/set_pose";
  ros::SubscribeOptions pose_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    pose_topic_name, 100,
    boost::bind(&VRCPlugin::SetRobotPose, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->atlas.subPose = this->rosNode->subscribe(pose_so);

  std::string configuration_topic_name = "atlas/configuration";
  ros::SubscribeOptions configuration_so =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    configuration_topic_name, 100,
    boost::bind(&VRCPlugin::SetRobotConfiguration, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->atlas.subConfiguration =
    this->rosNode->subscribe(configuration_so);

  std::string mode_topic_name = "atlas/mode";
  ros::SubscribeOptions mode_so =
    ros::SubscribeOptions::create<std_msgs::String>(
    mode_topic_name, 100,
    boost::bind(&VRCPlugin::SetRobotModeTopic, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->atlas.subMode = this->rosNode->subscribe(mode_so);
}

////////////////////////////////////////////////////////////////////////////////
void VRCPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr &_cmd)
{
  // added by sk
  std::map<std::string, double> jointPositions;
  gazebo::physics::JointPtr j;	
  for (int i=0; i<_cmd->position.size(); i++) {
  	std::stringstream ss;
	ss << "atlas::" << _cmd->name[i];
	std::string s = ss.str();
  	jointPositions[s] = _cmd->position[i];
  	j = this->atlas.model->GetJoint(_cmd->name[i]);
//  	j->SetVelocity(0,0.0);
//  	j->SetVelocity(1,0.0);
//  	j->SetVelocity(2,0.0);
//  	j->SetForce(0,0.0);
//  	j->SetForce(1,0.0);
//  	j->SetForce(2,0.0);
  }
  this->atlas.model->SetJointPositions(jointPositions);
  //this->atlas.model->SetJointVelocities(jointVelocities);
}
}
