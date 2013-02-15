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
#include <string>
#include <vector>
#include <sensor_msgs/Imu.h>

#include "FloatingSandiaHandPlugin.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(FloatingSandiaHandPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
FloatingSandiaHandPlugin::FloatingSandiaHandPlugin()
{

// ================ MIT MODIFICATION ===========================  
  this->leftImuLinkName = "left_palm";//"l_hand";
  this->rightImuLinkName = "right_palm";//"r_hand";
// ================ MIT MODIFICATION ===========================  
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
FloatingSandiaHandPlugin::~FloatingSandiaHandPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void FloatingSandiaHandPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();
  this->sdf = _sdf;
  this->lastControllerUpdateTime = this->world->GetSimTime();

  // initialize imu
  this->lastImuTime = this->world->GetSimTime();

  // get joints
  this->jointNames.push_back("left_f0_j0");
  this->jointNames.push_back("left_f0_j1");
  this->jointNames.push_back("left_f0_j2");
  this->jointNames.push_back("left_f1_j0");
  this->jointNames.push_back("left_f1_j1");
  this->jointNames.push_back("left_f1_j2");
  this->jointNames.push_back("left_f2_j0");
  this->jointNames.push_back("left_f2_j1");
  this->jointNames.push_back("left_f2_j2");
  this->jointNames.push_back("left_f3_j0");
  this->jointNames.push_back("left_f3_j1");
  this->jointNames.push_back("left_f3_j2");
  
// ================ MIT ADDITION ===========================  
  // the floating part
  this->jointNames.push_back("left_base_x");
  this->jointNames.push_back("left_base_y");
  this->jointNames.push_back("left_base_z");
  this->jointNames.push_back("left_base_roll");
  this->jointNames.push_back("left_base_pitch");
  this->jointNames.push_back("left_base_yaw");
// ================ END MIT ADDITION ===========================    

  this->jointNames.push_back("right_f0_j0");
  this->jointNames.push_back("right_f0_j1");
  this->jointNames.push_back("right_f0_j2");
  this->jointNames.push_back("right_f1_j0");
  this->jointNames.push_back("right_f1_j1");
  this->jointNames.push_back("right_f1_j2");
  this->jointNames.push_back("right_f2_j0");
  this->jointNames.push_back("right_f2_j1");
  this->jointNames.push_back("right_f2_j2");
  this->jointNames.push_back("right_f3_j0");
  this->jointNames.push_back("right_f3_j1");
  this->jointNames.push_back("right_f3_j2");

// ================ MIT ADDITION ===========================    
  // the other floating part  
  this->jointNames.push_back("right_base_x");
  this->jointNames.push_back("right_base_y");
  this->jointNames.push_back("right_base_z");
  this->jointNames.push_back("right_base_roll");
  this->jointNames.push_back("right_base_pitch");
  this->jointNames.push_back("right_base_yaw");
// ================ END MIT ADDITION ===========================  

  this->joints.resize(this->jointNames.size());

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->joints[i] = this->model->GetJoint(this->jointNames[i]);
    if (!this->joints[i])
    {
      ROS_ERROR("sandia hand not present, plugin not loaded");
      return;
    }
  }

  this->errorTerms.resize(this->joints.size());

  this->leftJointStates.name.resize(this->joints.size() / 2);
  this->leftJointStates.position.resize(this->joints.size() / 2);
  this->leftJointStates.velocity.resize(this->joints.size() / 2);
  this->leftJointStates.effort.resize(this->joints.size() / 2);

  this->rightJointStates.name.resize(this->joints.size() / 2);
  this->rightJointStates.position.resize(this->joints.size() / 2);
  this->rightJointStates.velocity.resize(this->joints.size() / 2);
  this->rightJointStates.effort.resize(this->joints.size() / 2);

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    if (i < this->joints.size() / 2)
    {
      this->leftJointStates.name[i] = this->jointNames[i];
    }
    else
    {
      unsigned j = i - this->joints.size() / 2;
      this->rightJointStates.name[j] = this->jointNames[i];
    }
  }

  this->jointCommands.name.resize(this->joints.size());
  this->jointCommands.position.resize(this->joints.size());
  this->jointCommands.velocity.resize(this->joints.size());
  this->jointCommands.effort.resize(this->joints.size());
  this->jointCommands.kp_position.resize(this->joints.size());
  this->jointCommands.ki_position.resize(this->joints.size());
  this->jointCommands.kd_position.resize(this->joints.size());
  this->jointCommands.kp_velocity.resize(this->joints.size());
  this->jointCommands.i_effort_min.resize(this->joints.size());
  this->jointCommands.i_effort_max.resize(this->joints.size());

  for (unsigned i = 0; i < this->joints.size(); ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].q_i = 0;
    this->errorTerms[i].qd_p = 0;
    this->jointCommands.name[i] = this->joints[i]->GetScopedName();
    this->jointCommands.position[i] = 0;
    this->jointCommands.velocity[i] = 0;
    this->jointCommands.effort[i] = 0;
    this->jointCommands.kp_position[i] = 0;
    this->jointCommands.ki_position[i] = 0;
    this->jointCommands.kd_position[i] = 0;
    this->jointCommands.kp_velocity[i] = 0;
    this->jointCommands.i_effort_min[i] = 0;
    this->jointCommands.i_effort_max[i] = 0;
  }

  // Get imu link
  this->leftImuLink = this->model->GetLink(this->leftImuLinkName);
  if (!this->leftImuLink)
    gzerr << this->leftImuLinkName << " not found\n";
  else
  {
    // initialize imu reference pose
    this->leftImuReferencePose = this->leftImuLink->GetWorldPose();
    this->leftImuLastLinearVel = leftImuReferencePose.rot.RotateVector(
      this->leftImuLink->GetWorldLinearVel());
  }

  this->rightImuLink = this->model->GetLink(this->rightImuLinkName);
  if (!this->rightImuLink)
    gzerr << this->rightImuLinkName << " not found\n";
  else
  {
    // initialize imu reference pose
    this->rightImuReferencePose = this->rightImuLink->GetWorldPose();
    this->rightImuLastLinearVel = rightImuReferencePose.rot.RotateVector(
      this->rightImuLink->GetWorldLinearVel());
  }

  // \todo: add ros topic / service to reset imu (imuReferencePose, etc.)

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&FloatingSandiaHandPlugin::DeferredLoad, this));
}

// helper function to save some typing
void FloatingSandiaHandPlugin::CopyVectorIfValid(const std::vector<double> &from,
                                         std::vector<double> &to,
                                         const unsigned joint_offset)
{
  if (joint_offset != 0 && joint_offset != to.size() / 2)
    return;  // get outta here, it's all over
  if (!from.size() || from.size() != to.size() / 2)
    return;
  for (size_t i = 0; i < from.size(); i++)
    to[i + joint_offset] = from[i];
}

////////////////////////////////////////////////////////////////////////////////
// Set Joint Commands
void FloatingSandiaHandPlugin::SetJointCommands(
  const osrf_msgs::JointCommands::ConstPtr &_msg,
  const unsigned ofs)  // ofs = joint offset
{
  // this implementation does not check the ordering of the joints. they must
  // agree with the structure initialized above!
  CopyVectorIfValid(_msg->position, this->jointCommands.position, ofs);
  CopyVectorIfValid(_msg->velocity, this->jointCommands.velocity, ofs);
  CopyVectorIfValid(_msg->effort, this->jointCommands.effort, ofs);
  CopyVectorIfValid(_msg->kp_position, this->jointCommands.kp_position, ofs);
  CopyVectorIfValid(_msg->ki_position, this->jointCommands.ki_position, ofs);
  CopyVectorIfValid(_msg->kd_position, this->jointCommands.kd_position, ofs);
  CopyVectorIfValid(_msg->kp_velocity, this->jointCommands.kp_velocity, ofs);
  CopyVectorIfValid(_msg->i_effort_min, this->jointCommands.i_effort_min, ofs);
  CopyVectorIfValid(_msg->i_effort_max, this->jointCommands.i_effort_max, ofs);
}


////////////////////////////////////////////////////////////////////////////////
// Load the controller
void FloatingSandiaHandPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // pull down controller parameters; they should be on the param server by now
  const int NUM_SIDES = 2, NUM_FINGERS = 4, NUM_FINGER_JOINTS = 3;
  const char *sides[NUM_SIDES] = {"left", "right"};
  for (int side = 0; side < NUM_SIDES; side++)
  {
    for (int finger = 0; finger < NUM_FINGERS; finger++)
    {
      for (int joint = 0; joint < NUM_FINGER_JOINTS; joint++)
      {
        char joint_ns[200] = "";
        snprintf(joint_ns, sizeof(joint_ns), "sandia_hands/gains/%s_f%d_j%d/",
                 sides[side], finger, joint);
        // this is so ugly
        double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
        string p_str = string(joint_ns)+"p";
        string i_str = string(joint_ns)+"i";
        string d_str = string(joint_ns)+"d";
        string i_clamp_str = string(joint_ns)+"i_clamp";
        if (!this->rosNode->getParam(p_str, p_val) ||
            !this->rosNode->getParam(i_str, i_val) ||
            !this->rosNode->getParam(d_str, d_val) ||
            !this->rosNode->getParam(i_clamp_str, i_clamp_val))
        {
          ROS_ERROR("couldn't find a param for %s", joint_ns);
          continue;
        }
        int joint_idx = side * (NUM_FINGERS * NUM_FINGER_JOINTS) +
                        finger * NUM_FINGER_JOINTS +
                        joint;
        this->jointCommands.kp_position[joint_idx]  =  p_val;
        this->jointCommands.ki_position[joint_idx]  =  i_val;
        this->jointCommands.kd_position[joint_idx]  =  d_val;
        this->jointCommands.i_effort_min[joint_idx] = -i_clamp_val;
        this->jointCommands.i_effort_max[joint_idx] =  i_clamp_val;
      }
    }
  }

  // ROS Controller API

  // ros publication / subscription
  /// brief broadcasts the robot states
  this->pubLeftJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "sandia_hands/l_hand/joint_states", 10);
  this->pubRightJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
    "sandia_hands/r_hand/joint_states", 10);

  // ros topic subscriptions
  ros::SubscribeOptions jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "sandia_hands/l_hand/joint_commands", 100,
    boost::bind(&FloatingSandiaHandPlugin::SetJointCommands, this, _1, 0),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands[0] = this->rosNode->subscribe(jointCommandsSo);
  jointCommandsSo =
    ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
    "sandia_hands/r_hand/joint_commands", 100,
    boost::bind(&FloatingSandiaHandPlugin::SetJointCommands, this, _1, 12),
    ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands[1] = this->rosNode->subscribe(jointCommandsSo);

  // publish imu data
  this->pubLeftImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/l_hand/imu", 10);
  this->pubRightImu =
    this->rosNode->advertise<sensor_msgs::Imu>(
      "sandia_hands/r_hand/imu", 10);

  // initialize status pub time
  this->lastStatusTime = this->world->GetSimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(
    boost::bind(&FloatingSandiaHandPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&FloatingSandiaHandPlugin::UpdateStates, this));
}

void FloatingSandiaHandPlugin::UpdateStates()
{
  common::Time curTime = this->world->GetSimTime();
  /// @todo:  robot internals
  /// self diagnostics, damages, etc.

  if (curTime > this->lastControllerUpdateTime)
  {
    // get imu data from imu link
    if (curTime > this->lastImuTime)
    {
      if (this->leftImuLink)
      {
        // Get imuLnk Pose/Orientation
        math::Pose leftImuPose = this->leftImuLink->GetWorldPose();
        math::Vector3 leftImuLinearVel = leftImuPose.rot.RotateVector(
          this->leftImuLink->GetWorldLinearVel());

        sensor_msgs::Imu leftImuMsg;
        leftImuMsg.header.frame_id = this->leftImuLinkName;
        leftImuMsg.header.stamp = ros::Time(curTime.Double());

        // compute angular rates
        {
          // get world twist and convert to local frame
          math::Vector3 wLocal = leftImuPose.rot.RotateVector(
            this->leftImuLink->GetWorldAngularVel());
          leftImuMsg.angular_velocity.x = wLocal.x;
          leftImuMsg.angular_velocity.y = wLocal.y;
          leftImuMsg.angular_velocity.z = wLocal.z;
        }

        // compute acceleration
        {
          math::Vector3 accel = leftImuLinearVel - this->leftImuLastLinearVel;
          double leftImuDdx = accel.x;
          double leftImuDdy = accel.y;
          double leftImuDdz = accel.z;

          leftImuMsg.linear_acceleration.x = leftImuDdx;
          leftImuMsg.linear_acceleration.y = leftImuDdy;
          leftImuMsg.linear_acceleration.z = leftImuDdz;

          this->leftImuLastLinearVel = leftImuLinearVel;
        }

        // compute orientation
        {
          // Get IMU rotation relative to Initial IMU Reference Pose
          math::Quaternion leftImuRot =
            leftImuPose.rot * this->leftImuReferencePose.rot.GetInverse();

          leftImuMsg.orientation.x = leftImuRot.x;
          leftImuMsg.orientation.y = leftImuRot.y;
          leftImuMsg.orientation.z = leftImuRot.z;
          leftImuMsg.orientation.w = leftImuRot.w;
        }

        this->pubLeftImu.publish(leftImuMsg);
      }

      if (this->rightImuLink)
      {
        // Get imuLnk Pose/Orientation
        math::Pose rightImuPose = this->rightImuLink->GetWorldPose();
        math::Vector3 rightImuLinearVel = rightImuPose.rot.RotateVector(
          this->rightImuLink->GetWorldLinearVel());

        sensor_msgs::Imu rightImuMsg;
        rightImuMsg.header.frame_id = this->rightImuLinkName;
        rightImuMsg.header.stamp = ros::Time(curTime.Double());

        // compute angular rates
        {
          // get world twist and convert to local frame
          math::Vector3 wLocal = rightImuPose.rot.RotateVector(
            this->rightImuLink->GetWorldAngularVel());
          rightImuMsg.angular_velocity.x = wLocal.x;
          rightImuMsg.angular_velocity.y = wLocal.y;
          rightImuMsg.angular_velocity.z = wLocal.z;
        }

        // compute acceleration
        {
          math::Vector3 accel = rightImuLinearVel - this->rightImuLastLinearVel;
          double rightImuDdx = accel.x;
          double rightImuDdy = accel.y;
          double rightImuDdz = accel.z;

          rightImuMsg.linear_acceleration.x = rightImuDdx;
          rightImuMsg.linear_acceleration.y = rightImuDdy;
          rightImuMsg.linear_acceleration.z = rightImuDdz;

          this->rightImuLastLinearVel = rightImuLinearVel;
        }

        // compute orientation
        {
          // Get IMU rotation relative to Initial IMU Reference Pose
          math::Quaternion rightImuRot =
            rightImuPose.rot * this->rightImuReferencePose.rot.GetInverse();

          rightImuMsg.orientation.x = rightImuRot.x;
          rightImuMsg.orientation.y = rightImuRot.y;
          rightImuMsg.orientation.z = rightImuRot.z;
          rightImuMsg.orientation.w = rightImuRot.w;
        }

        this->pubRightImu.publish(rightImuMsg);
      }

      // update time
      this->lastImuTime = curTime.Double();
    }

    // populate FromRobot from robot
    this->leftJointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->rightJointStates.header.stamp = this->leftJointStates.header.stamp;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      if (i < this->joints.size() / 2)
      {
        this->leftJointStates.position[i] =
          this->joints[i]->GetAngle(0).Radian();
        this->leftJointStates.velocity[i] = this->joints[i]->GetVelocity(0);
        // better to use GetForceTorque dot joint axis
        this->leftJointStates.effort[i] = this->joints[i]->GetForce(0);
      }
      else
      {
        unsigned j = i - this->joints.size() / 2;
        this->rightJointStates.position[j] =
          this->joints[i]->GetAngle(0).Radian();
        this->rightJointStates.velocity[j] = this->joints[i]->GetVelocity(0);
        this->rightJointStates.effort[j] = this->joints[i]->GetForce(0);
      }
    }
    this->pubLeftJointStates.publish(this->leftJointStates);
    this->pubRightJointStates.publish(this->rightJointStates);

    double dt = (curTime - this->lastControllerUpdateTime).Double();

    /// update pid with feedforward force
    double position;
    double velocity;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      if (i < this->joints.size() / 2)
      {
        position = this->leftJointStates.position[i];
        velocity = this->leftJointStates.velocity[i];
      }
      else
      {
        unsigned j = i - this->joints.size() / 2;
        position = this->rightJointStates.position[j];
        velocity = this->rightJointStates.velocity[j];
      }

      double q_p =
         this->jointCommands.position[i] - position;

      if (!math::equal(dt, 0.0))
        this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

      this->errorTerms[i].q_p = q_p;

      this->errorTerms[i].qd_p =
         this->jointCommands.velocity[i] - velocity;

      this->errorTerms[i].q_i = math::clamp(
        this->errorTerms[i].q_i + dt * this->errorTerms[i].q_p,
        static_cast<double>(this->jointCommands.i_effort_min[i]),
        static_cast<double>(this->jointCommands.i_effort_max[i]));

      // use gain params to compute force cmd
      double force = this->jointCommands.kp_position[i] *
                     this->errorTerms[i].q_p +
                     this->jointCommands.kp_velocity[i] *
                     this->errorTerms[i].qd_p +
                     this->jointCommands.ki_position[i] *
                     this->errorTerms[i].q_i +
                     this->jointCommands.kd_position[i] *
                     this->errorTerms[i].d_q_p_dt +
                     this->jointCommands.effort[i];

      this->joints[i]->SetForce(0, force);
    }
    this->lastControllerUpdateTime = curTime;
  }
}

void FloatingSandiaHandPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}
