/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2012 Open Source Robotics Foundation
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
 * Desc: Plugin to allow development shortcuts for VRC competition.
 * Author: John Hsu and Steven Peters
 * Date: December 2012
 */
#ifndef GAZEBO_VRC_PLUGIN_HH
#define GAZEBO_VRC_PLUGIN_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

typedef actionlib::SimpleActionClient<
  control_msgs::FollowJointTrajectoryAction > TrajClient;
namespace gazebo
{
  class VRCPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: VRCPlugin();

    /// \brief Destructor
    public: virtual ~VRCPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to parent world.
    /// \param[in] _sdf Pointer to sdf element.
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller on every World::Update
    private: void UpdateStates();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   List of available actions                                            //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief Sets Atlas planar navigational command velocity
    /// \param[in] _cmd A Vector3, where:
    ///   - x is the desired forward linear velocity, positive is robot-forward
    ///     and negative is robot-back.
    ///   - y is the desired lateral linear velocity, positive is robot-left
    ///     and negative is robot-right.
    ///   - z is the desired heading angular velocity, positive makes
    ///     the robot turn left, and negative makes the robot turn right
    public: void SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd);

    /// \brief sets robot's absolute world pose
    /// \param[in] _cmd Pose command for the robot
    public: void SetRobotPose(const geometry_msgs::Pose::ConstPtr &_cmd);

    /// \brief sets robot's joint positions
    /// \param[in] _cmd configuration made of sensor_msgs::JointState message
    /// \todo: not yet implemented
    public: void SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
                                       &/*_cmd*/);

    /// \brief sets robot mode via ros topic
    /// \sa SetRobotMode(const std::string &_str)
    public: void SetRobotModeTopic(const std_msgs::String::ConstPtr &_str);

    /// \brief sets robot mode
    /// \param[in] _str sets robot mode by a string.  Supported modes are:
    ///  - "no_gravity" Gravity disabled for the robot.
    ///  - "nominal" Nominal "normal" physics.
    ///  - "pinned" Robot is pinned to inertial world by the pelvis.
    ///  - "feet" same as no_gravity except for r_foot and l_foot links.
    public: void SetRobotMode(const std::string &_str);


    /// \brief Robot Vehicle Interaction, put robot in driver's seat.
    /// \param[in] _pose Relative pose offset, Pose()::Zero provides default
    ///                 behavior.
    public: void RobotEnterCar(const geometry_msgs::Pose::ConstPtr &_pose);

    /// \brief Robot Vehicle Interaction, put robot outside driver's side door.
    /// \param[in] _pose Relative pose offset, Pose()::Zero provides default
    ///                 behavior.
    public: void RobotExitCar(const geometry_msgs::Pose::ConstPtr &_pose);

    /// \brief Cheats to teleport fire hose to hand and make a fixed joint
    /// \param[in] _cmd Relative pose offset between the link and the hand.
    public: void RobotGrabFireHose(const geometry_msgs::Pose::ConstPtr &_cmd);

    /// \brief remove the fixed joint between robot hand link and fire hose.
    /// \param[in] _cmd not used.
    public: void RobotReleaseLink(const geometry_msgs::Pose::ConstPtr &_cmd);


    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Generic tools for manipulating models                                //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    /// \brief sets robot's absolute world pose
    /// \param[in] _pinLink Link to pin to the world
    /// \param[in] _pose sets the _pinLink world pose before pinning
    /// \param[in] _jp joint and positions for model configuration (TODO)
    /// \param[out] _pinJoint a pin Joint is created
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose,
                          const std::map<std::string, double> &/*_jp*/);

    /// \brief sets robot's absolute world pose
    /// \param[in] _pinLink Link to pin to the world
    /// \param[in] _pose sets the _pinLink world pose before pinning
    /// \param[out] _pinJoint a pin Joint is created
    private: void Teleport(const physics::LinkPtr &_pinLink,
                          physics::JointPtr &_pinJoint,
                          const math::Pose &_pose);

    /// \brief add a constraint between 2 links
    /// \param[in] _world a pointer to the current World
    /// \param[in] _model a pointer to the Model the new Joint will be under
    /// \param[in] _link1 parent link in the new Joint
    /// \param[in] _link2 child link in the new Joint
    /// \param[in] _type string specifying joint type
    /// \param[in] _anchor a Vector3 anchor offset of the new joint
    /// \param[in] _axis Vector3 containing xyz axis of the new joint
    /// \param[in] _upper upper linit of the new joint
    /// \param[in] _lower lower linit of the new joint
    /// \return Joint created between _link1 and _link2 under _model.
    private: physics::JointPtr AddJoint(physics::WorldPtr _world,
                                        physics::ModelPtr _model,
                                        physics::LinkPtr _link1,
                                        physics::LinkPtr _link2,
                                        std::string _type,
                                        math::Vector3 _anchor,
                                        math::Vector3 _axis,
                                        double _upper, double _lower);

    /// \brief Remove a joint.
    /// \param[in] _joint Joint to remove.
    private: void RemoveJoint(physics::JointPtr &_joint);

    /// \brief setup Robot ROS publication and sbuscriptions for the Robot
    /// These ros api describes Robot only actions
    private: void LoadRobotROSAPI();

    /// \brief setup ROS publication and sbuscriptions for VRC
    /// These ros api describes interactions between different models
    /// /atlas/cmd_vel - in pinned mode, the robot teleports based on
    ///                      messages from the cmd_vel
    private: void LoadVRCROSAPI();

    /// \brief check and spawn screw joint to simulate threads
    /// if links are aligned
    private: void CheckThreadStart();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief ROS callback queue thread
    private: void ROSQueueThread();

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Atlas properties and states                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Robot
    {
      /// \brief Load the atlas portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr model;
      private: physics::LinkPtr pinLink;
      private: physics::JointPtr pinJoint;

      /// \brief keep initial pose of robot to prevent z-drifting when
      /// teleporting the robot.
      private: math::Pose initialPose;

      /// \brief Pose of robot relative to vehicle.
      private: math::Pose vehicleRelPose;

      /// \brief Robot configuration when inside of vehicle.
      private: std::map<std::string, double> inVehicleConfiguration;

      /// \brief Flag to keep track of start-up 'harness' on the robot.
      private: bool startupHarness;

      /// \brief flag for successful initialization of atlas
      private: bool isInitialized;

      private: ros::Subscriber subTrajectory;
      private: ros::Subscriber subPose;
      private: ros::Subscriber subConfiguration;
      private: ros::Subscriber subMode;

      friend class VRCPlugin;

    } atlas;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Vehicle properties and states                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class Vehicle
    {
      /// \brief Load the drc vehicle portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr model;
      private: math::Pose initialPose;
      private: physics::LinkPtr seatLink;

      /// \brief flag for successful initialization of vehicle
      private: bool isInitialized;

      friend class VRCPlugin;
    } drcVehicle;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   DRC Fire Hose (and Standpipe)                                        //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class FireHose
    {
      /// \brief set initial configuration of the fire hose link
      private: void SetInitialConfiguration()
      {
        // for (unsigned int i = 0; i < this->fireHoseJoints.size(); ++i)
        //   gzerr << "joint [" << this->fireHoseJoints[i]->GetName() << "]\n";
        // for (unsigned int i = 0; i < this->links.size(); ++i)
        //   gzerr << "link [" << this->links[i]->GetName() << "]\n";
        this->fireHoseModel->SetWorldPose(this->initialFireHosePose);
        this->fireHoseJoints[fireHoseJoints.size()-4]->SetAngle(0, -M_PI/4.0);
        this->fireHoseJoints[fireHoseJoints.size()-2]->SetAngle(0, -M_PI/4.0);
      }

      /// \brief Load the drc_fire_hose portion of plugin.
      /// \param[in] _parent Pointer to parent world.
      /// \param[in] _sdf Pointer to sdf element.
      private: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

      private: physics::ModelPtr fireHoseModel;
      private: physics::ModelPtr standpipeModel;

      /// joint for pinning a link to the world
      private: physics::JointPtr fixedJoint;

      /// joints and links
      private: physics::Joint_V fireHoseJoints;
      private: physics::Link_V fireHoseLinks;
      /// screw joint
      private: physics::JointPtr screwJoint;
      private: double threadPitch;

      /// Pointer to the update event connection
      private: event::ConnectionPtr updateConnection;

      private: physics::LinkPtr couplingLink;
      private: physics::LinkPtr spoutLink;
      private: math::Pose couplingRelativePose;
      private: math::Pose initialFireHosePose;

      /// \brief flag for successful initialization of fire hose, standpipe
      private: bool isInitialized;

      friend class VRCPlugin;
    } drcFireHose;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Joint Trajectory Controller                                          //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: class JointTrajectory
    {
      /// \brief Constructor, note atlas_controller is the name
      /// of the controller loaded from yaml
      public: JointTrajectory() 
      {
        // tell the action client that we want to spin a thread by default
        this->clientTraj = new TrajClient(
          "/atlas_controller/follow_joint_trajectory", true);

      }

      /// \brief Destructor, clean up the action client
      public: ~JointTrajectory()
      {
        delete this->clientTraj;
      }

      /// \brief Sends the command to start a given trajectory
      public: void sendTrajectory(control_msgs::FollowJointTrajectoryGoal _goal)
      {
        // When to start the trajectory: 1s from now
        _goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

        this->clientTraj->sendGoal(_goal);
      }

      /// \brief Generates a simple trajectory
      /// Note that this trajectory contains two waypoints, joined together
      /// as a single trajectory. Alternatively, each of these waypoints could
      /// be in its own trajectory - a trajectory can have one or more waypoints
      /// depending on the desired application.
      public: control_msgs::FollowJointTrajectoryGoal seatingConfiguration()
      {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("l_leg_uhz");
        goal.trajectory.joint_names.push_back("l_leg_mhx");
        goal.trajectory.joint_names.push_back("l_leg_lhy");
        goal.trajectory.joint_names.push_back("l_leg_kny");
        goal.trajectory.joint_names.push_back("l_leg_uay");
        goal.trajectory.joint_names.push_back("l_leg_lax");

        goal.trajectory.joint_names.push_back("r_leg_uhz");
        goal.trajectory.joint_names.push_back("r_leg_mhx");
        goal.trajectory.joint_names.push_back("r_leg_lhy");
        goal.trajectory.joint_names.push_back("r_leg_kny");
        goal.trajectory.joint_names.push_back("r_leg_uay");
        goal.trajectory.joint_names.push_back("r_leg_lax");

        goal.trajectory.joint_names.push_back("l_arm_usy");
        goal.trajectory.joint_names.push_back("l_arm_shx");
        goal.trajectory.joint_names.push_back("l_arm_ely");
        goal.trajectory.joint_names.push_back("l_arm_elx");
        goal.trajectory.joint_names.push_back("l_arm_uwy");
        goal.trajectory.joint_names.push_back("l_arm_mwx");

        goal.trajectory.joint_names.push_back("r_arm_usy");
        goal.trajectory.joint_names.push_back("r_arm_shx");
        goal.trajectory.joint_names.push_back("r_arm_ely");
        goal.trajectory.joint_names.push_back("r_arm_elx");
        goal.trajectory.joint_names.push_back("r_arm_uwy");
        goal.trajectory.joint_names.push_back("r_arm_mwx");

        goal.trajectory.joint_names.push_back("neck_ay"  );
        goal.trajectory.joint_names.push_back("back_lbz" );
        goal.trajectory.joint_names.push_back("back_mby" );
        goal.trajectory.joint_names.push_back("back_ubx" );

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(28);
        goal.trajectory.points[ind].positions[0]  =   0.00;
        goal.trajectory.points[ind].positions[1]  =   0.00;
        goal.trajectory.points[ind].positions[2]  =  -1.70;
        goal.trajectory.points[ind].positions[3]  =   1.80;
        goal.trajectory.points[ind].positions[4]  =  -0.10;
        goal.trajectory.points[ind].positions[5]  =   0.00;

        goal.trajectory.points[ind].positions[6]  =   0.00;
        goal.trajectory.points[ind].positions[7]  =   0.00;
        goal.trajectory.points[ind].positions[8]  =  -1.70;
        goal.trajectory.points[ind].positions[9]  =   1.80;
        goal.trajectory.points[ind].positions[10] =  -0.10;
        goal.trajectory.points[ind].positions[11] =   0.00;

        goal.trajectory.points[ind].positions[12] =  -1.60;
        goal.trajectory.points[ind].positions[13] =  -1.60;
        goal.trajectory.points[ind].positions[14] =   0.00;
        goal.trajectory.points[ind].positions[15] =   0.00;
        goal.trajectory.points[ind].positions[16] =   0.00;
        goal.trajectory.points[ind].positions[17] =   0.00;

        goal.trajectory.points[ind].positions[18] =   1.60;
        goal.trajectory.points[ind].positions[19] =   1.60;
        goal.trajectory.points[ind].positions[20] =   0.00;
        goal.trajectory.points[ind].positions[21] =   0.00;
        goal.trajectory.points[ind].positions[22] =   0.00;
        goal.trajectory.points[ind].positions[23] =   0.00;

        goal.trajectory.points[ind].positions[24] =   0.00;
        goal.trajectory.points[ind].positions[25] =   0.00;
        goal.trajectory.points[ind].positions[26] =   0.00;
        goal.trajectory.points[ind].positions[27] =   0.00;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }

        // tolerances
        for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
        {
          control_msgs::JointTolerance jt;
          jt.name = goal.trajectory.joint_names[j];
          jt.position = 1000;
          jt.velocity = 1000;
          jt.acceleration = 1000;
          goal.goal_tolerance.push_back(jt);
        }

        goal.goal_time_tolerance.sec = 10;
        goal.goal_time_tolerance.nsec = 0;

        //we are done; return the goal
        return goal;
      }

      /// \brief Generates a simple trajectory for standing configuration.
      public: control_msgs::FollowJointTrajectoryGoal standingConfiguration()
      {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("l_leg_uhz");
        goal.trajectory.joint_names.push_back("l_leg_mhx");
        goal.trajectory.joint_names.push_back("l_leg_lhy");
        goal.trajectory.joint_names.push_back("l_leg_kny");
        goal.trajectory.joint_names.push_back("l_leg_uay");
        goal.trajectory.joint_names.push_back("l_leg_lax");

        goal.trajectory.joint_names.push_back("r_leg_uhz");
        goal.trajectory.joint_names.push_back("r_leg_mhx");
        goal.trajectory.joint_names.push_back("r_leg_lhy");
        goal.trajectory.joint_names.push_back("r_leg_kny");
        goal.trajectory.joint_names.push_back("r_leg_uay");
        goal.trajectory.joint_names.push_back("r_leg_lax");

        goal.trajectory.joint_names.push_back("l_arm_usy");
        goal.trajectory.joint_names.push_back("l_arm_shx");
        goal.trajectory.joint_names.push_back("l_arm_ely");
        goal.trajectory.joint_names.push_back("l_arm_elx");
        goal.trajectory.joint_names.push_back("l_arm_uwy");
        goal.trajectory.joint_names.push_back("l_arm_mwx");

        goal.trajectory.joint_names.push_back("r_arm_usy");
        goal.trajectory.joint_names.push_back("r_arm_shx");
        goal.trajectory.joint_names.push_back("r_arm_ely");
        goal.trajectory.joint_names.push_back("r_arm_elx");
        goal.trajectory.joint_names.push_back("r_arm_uwy");
        goal.trajectory.joint_names.push_back("r_arm_mwx");

        goal.trajectory.joint_names.push_back("neck_ay"  );
        goal.trajectory.joint_names.push_back("back_lbz" );
        goal.trajectory.joint_names.push_back("back_mby" );
        goal.trajectory.joint_names.push_back("back_ubx" );

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(28);
        goal.trajectory.points[ind].positions[0]  =   0.00;
        goal.trajectory.points[ind].positions[1]  =   0.00;
        goal.trajectory.points[ind].positions[2]  =   0.00;
        goal.trajectory.points[ind].positions[3]  =   0.00;
        goal.trajectory.points[ind].positions[4]  =   0.00;
        goal.trajectory.points[ind].positions[5]  =   0.00;

        goal.trajectory.points[ind].positions[6]  =   0.00;
        goal.trajectory.points[ind].positions[7]  =   0.00;
        goal.trajectory.points[ind].positions[8]  =   0.00;
        goal.trajectory.points[ind].positions[9]  =   0.00;
        goal.trajectory.points[ind].positions[10] =   0.00;
        goal.trajectory.points[ind].positions[11] =   0.00;

        goal.trajectory.points[ind].positions[12] =   0.00;
        goal.trajectory.points[ind].positions[13] =  -1.60;
        goal.trajectory.points[ind].positions[14] =   0.00;
        goal.trajectory.points[ind].positions[15] =   0.00;
        goal.trajectory.points[ind].positions[16] =   0.00;
        goal.trajectory.points[ind].positions[17] =   0.00;

        goal.trajectory.points[ind].positions[18] =   0.00;
        goal.trajectory.points[ind].positions[19] =   1.60;
        goal.trajectory.points[ind].positions[20] =   0.00;
        goal.trajectory.points[ind].positions[21] =   0.00;
        goal.trajectory.points[ind].positions[22] =   0.00;
        goal.trajectory.points[ind].positions[23] =   0.00;

        goal.trajectory.points[ind].positions[24] =   0.00;
        goal.trajectory.points[ind].positions[25] =   0.00;
        goal.trajectory.points[ind].positions[26] =   0.00;
        goal.trajectory.points[ind].positions[27] =   0.00;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Velocities
        goal.trajectory.points[ind].velocities.resize(28);
        for (size_t j = 0; j < 28; ++j)
        {
          goal.trajectory.points[ind].velocities[j] = 0.0;
        }

        // tolerances
        for (unsigned j = 0; j < goal.trajectory.joint_names.size(); ++j)
        {
          control_msgs::JointTolerance jt;
          jt.name = goal.trajectory.joint_names[j];
          jt.position = 1000;
          jt.velocity = 1000;
          jt.acceleration = 1000;
          goal.goal_tolerance.push_back(jt);
        }

        goal.goal_time_tolerance.sec = 10;
        goal.goal_time_tolerance.nsec = 0;

        //we are done; return the goal
        return goal;
      }

      /// \brief Get state of the simple action client
      /// \return the current state of the action
      public: actionlib::SimpleClientGoalState getState()
      {
        return this->clientTraj->getState();
      }

      // Action client for the joint trajectory action 
      // used to trigger the arm movement action
      public: TrajClient* clientTraj;

    } jointTrajectoryController;

    ////////////////////////////////////////////////////////////////////////////
    //                                                                        //
    //   Private variables                                                    //
    //                                                                        //
    ////////////////////////////////////////////////////////////////////////////
    private: bool warpRobotWithCmdVel;
    private: double lastUpdateTime;
    private: geometry_msgs::Twist robotCmdVel;

    /// \brief fix robot butt to vehicle for efficiency
    // public: std::pair<physics::LinkPtr, physics::LinkPtr> vehicleRobot;
    public: physics::JointPtr vehicleRobotJoint;

    /// \brief Pointer to parent world.
    private: physics::WorldPtr world;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // default ros stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueueThread;

    // ros subscribers for robot actions
    private: ros::Subscriber subRobotGrab;
    private: ros::Subscriber subRobotRelease;
    private: ros::Subscriber subRobotEnterCar;
    private: ros::Subscriber subRobotExitCar;
    private: physics::JointPtr grabJoint;

    // items below are used for deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;
  };
/** \} */
/// @}
}
#endif
