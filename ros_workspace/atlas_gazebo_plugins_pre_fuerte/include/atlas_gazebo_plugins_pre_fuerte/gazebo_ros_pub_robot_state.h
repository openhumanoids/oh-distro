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

#ifndef GAZEBO_ROS_PUB_ROBOT_STATE_HH
#define GAZEBO_ROS_PUB_ROBOT_STATE_HH

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo_msgs/WorldState.h>

#include <ros/ros.h>

#include <map>
#include <vector>
#include <string>

#include <boost/thread.hpp>

// Topics
#include <atlas_gazebo_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo Dynamic Plugins
/// @{
/** \defgroup GazeboRosPubWorldState

  \brief A sample gazebo dynamic plugin
  
  This is a gazebo controller that does nothing

  Example Usage:
  \verbatim
    <model:physical name="robot_model1">

      <controller:gazebo_ros_pub_robot_state name="gazebo_ros_pub_robot_controller" plugin="libgazebo_ros_pub_robot_state.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>1000.0</updateRate>
      </controller:gazebo_ros_pub_robot_state>

      <xyz>0.0 0.0 0.02</xyz>
      <rpy>0.0 0.0 0.0 </rpy>

      <!-- a box -->
      <body:box name="test_block">
          <massMatrix>true</massMatrix>
          <mass>1000</mass>
          <ixx>100</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>100</iyy>
          <iyz>0</iyz>
          <izz>100</izz>
          <cx>0</cx>
          <cy>0</cy>
          <cz>0</cz>
          <xyz>0 0 0.002</xyz>
          <rpy>0 -0 0</rpy>
          <geom:box name="test_block_collision_geom">
              <xyz>0 0 10</xyz>
              <rpy>0 0 0</rpy>
              <size>20 20 20</size>
              <visual>
                  <xyz>0 0 0</xyz>
                  <rpy>0 -0 0</rpy>
                  <scale>20 20 20</scale>
                  <mesh>unit_box</mesh>
                  <material>Gazebo/GrassFloor</material>
              </visual>
          </geom:box>
      </body:box>

    </model:physical>
  \endverbatim
 
\{
*/

class GazeboRosPubRobotState : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosPubRobotState(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosPubRobotState();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief: keep track of number of connections
  private: int robotStateConnectCount;
  private: void RobotStateConnect();
  private: void RobotStateDisconnect();

  /// \brief: Message for sending world state
  //private: gazebo_msgs::WorldState worldStateMsg;
  private: atlas_gazebo_msgs::RobotState robotStateMsg;

  /// \bridf: parent should be a model
  private: gazebo::Model* parent_model_;
  private: std::map<std::string, gazebo::Joint* > joints_;

  /// \bridf: ros node handle and publisher
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;

  /// \brief frame transform name, should match link name
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread callback_queue_thread_;

};

/** \} */
/// @}

}
#endif

