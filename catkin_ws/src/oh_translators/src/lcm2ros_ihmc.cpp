// Copyright 2015 Maurice Fallon, Vladimir Ivan

#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/drc/walking_plan_t.hpp"
#include "lcmtypes/drc/walking_plan_request_t.hpp"
#include "lcmtypes/drc/footstep_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/neck_pitch_t.hpp"
#include "lcmtypes/drc/scs_api_command_t.hpp"
#include "lcmtypes/drc/int64_stamped_t.hpp"

#include "lcmtypes/ihmc/com_height_packet_message_t.hpp"
#include "lcmtypes/ihmc/pause_command_message_t.hpp"
#include "lcmtypes/ihmc/hand_pose_packet_message_t.hpp"

#include <ihmc_msgs/FootstepDataListMessage.h>
#include <ihmc_msgs/ComHeightPacketMessage.h>
#include <ihmc_msgs/PauseCommandMessage.h>
#include <ihmc_msgs/HandPosePacketMessage.h>
#include <ihmc_msgs/ArmJointTrajectoryPacketMessage.h>
#include <ihmc_msgs/WholeBodyTrajectoryPacketMessage.h>
#include <ihmc_msgs/StopMotionPacketMessage.h>
#include <ihmc_msgs/HeadOrientationPacketMessage.h>
#include <ihmc_msgs/FootPosePacketMessage.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#define LEFT 0
#define RIGHT 1
#define MIN_SWING_HEIGHT 0.05
#define MAX_SWING_HEIGHT 0.3

enum class TrajectoryMode {wholeBody, leftArm, rightArm, bothArms}; // 0,1,2,3
std::vector<std::string> TrajectoryNames = {"Whole Body", "Left Arm", "Right Arm", "Both Arms"};


class LCM2ROS
{
public:
  LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string robotName_);
  ~LCM2ROS()
  {
  }

private:
  boost::shared_ptr<lcm::LCM> lcm_;
  ros::NodeHandle nh_;
  ros::NodeHandle* node_;
  std::string robotName_;
  boost::shared_ptr<ModelClient> model_;
  boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
  std::string chestLinkName_;

  // Parameters and Variables:
  double default_transfer_time_;
  double default_swing_time_;
  // Seconds to offset the plan so that the controller
  // can blend from the current desired joint position to the plan joint position
  // this was added to avoid controller jerks when starting short plans.
  double planDesiredOffset_;


  TrajectoryMode outputTrajectoryMode_;

  // Subscriptions and Publishes below ////////////////////////////
  void footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const drc::walking_plan_request_t* msg);
  void footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                  const drc::footstep_plan_t* msg);
  ros::Publisher walking_plan_pub_;
  ihmc_msgs::FootstepDataMessage convertFootStepToIHMC(const drc::footstep_t & drc_step);

  void comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                        const ihmc::com_height_packet_message_t* msg);
  ros::Publisher com_height_pub_;

  void pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                    const ihmc::pause_command_message_t* msg);
  void stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
  void stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
  ros::Publisher pause_pub_, stop_manip_pub_;

  void handPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                       const ihmc::hand_pose_packet_message_t* msg);
  ros::Publisher hand_pose_pub_;

  void robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg);
  ros::Publisher arm_joint_traj2_pub_, whole_body_trajectory_pub_;

  double getPlanTimeAtWaypoint(int64_t planUtime);
  void sendSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                         std::vector<std::string> input_joint_names, bool is_right);
  bool getSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                        std::vector<std::string> input_joint_names, bool is_right,
                        ihmc_msgs::ArmJointTrajectoryPacketMessage &m);
  bool getChestTrajectoryPlan(const drc::robot_plan_t* msg, std::vector<geometry_msgs::Quaternion> &m);

  void scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::scs_api_command_t* msg);
  ros::Publisher scs_api_pub_;

  void neckPitchHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::neck_pitch_t* msg);
  void headOrientationHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
  ros::Publisher neck_orientation_pub_;
  void lFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
  ros::Publisher lfoot_pose_pub_;
  void rFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
  ros::Publisher rfoot_pose_pub_;

  void ihmcControlModeCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::int64_stamped_t* msg);
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_in, ros::NodeHandle &nh_in, std::string robotName_in):
    lcm_(lcm_in), nh_(nh_in), robotName_(robotName_in)
{
  model_ = boost::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(), 0));
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(model_->getURDFString(), tree))
  {
    std::cerr << "ERROR: Failed to extract kdl tree from xml robot description" << std::endl;
    exit(-1);
  }
  fksolver_ = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(tree));

  if (robotName_.compare("atlas") == 0)  // Atlas has utorso
    chestLinkName_ = "utorso";
  else  // valkyrie has torso
    chestLinkName_ = "torso";
  std::cout << "Using "<< robotName_ << " so expecting chest link called " << chestLinkName_ << "\n";

  // Hard Coded Parameters:
  // Conservative values for real Valkyrie, using defaults used by IHMC
  default_transfer_time_ = 1.0;
  default_swing_time_ = 1.0;
  planDesiredOffset_ = 1.0;
  // Variable to set what part of a whole body plan gets passed through to Val:
  outputTrajectoryMode_ = TrajectoryMode::wholeBody;
  ROS_ERROR("LCM2ROS Controller TrajectoryMode: %s", TrajectoryNames.at( (int) outputTrajectoryMode_).c_str() );

  ////////////////// Subscriptions and Adverts //////////////////////
  // If pronto is running never send plans like this:
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST", &LCM2ROS::footstepPlanHandler, this);
  // COMMITTED_FOOTSTEP_PLAN is creating in Pronto frame and transformed into 
  // BDI/IHMC coordinate frame using BDI_ADJUSTED_FOOTSTEP_PLAN
  lcm_->subscribe("BDI_ADJUSTED_FOOTSTEP_PLAN", &LCM2ROS::footstepPlanBDIModeHandler, this);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListMessage>(
      "/ihmc_ros/" + robotName_ + "/control/footstep_list", 10);

  lcm_->subscribe("VAL_COMMAND_COM_HEIGHT", &LCM2ROS::comHeightHandler, this);
  com_height_pub_ = nh_.advertise<ihmc_msgs::ComHeightPacketMessage>("/ihmc_ros/" + robotName_ + "/control/com_height",
                                                                     10);

  lcm_->subscribe("VAL_COMMAND_PAUSE", &LCM2ROS::pauseHandler, this);
  lcm_->subscribe("STOP_WALKING", &LCM2ROS::stopHandler, this);  // from Director
  pause_pub_ = nh_.advertise<ihmc_msgs::PauseCommandMessage>("/ihmc_ros/" + robotName_ + "/control/pause_footstep_exec",
                                                             10);
  lcm_->subscribe("COMMITTED_PLAN_PAUSE", &LCM2ROS::stopManipHandler, this);  // from Director to stop manipulation plans
  stop_manip_pub_ = nh_.advertise<ihmc_msgs::StopMotionPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/stop_motion", 10);

  // robot plan messages now used, untested
  lcm_->subscribe("COMMITTED_ROBOT_PLAN", &LCM2ROS::robotPlanHandler, this);
  arm_joint_traj2_pub_ = nh_.advertise<ihmc_msgs::ArmJointTrajectoryPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/arm_joint_trajectory", 10);
  whole_body_trajectory_pub_ = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/whole_body_trajectory", 10);

  lcm_->subscribe("SCS_API_CONTROL", &LCM2ROS::scsAPIHandler, this);
  scs_api_pub_ = nh_.advertise<std_msgs::String>("/ihmc_ros/" + robotName_ + "/api_command", 10);

  lcm_->subscribe("DESIRED_NECK_PITCH", &LCM2ROS::neckPitchHandler, this);
  lcm_->subscribe("DESIRED_HEAD_ORIENTATION", &LCM2ROS::headOrientationHandler, this);
  neck_orientation_pub_ = nh_.advertise<ihmc_msgs::HeadOrientationPacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/head_orientation", 10);

  lcm_->subscribe("DESIRED_LEFT_FOOT_POSE", &LCM2ROS::lFootPoseHandler, this);
  lfoot_pose_pub_ = nh_.advertise<ihmc_msgs::FootPosePacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/foot_pose", 10);

  lcm_->subscribe("DESIRED_RIGHT_FOOT_POSE", &LCM2ROS::rFootPoseHandler, this);
  rfoot_pose_pub_ = nh_.advertise<ihmc_msgs::FootPosePacketMessage>(
      "/ihmc_ros/" + robotName_ + "/control/foot_pose", 10);

  // depreciated:
  // lcm_->subscribe("VAL_COMMAND_HAND_POSE",&LCM2ROS::handPoseHandler, this);
  // hand_pose_pub_ =  nh_.advertise<ihmc_msgs::HandPosePacketMessage>("/ihmc_ros/" + robotName_ + "/control/hand_pose",10);

  // Subscriptions that handle local variables:
  lcm_->subscribe("IHMC_CONTROL_MODE_COMMAND", &LCM2ROS::ihmcControlModeCommandHandler, this);

  node_ = new ros::NodeHandle();
}

ihmc_msgs::FootstepDataMessage LCM2ROS::convertFootStepToIHMC(const drc::footstep_t &drc_step) {
  ihmc_msgs::FootstepDataMessage ihmc_step;
  ihmc_step.robot_side = drc_step.is_right_foot;
  ihmc_step.location.x = drc_step.pos.translation.x;
  ihmc_step.location.y = drc_step.pos.translation.y;
  ihmc_step.location.z = drc_step.pos.translation.z;
  ihmc_step.orientation.w = drc_step.pos.rotation.w;
  ihmc_step.orientation.x = drc_step.pos.rotation.x;
  ihmc_step.orientation.y = drc_step.pos.rotation.y;
  ihmc_step.orientation.z = drc_step.pos.rotation.z;

  // Used values from footstepsdriver.py for Valkyrie version 2
  // foot_length = 0.21, foot_width = 0.11 (dimension of the sole))
  if (robotName_.compare("valkyrie") == 0)
  {
    double footsizeReduction = 0.04;
    double foot_length = 0.25 - footsizeReduction;
    double foot_width = 0.15 - footsizeReduction;
    // if (support_contact_groups == 0) we do not set the contact points because
    // a value of null will default to use the entire foot
    if (drc_step.params.support_contact_groups == 1)
    {
      ihmc_msgs::Point2dMessage point;
      point.x = 0.5 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = 0.5 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.166666667 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.166666667 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
    }
    else if (drc_step.params.support_contact_groups == 2)
    {
      ihmc_msgs::Point2dMessage point;
      point.x = 0.166666667 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = 0.166666667 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.5 * foot_length;
      point.y = -0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
      point.x = -0.5 * foot_length;
      point.y = 0.5 * foot_width;
      ihmc_step.predicted_contact_points.push_back(point);
    }
  }
  // else if atlas, always use the entire foot


  double max_relative_terrain_height = 0.0;
  ihmc_step.trajectory_type = ihmc_step.BASIC;

  if (drc_step.terrain_height.size() > 0) {
    assert(drc_step.terrain_height.size() == drc_step.terrain_path_dist.size());
    double starting_terrain_height = drc_step.terrain_height[0];
    max_relative_terrain_height = *std::max_element(drc_step.terrain_height.begin(), drc_step.terrain_height.end()) - starting_terrain_height;
    double swing_distance_in_plane = drc_step.terrain_path_dist[drc_step.terrain_path_dist.size() - 1] - drc_step.terrain_path_dist[0];
    if (swing_distance_in_plane > 0) {
      // Does the terrain rise significantly above a triangle that starts at the initial foot pose, rises to the maximum terrain height halfway through the swing, and finishes at the final foot pose? If so, do a high clearance step. 
      for (size_t i = 0; i < drc_step.terrain_height.size(); i++) {
        double expected_relative_terrain_height = max_relative_terrain_height * (1 - std::abs(drc_step.terrain_path_dist[i] - swing_distance_in_plane / 2.0) / (swing_distance_in_plane / 2.0));
        if (drc_step.terrain_height[i] > expected_relative_terrain_height + starting_terrain_height + 0.02) {
          ihmc_step.trajectory_type = ihmc_step.OBSTACLE_CLEARANCE;
          break;
        }
      }
    }
  }

  double swing_height = max_relative_terrain_height + drc_step.params.step_height;
  ihmc_step.swing_height = std::min(std::max(swing_height, MIN_SWING_HEIGHT),
                                    MAX_SWING_HEIGHT);

  return ihmc_step;
}

Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw)
{
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  // Post DRC Trails: replace these with Eigen's own conversions
  if (((roll == M_PI) && (pitch == 0)) && (yaw == 0))
  {
    return Eigen::Quaterniond(0, 1, 0, 0);
  }
  else if (((pitch == M_PI) && (roll == 0)) && (yaw == 0))
  {
    return Eigen::Quaterniond(0, 0, 1, 0);
  }
  else if (((yaw == M_PI) && (roll == 0)) && (pitch == 0))
  {
    return Eigen::Quaterniond(0, 0, 0, 1);
  }

  double sy = sin(yaw * 0.5);
  double cy = cos(yaw * 0.5);
  double sp = sin(pitch * 0.5);
  double cp = cos(pitch * 0.5);
  double sr = sin(roll * 0.5);
  double cr = cos(roll * 0.5);
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;
  return Eigen::Quaterniond(w, x, y, z);
}

void LCM2ROS::scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                            const drc::scs_api_command_t* msg)
{
  std_msgs::String rmsg;
  rmsg.data = msg->command;
  scs_api_pub_.publish(rmsg);
}

void LCM2ROS::footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                  const drc::walking_plan_request_t* msg)
{
  ROS_ERROR("LCM2ROS got WALKING_CONTROLLER_PLAN_REQUEST (non-pronto and drake mode)");

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transfer_time = default_transfer_time_;
  mout.swing_time = default_swing_time_;
  for (int i = 2; i < msg->footstep_plan.num_steps; i++)  // skip the first two standing steps
  {
    mout.footstep_data_list.push_back(convertFootStepToIHMC(msg->footstep_plan.footsteps[i]));
  }
  walking_plan_pub_.publish(mout);
}

void LCM2ROS::footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                                         const drc::footstep_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got BDI_ADJUSTED_FOOTSTEP_PLAN or COMMITTED_FOOTSTEP_PLAN (pronto and bdi mode)");

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transfer_time = default_transfer_time_;
  mout.swing_time = default_swing_time_;
  for (int i = 2; i < msg->num_steps; i++)  // skip the first two standing steps
  {
    mout.footstep_data_list.push_back(convertFootStepToIHMC(msg->footsteps[i]));
  }
  walking_plan_pub_.publish(mout);
}

void LCM2ROS::comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const ihmc::com_height_packet_message_t* msg)
{
  ROS_ERROR("LCM2ROS got com height");
  ihmc_msgs::ComHeightPacketMessage mout;
  mout.height_offset = msg->height_offset;
  com_height_pub_.publish(mout);
}

void LCM2ROS::pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                           const ihmc::pause_command_message_t* msg)
{
  ROS_ERROR("LCM2ROS got pause %d", static_cast<int>(msg->pause));
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = msg->pause;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got STOP_WALKING - sending pause=true");
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = true;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                               const drc::plan_control_t* msg)
{
  ROS_ERROR("LCM2ROS got COMMITTED_PLAN_PAUSE - sending manipulate stop");
  ihmc_msgs::StopMotionPacketMessage mout;
  mout.unique_id = msg->utime;
  stop_manip_pub_.publish(mout);
}

void LCM2ROS::handPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel,
                              const ihmc::hand_pose_packet_message_t* msg)
{
  ROS_ERROR("LCM2ROS got handPose packet");
  ihmc_msgs::HandPosePacketMessage mout;
  mout.robot_side = msg->robot_side;
  mout.to_home_position = msg->to_home_position;
  mout.trajectory_time = msg->trajectory_time;
  mout.joint_angles = msg->joint_angles;
  hand_pose_pub_.publish(mout);
}

void LCM2ROS::neckPitchHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::neck_pitch_t* msg)
{
  ROS_ERROR("LCM2ROS got desired neck pitch");
  ihmc_msgs::HeadOrientationPacketMessage mout;
  Eigen::Quaterniond quat = euler_to_quat(0, msg->pitch, 0);
  mout.trajectory_time = 1;
  mout.orientation.w = quat.w();
  mout.orientation.x = quat.x();
  mout.orientation.y = quat.y();
  mout.orientation.z = quat.z();
  mout.unique_id = msg->utime;
  neck_orientation_pub_.publish(mout);
}

void LCM2ROS::headOrientationHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg)
{
  ROS_ERROR("LCM2ROS got desired desired head orientation");
  ihmc_msgs::HeadOrientationPacketMessage mout;
  mout.trajectory_time = 1;
  mout.orientation.w = msg->orientation[0];
  mout.orientation.x = msg->orientation[1];
  mout.orientation.y = msg->orientation[2];
  mout.orientation.z = msg->orientation[3];
  mout.unique_id = msg->utime;
  neck_orientation_pub_.publish(mout);
}

void LCM2ROS::lFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg)
{
  ROS_ERROR("LCM2ROS got desired foot pose");
  ihmc_msgs::FootPosePacketMessage mout;
  mout.trajectory_time = 1;
  mout.position.x = msg->pos[0];
  mout.position.y = msg->pos[1];
  mout.position.z = msg->pos[2];
  mout.orientation.w = msg->orientation[0];
  mout.orientation.x = msg->orientation[1];
  mout.orientation.y = msg->orientation[2];
  mout.orientation.z = msg->orientation[3];
  mout.unique_id = msg->utime;
  mout.robot_side = 0;
  lfoot_pose_pub_.publish(mout);
}

void LCM2ROS::rFootPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg)
{
  ROS_ERROR("LCM2ROS got desired foot pose");
  ihmc_msgs::FootPosePacketMessage mout;
  mout.trajectory_time = 1;
  mout.position.x = msg->pos[0];
  mout.position.y = msg->pos[1];
  mout.position.z = msg->pos[2];
  mout.orientation.w = msg->orientation[0];
  mout.orientation.x = msg->orientation[1];
  mout.orientation.y = msg->orientation[2];
  mout.orientation.z = msg->orientation[3];
  mout.unique_id = msg->utime;
  mout.robot_side = 1;
  rfoot_pose_pub_.publish(mout);
}

void filterJointNamesToIHMC(std::vector<std::string> &joint_name)
{
  // Rename these joints to expected values:

  int n_joints = joint_name.size();
  for (int i = 0; i < n_joints; i++)
  {
    // ihmc v3 to mit v3:
    // if (joint_name[i] == "l_arm_shz"){
    //   joint_name[i] = "l_arm_usy";
    // }
    // if (joint_name[i] == "r_arm_shz"){
    //   joint_name[i] = "r_arm_usy";
    // }

    if (joint_name[i] == "l_arm_uwy")
    {
      joint_name[i] = "l_arm_wry";
    }
    if (joint_name[i] == "l_arm_mwx")
    {
      joint_name[i] = "l_arm_wrx";
    }
    if (joint_name[i] == "l_arm_mwx")
    {
      joint_name[i] = "l_arm_wrx";
    }
    if (joint_name[i] == "r_arm_uwy")
    {
      joint_name[i] = "r_arm_wry";
    }
    if (joint_name[i] == "r_arm_mwx")
    {
      joint_name[i] = "r_arm_wrx";
    }

    // ihmc v5 to mit v5:
    if (joint_name[i] == "l_arm_lwy")
    {
      joint_name[i] = "l_arm_wry2";
    }
    if (joint_name[i] == "r_arm_lwy")
    {
      joint_name[i] = "r_arm_wry2";
    }

    if (joint_name[i] == "neck_ay")
    {
      joint_name[i] = "neck_ry";
    }
    if (joint_name[i] == "hokuyo_joint")
    {
      // double output = remainderf( joint_position[i] , M_PI);
      // std::cout << (joint_position[i]) << " "  << output << "\n";
      // joint_position[i] = output;
      // joint_name[i] = "hokuyo_link";
    }
  }
}

void LCM2ROS::sendSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                                std::vector<std::string> input_joint_names, bool is_right)
{
  ihmc_msgs::ArmJointTrajectoryPacketMessage m;
  bool status = getSingleArmPlan(msg, output_joint_names_arm, input_joint_names, is_right, m);
  if (status)
    arm_joint_traj2_pub_.publish(m);
}


double LCM2ROS::getPlanTimeAtWaypoint(int64_t planUtime)
{
  // Convert the input plan timestamp (int64_t into a safe output (double, seconds)
  // including adjusting by a fixed offset which is required to do the difference
  // between the current joint positions (from where the plan begins) and the
  // controller's current desired position references. Used for both arm and pelvis timestamps
  double time_at_waypoint = (double) planDesiredOffset_ + (planUtime * 1E-6);
  return time_at_waypoint;
}



bool LCM2ROS::getSingleArmPlan(const drc::robot_plan_t* msg, std::vector<std::string> output_joint_names_arm,
                               std::vector<std::string> input_joint_names, bool is_right,
                               ihmc_msgs::ArmJointTrajectoryPacketMessage &m)
{
  // Find the indices of the arm joints which we want to extract
  std::vector<int> arm_indices;
  for (size_t i = 0; i < output_joint_names_arm.size(); i++)
  {
    std::string name = output_joint_names_arm[i];
    std::vector<std::string>::iterator it;
    it = find(input_joint_names.begin(), input_joint_names.end(), name);
    int index = std::distance(input_joint_names.begin(), it);
    if (index < input_joint_names.size())
    {
      // std::cout << name << " found in input_joint_names at " << index << '\n';
      arm_indices.push_back(index);
    }
    else
    {
      ROS_ERROR("%s not found in input_joint_names, not sending plan", name.c_str());
      std::cout << name << " not found in input_joint_names, not sending plan\n";
      return false;
    }
  }

  // Fish out the arm indices:
  if (is_right)
  {
    m.robot_side = 1;
  }
  else
  {
    m.robot_side = 0;
  }

  // m.joint_names = output_joint_names_arm;
  for (int i = 1; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
  {
    bot_core::robot_state_t state = msg->plan[i];
    ihmc_msgs::JointTrajectoryPointMessage point;
    int i1 = (i > 0) ? (i - 1) : 0;
    int i2 = i;
    int i3 = (i < msg->num_states - 1) ? (i + 1) : (msg->num_states - 1);

    for (int j = 0; j < arm_indices.size(); j++)
    {
      point.positions.push_back(state.joint_position[arm_indices[j]]);
      double dt1 = (msg->plan[i2].utime - msg->plan[i1].utime) * 1e-6;
      double dt2 = (msg->plan[i3].utime - msg->plan[i2].utime) * 1e-6;
      double dq1 = msg->plan[i2].joint_position[arm_indices[j]] - msg->plan[i1].joint_position[arm_indices[j]];
      double dq2 = msg->plan[i3].joint_position[arm_indices[j]] - msg->plan[i2].joint_position[arm_indices[j]];
      point.velocities.push_back((dt1 * dt2 != 0) ? (dq1 / dt1 * 0.5 + dq2 / dt2 * 0.5) : 0.0);
      // point.accelerations.push_back( 0  );
      // point.effort.push_back( state.joint_effort[ arm_indices[j] ] );

      point.time = getPlanTimeAtWaypoint( state.utime );
      std::cout << i << ": " << getPlanTimeAtWaypoint(state.utime) << " " << (state.utime - msg->plan[i-1].utime) * 1E-6
                << " is time and difference of the arm waypoints [Right: " << is_right << "]\n";


    }
    m.trajectory_points.push_back(point);
  }
  return true;
}

Eigen::Isometry3d KDLToEigen(KDL::Frame tf)
{
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion(q.x(), q.y(), q.z(), q.w());
  tf_out.rotate(q);
  return tf_out;
}


bool LCM2ROS::getChestTrajectoryPlan(const drc::robot_plan_t* msg, std::vector<geometry_msgs::Quaternion> &m)
{
  for (int i = 1; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
  {
    // 0. Extract World Pose of body:
    bot_core::robot_state_t this_state = msg->plan[i];
    Eigen::Isometry3d world_to_body;
    world_to_body.setIdentity();
    world_to_body.translation()  << this_state.pose.translation.x, this_state.pose.translation.y, this_state.pose.translation.z;
    world_to_body.rotate(Eigen::Quaterniond(this_state.pose.rotation.w, this_state.pose.rotation.x,
                                                 this_state.pose.rotation.y, this_state.pose.rotation.z));

    // 1. Solve for Forward Kinematics:
    std::map<std::string, double> jointpos_in;
    std::map<std::string, KDL::Frame > cartpos_out;
    for (uint i=0; i< (uint) this_state.num_joints; i++)  // cast to uint to suppress compiler warning
      jointpos_in.insert(make_pair(this_state.joint_name[i], this_state.joint_position[i]));

    // Calculate forward position kinematics
    bool kinematics_status;
    bool flatten_tree = true;  // determines absolute transforms to robot origin, otherwise relative transforms between joints.
    kinematics_status = fksolver_->JntToCart(jointpos_in, cartpos_out, flatten_tree);
    if (kinematics_status < 0)
    {
      std::cerr << "Error: could not calculate forward kinematics!" << std::endl;
      return false;
    }

    // 2. Find the world orientation of the chest:
    Eigen::Isometry3d world_to_torso = world_to_body * KDLToEigen(cartpos_out.find(chestLinkName_)->second);
    Eigen::Quaterniond wTt_quat = Eigen::Quaterniond(world_to_torso.rotation());
    geometry_msgs::Quaternion this_chest;
    this_chest.w = wTt_quat.w();
    this_chest.x = wTt_quat.x();
    this_chest.y = wTt_quat.y();
    this_chest.z = wTt_quat.z();
    m.push_back(this_chest);

    /*
    bot_core::pose_t lcm_pose_msg;
    lcm_pose_msg.utime = (int64_t)0;
    lcm_pose_msg.pos[0] = world_to_torso.translation().x();
    lcm_pose_msg.pos[1] = world_to_torso.translation().y();
    lcm_pose_msg.pos[2] = world_to_torso.translation().z();
    lcm_pose_msg.orientation[0] = wTt_quat.w();
    lcm_pose_msg.orientation[1] = wTt_quat.x();
    lcm_pose_msg.orientation[2] = wTt_quat.y();
    lcm_pose_msg.orientation[3] = wTt_quat.z();
    lcm_->publish("POSE_BDI", &lcm_pose_msg);
    std::cout << i << "\n";
    sleep(1);
    */
  }

  return true;
}


void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg)
{
  ROS_ERROR("LCM2ROS got robot plan with %d states", msg->num_states);

  ihmc_msgs::WholeBodyTrajectoryPacketMessage wbt_msg;
  wbt_msg.unique_id = msg->utime;

  // 1. Insert Arm Joints
  std::vector<std::string> l_arm_strings;
  std::vector<std::string> r_arm_strings;
  std::vector<std::string> input_joint_names = msg->plan[0].joint_name;
  if (robotName_.compare("atlas") == 0)
  {
    // Remove MIT/IPAB joint names and use IHMC joint names:
    filterJointNamesToIHMC(input_joint_names);
    l_arm_strings =
    { "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx", "l_arm_wry2"};
    r_arm_strings =
    { "r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx", "r_arm_wry2"};
  }
  else if (robotName_.compare("valkyrie") == 0)
  {
    l_arm_strings =
    {"leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw",
    "leftWristRoll", "leftWristPitch"};
    r_arm_strings =
    {"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw",
    "rightWristRoll", "rightWristPitch"};
  }
  ihmc_msgs::ArmJointTrajectoryPacketMessage left_arm_trajectory;
  bool status_left = getSingleArmPlan(msg, l_arm_strings, input_joint_names, false, left_arm_trajectory);
  ihmc_msgs::ArmJointTrajectoryPacketMessage right_arm_trajectory;
  bool status_right = getSingleArmPlan(msg, r_arm_strings, input_joint_names, true, right_arm_trajectory);
  if (!status_left || !status_right)
  {
    ROS_ERROR("LCM2ROS: problem with arm plan, not sending");
    return;
  }
  wbt_msg.left_arm_trajectory = left_arm_trajectory;
  wbt_msg.right_arm_trajectory = right_arm_trajectory;
  wbt_msg.num_joints_per_arm = l_arm_strings.size();

  // 2. Insert Pelvis Pose
  for (int i = 1; i < msg->num_states; i++)  // NB: skipping the first sample as it has time = 0
  {
    bot_core::robot_state_t state = msg->plan[i];

    geometry_msgs::Vector3 pelvis_world_position;
    pelvis_world_position.x = state.pose.translation.x;
    pelvis_world_position.y = state.pose.translation.y;
    pelvis_world_position.z = state.pose.translation.z;
    wbt_msg.pelvis_world_position.push_back(pelvis_world_position);

    geometry_msgs::Quaternion pelvis_world_orientation;
    pelvis_world_orientation.w = state.pose.rotation.w;
    pelvis_world_orientation.x = state.pose.rotation.x;
    pelvis_world_orientation.y = state.pose.rotation.y;
    pelvis_world_orientation.z = state.pose.rotation.z;
    wbt_msg.pelvis_world_orientation.push_back(pelvis_world_orientation);
    wbt_msg.time_at_waypoint.push_back( getPlanTimeAtWaypoint(state.utime) ) ;
    std::cout << i << ": " << getPlanTimeAtWaypoint(state.utime) << " " << (state.utime - msg->plan[i-1].utime) * 1E-6 << " is time and difference of the pelvis waypoints\n";

  }
  wbt_msg.num_waypoints = msg->num_states - 1;  // NB: skipping the first sample as it has time = 0

  // 3. Insert Chest Pose (in work frame)
  std::vector<geometry_msgs::Quaternion> chest_trajectory;
  bool status_chest = getChestTrajectoryPlan(msg, chest_trajectory);
  if (!status_chest)
  {
    ROS_ERROR("LCM2ROS: problem with chest plan, not sending");
    return;
  }
  wbt_msg.chest_world_orientation = chest_trajectory;

  if (outputTrajectoryMode_ == TrajectoryMode::wholeBody)
  {
    whole_body_trajectory_pub_.publish(wbt_msg);
    ROS_ERROR("LCM2ROS sent Whole Body Trajectory");
  }
  else if (outputTrajectoryMode_ == TrajectoryMode::leftArm)
  {
    sendSingleArmPlan(msg, l_arm_strings, input_joint_names, false);
    ROS_ERROR("LCM2ROS sent left arm");
  }
  else if (outputTrajectoryMode_ == TrajectoryMode::rightArm)
  {
    sendSingleArmPlan(msg, r_arm_strings, input_joint_names, true);
    ROS_ERROR("LCM2ROS sent right arm");
  }
  else if (outputTrajectoryMode_ == TrajectoryMode::bothArms)
  {
    sendSingleArmPlan(msg, l_arm_strings, input_joint_names, false);
    ROS_ERROR("LCM2ROS sent left arm");
    sleep(1); 
    ROS_ERROR("LCM2ROS sleep 1 second");
    sendSingleArmPlan(msg, r_arm_strings, input_joint_names, true);
    ROS_ERROR("LCM2ROS sent right arm");
  }

  if (1==0){
    if (robotName_.compare("valkyrie") == 0){

      std::vector<std::string>::iterator it1 = find(input_joint_names.begin(),
          input_joint_names.end(), "lowerNeckPitch");
      int lowerNeckPitchIndex = std::distance(input_joint_names.begin(), it1);
      float lowerNeckPitchAngle = msg->plan[ msg->num_states-1 ].joint_position[lowerNeckPitchIndex];

      std::vector<std::string>::iterator it2 = find(input_joint_names.begin(),
          input_joint_names.end(), "neckYaw");
      int neckYawIndex = std::distance(input_joint_names.begin(), it2);
      float neckYawAngle = msg->plan[ msg->num_states-1 ].joint_position[neckYawIndex];

      Eigen::Quaterniond headOrientation = euler_to_quat(0, lowerNeckPitchAngle, neckYawAngle);
      ihmc_msgs::HeadOrientationPacketMessage mout;
      mout.trajectory_time = 1; // time doesn't seem to be tracked currently.
      mout.orientation.w = headOrientation.w();
      mout.orientation.x = headOrientation.x();
      mout.orientation.y = headOrientation.y();
      mout.orientation.z = headOrientation.z();
      mout.unique_id = msg->utime;
      neck_orientation_pub_.publish(mout);
      ROS_ERROR("LCM2ROS sent desired head orientation");
    }
  }
   
}


void LCM2ROS::ihmcControlModeCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::int64_stamped_t* msg)
{
  outputTrajectoryMode_ = TrajectoryMode( (int) msg->data);
  ROS_ERROR("LCM2ROS Setting Controller TrajectoryMode: %s",
      TrajectoryNames.at( (int) outputTrajectoryMode_).c_str() );
}

int main(int argc, char** argv)
{
  std::string robotName;  // = "valkyrie";  // "atlas"

  if (argc >= 2)
  {
    ROS_ERROR("Robot Name: %s", argv[1]);
    robotName = argv[1];
  }
  else
  {
    ROS_ERROR("Need to have an argument: robot name");
    exit(-1);
  }

  ros::init(argc, argv, "lcm2ros_ihmc", ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh, robotName);
  ROS_ERROR("LCM2ROS IHMC Translator Ready [robotName: %s]", robotName.c_str());

  while (0 == lcm->handle())
  {
  }

  return 0;
}
