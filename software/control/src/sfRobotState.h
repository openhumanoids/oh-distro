#pragma once

#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "RobotStateDriver.hpp"

using namespace Eigen;


// stuff that I want to compute
class sfRobotState {
public:
  std::unique_ptr<RigidBodyTree> robot;
  
  // minimum representation
  VectorXd q;
  VectorXd qd;
  Matrix<double,6,1> footFT_b[2];
  
  VectorXd pos;
  VectorXd vel;
  VectorXd trq;
  // computed from kinematics  
  Eigen::Isometry3d pelv;
  Eigen::Isometry3d foot[2];
  Eigen::Isometry3d foot_ft_sensor[2];
  Vector3d com;
  
  Vector3d comd;
  Vector6d pelvd;
  Vector6d footd[2];

  Vector2d cop;
  Vector2d cop_b[2];
  
  Matrix<double,6,1> footFT_w[2];
  

  sfRobotState(std::unique_ptr<RigidBodyTree> robot_in)
    : robot(std::move(robot_in)),
      cache(this->robot->bodies)
  {
    _init();
  }

  void parseMsg(const bot_core::robot_state_t &msg);
  void buildJointName2Id(const bot_core::robot_state_t &msg);

private:
  DrakeRobotState robot_state;
  std::shared_ptr<RobotStateDriver> state_driver;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> body_or_frame_name_to_id;
  std::unordered_map<std::string, int> joint_name_to_id;

  void _init();
};

