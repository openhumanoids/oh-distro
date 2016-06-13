#pragma once

#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "RobotStateDriver.hpp"
#include "drc/controller_state_t.hpp"

using namespace Eigen;


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
  Matrix<double,6,1> footFT_w_statics[2];
  
  // Jacobians
  MatrixXd J_foot[2];
  MatrixXd J_pelv;
  MatrixXd J_com;
  VectorXd Jdqd_foot[2];
  VectorXd Jdqd_pelv;
  VectorXd Jdqd_com;

  sfRobotState(std::unique_ptr<RigidBodyTree> robot_in)
    : robot(std::move(robot_in)),
      cache(this->robot->bodies)
  {
    // build map
    this->body_or_frame_name_to_id = std::unordered_map<std::string, int>();
    for (auto it = robot->bodies.begin(); it != robot->bodies.end(); ++it) {
      this->body_or_frame_name_to_id[(*it)->linkname] = it - robot->bodies.begin();
    }

    for (auto it = robot->frames.begin(); it != robot->frames.end(); ++it) {
      this->body_or_frame_name_to_id[(*it)->name] = -(it - robot->frames.begin()) - 2;
    }

    // init state driver
    int num_states = robot->num_positions + robot->num_velocities;
    std::vector<std::string> state_coordinate_names(num_states);
    for (int i=0; i<num_states; i++){
      state_coordinate_names[i] = robot->getStateName(i);
    }

    state_driver.reset(new RobotStateDriver(state_coordinate_names));

    this->pos.resize(robot->num_positions);
    this->vel.resize(robot->num_positions);
    this->trq.resize(robot->num_positions); 

    this->_inited = false;
  }

  void parseMsg(const bot_core::robot_state_t &msg);
  void init(const bot_core::robot_state_t &msg);
  bool hasInit() const { return this->_inited; }

private:
  DrakeRobotState robot_state;
  std::shared_ptr<RobotStateDriver> state_driver;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> body_or_frame_name_to_id;
  std::unordered_map<std::string, int> joint_name_to_id;
  bool _inited;
};


class sfQPOutput {
public:
  VectorXd qdd;
  VectorXd trq;
  Vector6d grf[2];

  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d footdd[2];

  void parseMsg(const drc::controller_state_t &msg, const sfRobotState &rs);
  void init(const drc::controller_state_t &msg)
  {
    this->qdd.resize(msg.num_joints);
    this->trq.resize(msg.num_joints);
    this->_inited = true;
  }

  bool hasInit() const { return this->_inited; }

  sfQPOutput() 
  {
    this->_inited = false;
  }

private:
  bool _inited;
};

 

