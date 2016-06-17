#pragma once

#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include "RobotStateDriver.hpp"
#include "drc/controller_state_t.hpp"
#include "drake/systems/robotInterfaces/Side.h"
#include "Logger.h"

using namespace Eigen;

class BodyOfInterest {
public:
  std::string name;
  Eigen::Isometry3d pose;
  Vector6d vel;
  
  MatrixXd J;
  Vector6d Jdv;

  BodyOfInterest(const std::string &n) { name = n; }

  void addToLog(Logger &logger) const
  {
    logger.add_datapoint(name+"[x]", "m", pose.translation().data());
    logger.add_datapoint(name+"[y]", "m", pose.translation().data()+1);
    logger.add_datapoint(name+"[z]", "m", pose.translation().data()+2);
    
    logger.add_datapoint(name+"d[x]", "m/s", vel.data()+3);
    logger.add_datapoint(name+"d[y]", "m/s", vel.data()+4);
    logger.add_datapoint(name+"d[z]", "m/s", vel.data()+5);
    logger.add_datapoint(name+"d[wx]", "m/s", vel.data()+0);
    logger.add_datapoint(name+"d[wy]", "m/s", vel.data()+1);
    logger.add_datapoint(name+"d[wz]", "m/s", vel.data()+2);
  }
};


class sfRobotState {
public:
  std::unique_ptr<RigidBodyTree> robot;
  double time;
  
  // minimum representation
  VectorXd q;
  VectorXd qd;
  Matrix<double,6,1> footFT_b[2];
  
  VectorXd pos;
  VectorXd vel;
  VectorXd trq;
  // computed from kinematics  
  Isometry3d foot_ft_sensor[2];
  Vector3d com;
  Vector3d comd;

  BodyOfInterest pelv;
  BodyOfInterest l_foot;
  BodyOfInterest r_foot;
  BodyOfInterest torso;

  std::vector<BodyOfInterest *> foot;

  Vector2d cop;
  Vector2d cop_b[2];
  
  Matrix<double,6,1> footFT_w[2];
  Matrix<double,6,1> footFT_w_statics[2];
  
  // Jacobians
  MatrixXd J_com;
  Vector3d Jdv_com;

  sfRobotState(std::unique_ptr<RigidBodyTree> robot_in)
    : robot(std::move(robot_in)),
      cache(this->robot->bodies),
      pelv("pelvis"),
      l_foot("leftFoot"),
      r_foot("rightFoot"),
      torso("torso")
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

    foot.resize(2);
    foot[Side::LEFT] = &l_foot;
    foot[Side::RIGHT] = &r_foot;
  }

  void parseMsg(const bot_core::robot_state_t &msg);
  void init(const bot_core::robot_state_t &msg);
  bool hasInit() const { return this->_inited; }
  void addToLog(Logger &logger) const;

private:
  DrakeRobotState robot_state;
  std::shared_ptr<RobotStateDriver> state_driver;
  KinematicsCache<double> cache;
  std::unordered_map<std::string, int> body_or_frame_name_to_id;
  std::unordered_map<std::string, int> joint_name_to_id;
  bool _inited;
  double _time0;

  void _fillKinematics(const std::string &name, Isometry3d &pose, Vector6d &vel, MatrixXd &J, Vector6d &Jdv, const Vector3d &local_offset = Vector3d::Zero());
};

Vector6d getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset = Vector3d::Zero());
MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, KinematicsCache<double> &cache, int body, const Vector3d &local_offset = Vector3d::Zero());
Vector6d getTaskSpaceJacobianDotTimesV(const RigidBodyTree &r, KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset = Vector3d::Zero());

class sfQPOutput {
public:
  VectorXd qdd;
  VectorXd trq;
  Vector6d grf[2];

  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d footdd[2];
  Vector6d torsodd;

  void parseMsg(const drc::controller_state_t &msg, const sfRobotState &rs);
  void init(const drc::controller_state_t &msg)
  {
    this->qdd.resize(msg.num_joints);
    this->trq.resize(msg.num_joints);
    this->_inited = true;
  }

  bool hasInit() const { return this->_inited; }

  void addToLog(Logger &logger, const sfRobotState &rs) const;

  sfQPOutput() 
  {
    this->_inited = false;
  }

private:
  bool _inited;
};

 

