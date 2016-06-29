#pragma once

#include "drake/lcmt_qp_controller_input.hpp"
#include "drc/controller_state_t.hpp"
#include "drake/systems/robotInterfaces/Side.h" 
#include "MRDLogger.h"
#include "sfRobotState.h"

using namespace Eigen;

class sfQPState {
public:
  // output
  VectorXd qdd;
  VectorXd trq;
  Vector6d grf_w[2]; // grf in world frame, at the ft sensor
  Vector6d grf_b[2]; // grf in body frame, at the ft sensor
  Vector2d cop_w; // total cop in world
  Vector2d cop_b[2]; // b frame cop for each foot, at the ft sensor

  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d footdd[2];
  Vector6d torsodd;

  // input
  Vector3d com_d;
  Vector3d comd_d;
  Vector3d comdd_d;
  Vector3d comdd_d1;
  Vector2d cop_d;
  
  VectorXd qdd_d;
  Vector6d pelvdd_d;
  Vector6d footdd_d[2];
  Vector6d torsodd_d;

  void parseZMPInput(const drake::lcmt_qp_controller_input &msg);
  void parseMsg(const drc::controller_state_t &msg, const sfRobotState &rs);
  void init(const drc::controller_state_t &msg)
  {
    this->qdd.resize(msg.num_joints);
    this->trq.resize(msg.num_joints);
    this->_inited = true;
  }

  bool hasInit() const { return this->_inited; }

  void addToLog(MRDLogger &logger, const sfRobotState &rs) const;

  sfQPState() 
  {
    this->_inited = false;
    this->_hasZMPInput = false;
  }

private:
  bool _inited;
  bool _hasZMPInput;
  Matrix<double,4,4> A_ls; 
  Matrix<double,4,2> B_ls;
  Matrix<double,2,4> C_ls;
  Matrix<double,2,2> D_ls;
  Matrix<double,4,1> x0;
  Matrix<double,2,1> y0;
  Matrix<double,2,1> u0;
  Matrix<double,2,2> R_ls;
  Matrix<double,2,2> Qy;
  Matrix<double,4,4> S;
  Matrix<double,4,1> s1;
  Matrix<double,4,1> s1dot;
};
 
