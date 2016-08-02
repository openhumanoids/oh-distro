#pragma once
#include "drc/controller_state_t.hpp"
#include "drake/lcmt_qp_controller_input.hpp"
#include "humanoid_status.h"

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
};

class QPIO {
 public:
  // output
  Eigen::VectorXd qdd;
  Eigen::VectorXd trq;
  Eigen::Vector6d grf_w[2]; // grf in world frame, at the ft sensor
  Eigen::Vector6d grf_b[2]; // grf in body frame, at the ft sensor
  Eigen::Vector2d cop_w; // total cop in world
  Eigen::Vector2d cop_b[2]; // b frame cop for each foot, at the ft sensor

  Eigen::Vector3d comdd;
  Eigen::Vector6d pelvdd;
  Eigen::Vector6d footdd[2];
  Eigen::Vector6d torsodd;

  // input
  Eigen::Vector3d com_d;
  Eigen::Vector3d comd_d;
  Eigen::Vector3d comdd_d;
  Eigen::Vector3d comdd_d1;
  Eigen::Vector2d cop_d;

  Eigen::VectorXd qdd_d;
  Eigen::Vector6d pelvdd_d;
  Eigen::Vector6d footdd_d[2];
  Eigen::Vector6d torsodd_d;

  void ParseZMPInput(const drake::lcmt_qp_controller_input &msg);
  void ParseMsg(const drc::controller_state_t &msg, const HumanoidStatus &rs);

  void Init(const drc::controller_state_t &msg) {
    qdd.resize(msg.num_joints);
    trq.resize(msg.num_joints);
    _inited = true;
  }

  bool has_init() const { return this->_inited; }

  void AddToLog(MRDLogger &logger, const HumanoidStatus &rs) const;

  QPIO() {
    _inited = false;
    _hasZMPInput = false;
  }

 private:
  bool _inited;
  bool _hasZMPInput;
  Eigen::Matrix<double,4,4> A_ls;
  Eigen::Matrix<double,4,2> B_ls;
  Eigen::Matrix<double,2,4> C_ls;
  Eigen::Matrix<double,2,2> D_ls;
  Eigen::Matrix<double,4,1> x0;
  Eigen::Matrix<double,2,1> y0;
  Eigen::Matrix<double,2,1> u0;
  Eigen::Matrix<double,2,2> R_ls;
  Eigen::Matrix<double,2,2> Qy;
  Eigen::Matrix<double,4,4> S;
  Eigen::Matrix<double,4,1> s1;
  Eigen::Matrix<double,4,1> s1dot;
};
