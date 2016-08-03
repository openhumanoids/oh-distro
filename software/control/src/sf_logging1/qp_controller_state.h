#pragma once
#include "drc/controller_state_t.hpp"
#include "drake/lcmt_qp_controller_input.hpp"
#include "humanoid_status.h"

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
};

class QPDesiredCartAccInput {
 public:
  std::string name;
  Eigen::Vector6d pos;
  Eigen::Vector6d vel;
  Eigen::Vector6d acc;
  
  Eigen::Vector6d acc_with_pd;

  QPDesiredCartAccInput(const std::string &n) {
    name = n;
  }

  void AddToLog(const std::string &prefix, MRDLogger &logger) const {
    std::string dim[6] = {std::string("x"), std::string("y"), std::string("z"), std::string("wx"), std::string("wy"), std::string("wz")}
    for (int i = 0; i < pos.size(); i++)
      logger.AddChannel(prefix + name + std::string("_d[") + dim[i] + std::string("]"), "-", pos.data() + i);
    for (int i = 0; i < vel.size(); i++)
      logger.AddChannel(prefix + name + std::string("d_d[") + dim[i] + std::string("]"), "-", vel.data() + i);
    for (int i = 0; i < acc.size(); i++)
      logger.AddChannel(prefix + name + std::string("dd_d[") + dim[i] + std::string("]"), "-", acc.data() + i);
    for (int i = 0; i < acc_with_pd.size(); i++)
      logger.AddChannel(prefix + name + std::string("dd_w_pd[") + dim[i] + std::string("]"), "-", acc_with_pd.data() + i);
  }

  void ParseMsg(const drc::qp_desired_body_motion_t &msg) {
    if (name.compare(msg.body_name) != 0) {
      std::cerr << "mismatch name: " << msg.body_name << " " << name << std::endl;
      return;
    }
    for (int i = 0; i < 6; i++) {
      pos[i] = msg.body_q_d[i];
      vel[i] = msg.body_v_d[i];
      acc[i] = msg.body_vdot_d[i];
      acc_with_pd[i] = msg.body_vdot_with_pd[i];
    }
  }
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

  QPDesiredCartAccInput pelv;
  QPDesiredCartAccInput torso;
  QPDesiredCartAccInput foot[2];

  Eigen::VectorXd qdd_d;

  void ParseZMPInput(const drake::lcmt_qp_controller_input &msg);
  void ParseMsg(const drc::controller_state_t &msg, const HumanoidStatus &rs);

  void Init(const drc::controller_state_t &msg) {
    qdd.resize(msg.num_joints);
    trq.resize(msg.num_joints);
    _inited = true;
  }

  bool has_init() const { return this->_inited; }

  void AddToLog(MRDLogger &logger, const HumanoidStatus &rs) const;

  QPIO() 
    : pelv(QPDesiredCartAccInput("pelvis")), 
      torso(QPDesiredCartAccInput("torso")), 
      foot({QPDesiredCartAccInput("leftFoot"), QPDesiredCartAccInput("rightFoot")})
  {
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
