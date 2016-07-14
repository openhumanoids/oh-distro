#pragma once

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/trajectories/ExponentialPlusPiecewisePolynomial.h"
#include "drake/lcmt_zmp_data.hpp"

class ZMPPlanner {
 public:
 //private:
  PiecewisePolynomial<double> zmp_traj_;
  PiecewisePolynomial<double> com_traj_;
  ExponentialPlusPiecewisePolynomial<double> s1_traj_;

  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix<double, 2, 4> C_;
  Eigen::Matrix<double, 2, 2> D_;
  Eigen::Matrix<double, 2, 2> D_control_;
  Eigen::Matrix<double, 2, 2> Qy_, R_;
  Eigen::Matrix<double, 4, 4> S_;
  Eigen::Matrix<double, 2, 4> K_;
  
  Eigen::Vector4d s1_dot_;
  Eigen::Vector2d u0_;
  
  void Plan(const PiecewisePolynomial<double> &zmp_d, double height);
  drake::lcmt_zmp_data MakeMessage(double time) const;
};
