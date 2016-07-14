#include "zmp_planner.h"
#include "drake/util/drakeUtil.h"
#include <unsupported/Eigen/MatrixFunctions>

void ZMPPlanner::Plan(const PiecewisePolynomial<double> &zmp_d, double height) {
  assert(zmp_d.rows() == 2 && zmp_d.cols() == 1);
  assert(zmp_d.getSegmentPolynomialDegree(0) == 4);

  zmp_traj_ = zmp_d;

  A_.setZero();
  A_.block<2, 2>(0, 2).setIdentity();
  B_.setZero();
  B_.block<2, 2>(2, 0).setIdentity();
  C_.setZero();
  C_.block<2, 2>(0, 0).setIdentity();
  D_ = -height / 9.81 * Eigen::Matrix2d::Identity();
  // TODO: take params
  D_control_ = D_;

  // TODO: take params
  Qy_ = Eigen::Matrix2d::Identity();
  R_.setZero();

  Eigen::Matrix<double, 4, 4> Q1 = C_.transpose() * Qy_ * C_;
  Eigen::Matrix<double, 2, 2> R1 = R_ + D_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 4, 2> N = C_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 2, 2> R1i = R1.inverse();

  lqr(A_, B_, Q1, R1, N, K_, S_);
  K_ = -K_;

  s1_dot_.setZero();
  u0_.setZero(); 

  Eigen::Matrix<double, 2, 4> NB = (N.transpose() + B_.transpose() * S_);
  Eigen::Matrix<double, 4, 4> A2 = NB.transpose() * R1i * B_.transpose() - A_.transpose();
  Eigen::Matrix<double, 4, 2> B2 = 2 * (C_.transpose() - NB.transpose() * R1i * D_) * Qy_;
  Eigen::Matrix<double, 4, 4> A2i = A2.inverse();

  int n_segments = zmp_d.getNumberOfSegments();
  Eigen::VectorXd coeff_x, coeff_y;
  Eigen::Matrix<double, 2, 4> c;
  Eigen::Vector4d s1dt;

  // number of segments * number of degree * [x, y, dx, dy]
  std::vector<std::vector<Eigen::Vector4d>> beta(n_segments);
  std::vector<std::vector<Eigen::Vector2d>> gamma(n_segments);
  Eigen::MatrixXd alpha(4, n_segments);
  
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> beta_poly(n_segments);

  for (int t = n_segments-1; t >= 0; t--) {
    beta[t].resize(4);
    gamma[t].resize(4);

    coeff_x = zmp_d.getPolynomial(t, 0, 0).getCoefficients();
    coeff_y = zmp_d.getPolynomial(t, 1, 0).getCoefficients();
    c.row(0) = coeff_x;
    c.row(1) = coeff_y;

    // degree 4
    beta[t][3] = -A2i * B2 * c.col(3);
    gamma[t][3] = R1i * D_ * Qy_ * c.col(3) - 0.5 * R1i * B_.transpose() * beta[t][3];
    for (int d = 2; d >= 0; d--) {
      beta[t][d] = A2i * (d * beta[t][d+1] - B2 * c.col(d));
      gamma[t][d] = R1i * D_ * Qy_ * c.col(d) - 0.5 * R1i * B_.transpose() * beta[t][d];
    }
    if (t == n_segments-1) {
      s1dt = Eigen::Vector4d::Zero();
    }
    else {
      s1dt = alpha.col(t+1) + beta[t+1][0];
    }
    double dt = zmp_d.getDuration(t);
    Eigen::Matrix4d A2exp = A2 * dt;
    Eigen::Matrix4d squeezed_beta;
    for (int j = 0; j < 4; j++) {
      squeezed_beta.col(j) = beta[t][j];
    }

    A2exp = A2exp.exp();
    alpha.col(t) = Eigen::Vector4d(1, dt, dt * dt, dt * dt * dt);
    alpha.col(t) = s1dt - squeezed_beta * alpha.col(t);
    alpha.col(t) = A2exp.inverse() * alpha.col(t);

    // setup the poly part
    beta_poly[t].resize(4,1);
    for (int d = 0; d < 4; d++) {
      Eigen::Vector4d tmp(beta[t][0][d], beta[t][1][d], beta[t][2][d], beta[t][3][d]);
      beta_poly[t](d, 0) = Polynomial<double>(tmp);
    }
  }

  PiecewisePolynomial<double> beta_traj(beta_poly, zmp_d.getSegmentTimes());
  s1_traj_ = ExponentialPlusPiecewisePolynomial<double>(Eigen::Matrix4d::Identity(), A2, alpha, beta_traj);
}

drake::lcmt_zmp_data ZMPPlanner::MakeMessage(double plan_time) const {
  drake::lcmt_zmp_data zmp_data_lcm;
  zmp_data_lcm.timestamp = 0;
  eigenToCArrayOfArrays(A_, zmp_data_lcm.A);
  eigenToCArrayOfArrays(B_, zmp_data_lcm.B);
  eigenToCArrayOfArrays(C_, zmp_data_lcm.C);
  eigenToCArrayOfArrays(D_control_, zmp_data_lcm.D);
  eigenToCArrayOfArrays(u0_, zmp_data_lcm.u0);
  eigenToCArrayOfArrays(R_, zmp_data_lcm.R);
  eigenToCArrayOfArrays(Qy_, zmp_data_lcm.Qy);
  zmp_data_lcm.s2 = 0;  // never used by the controller
  zmp_data_lcm.s2dot = 0;

  Eigen::Vector2d zmp_d_final = zmp_traj_.value(zmp_traj_.getEndTime());
  for (size_t i = 0; i < 2; i++) {
    zmp_data_lcm.x0[i][0] = zmp_d_final[i];
    zmp_data_lcm.x0[i + 2][0] = 0;
  }
  Eigen::Vector2d zmp_d = zmp_traj_.value(plan_time);
  eigenToCArrayOfArrays(zmp_d, zmp_data_lcm.y0);
  
  // Lyapunov function
  eigenToCArrayOfArrays(S_, zmp_data_lcm.S);
  Eigen::Vector4d s1 = s1_traj_.value(plan_time);
  eigenToCArrayOfArrays(s1, zmp_data_lcm.s1);
  eigenToCArrayOfArrays(s1_dot_, zmp_data_lcm.s1dot);
   
  return zmp_data_lcm;
}
