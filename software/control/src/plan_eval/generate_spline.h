#pragma once

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/util/drakeGeometryUtil.h"

struct SimplePose {
  Eigen::Vector3d lin;
  Eigen::Quaterniond rot;
};

// poses and vels are task space pose and vel
PiecewisePolynomial<double> GenerateCubicCartesianSpline(const std::vector<double> &times, const std::vector<SimplePose> &poses, const std::vector<SimplePose> &vels);

template <typename Scalar, int rows, int cols> bool CheckSplineInputs(const std::vector<Scalar> &T, const std::vector<Eigen::Matrix<double, rows, cols>> &Y) {
  bool ret = T.size() == Y.size();
  ret &= (T.size() >= 2);
  for (size_t t = 0; t < T.size()-1; t++) {
    ret &= (Y[t].rows() == Y[t+1].rows() && Y[t].cols() == Y[t+1].cols());
    ret &= T[t] < T[t+1];
  }
  return ret;
}

template <typename Scalar, int rows, int cols> PiecewisePolynomial<Scalar> GenerateLinearSpline(const std::vector<Scalar> &T, const std::vector<Eigen::Matrix<double, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(N - 1);

  for (size_t t = 0; t < N-1; t++) {
    polynomials[t].resize(Y[t].rows(),Y[t].cols());
  }

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].rows(); k++) {
      for (size_t t = 0; t < N-1; t++) {
        polynomials[t](j,k) = Polynomial<Scalar>(Eigen::Vector2d(Y[t](j,k), (Y[t+1](j,k) - Y[t](j,k)) / (T[t+1] - T[t])));
      }
    }
  }
  return PiecewisePolynomial<double>(polynomials, T);
}

template <typename Scalar, int rows, int cols> PiecewisePolynomial<Scalar> GeneratePCHIPSpline(const std::vector<Scalar> &T, const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y)
{
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  if (N == 2) {
    return GenerateLinearSpline(T, Y);
  }

  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(N-1);
  std::vector<Eigen::Matrix<Scalar, rows, cols>> m(N-1);
  std::vector<Eigen::Matrix<Scalar, rows, cols>> c1(N);
  std::vector<Scalar> dt(N-1);

  for (size_t t = 0; t < N-1; t++) {
    dt[t] = T[t+1] - T[t];
    m[t] = (Y[t+1] - Y[t]) / dt[t];
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
  }

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      for (size_t t = 0; t < dt.size()-1; t++) {
        if (m[t](j,k) * m[t+1](j,k) <= 0) {
          c1[t+1](j,k) = 0;
        }
        else {
          Scalar common = dt[t] + dt[t+1];
          c1[t+1](j,k) = 3*common / ((common + dt[t+1])/m[t](j,k) + (common + dt[t]) / m[t+1](j,k));
        }
      }

      // fix end points' slopes
      c1[0](j,k) = ((2*dt[0]+dt[1])*m[0](j,k) - dt[0]*m[1](j,k))/(dt[0]+dt[1]);
      if (c1[0](j,k) * m[0](j,k) <= 0)
        c1[0](j,k) = 0;
      else if (m[0](j,k) * m[1](j,k) <= 0 && abs(c1[0](j,k)) > abs(3*m[0](j,k)))
        c1[0](j,k) = 3*m[0](j,k);

      int n = N-1;
      c1[n](j,k) = ((2*dt[n-1]+dt[n-2])*m[n-1](j,k) - dt[n-1]*m[n-2](j,k))/(dt[n-1]+dt[n-2]);
      if (c1[n](j,k) * m[n-1](j,k) <= 0)
        c1[n](j,k) = 0;
      else if (m[n-1](j,k) * m[n-2](j,k) <= 0 && abs(c1[n](j,k)) > abs(3*m[n-1](j,k)))
        c1[n](j,k) = 3*m[n-1](j,k);

      for (size_t t = 0; t < N-1; t++) {
        Eigen::Vector4d coeffs;
        coeffs[0] = Y[t](j,k);
        coeffs[1] = c1[t](j,k);
        coeffs[3] = ((c1[t+1](j,k) - c1[t](j,k)) * dt[t] / 2 - Y[t+1](j,k) + Y[t](j,k) + c1[t](j,k) * dt[t]) * 2 / (dt[t]*dt[t]*dt[t]);
        coeffs[2] = (Y[t+1](j,k) - Y[t](j,k) - c1[t](j,k) * dt[t] - coeffs[3] * (dt[t]*dt[t]*dt[t])) / (dt[t]*dt[t]);
        polynomials[t](j,k) = Polynomial<Scalar>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

template <typename Scalar, int rows, int cols> PiecewisePolynomial<Scalar> GenerateCubicSpline(const std::vector<Scalar> &T, const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y, const Eigen::Matrix<Scalar, rows, cols> &dydt0, const Eigen::Matrix<Scalar, rows, cols> &dydt1) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }
  assert(dydt0.rows() == dydt1.rows() && dydt0.rows() == Y[0].rows());
  assert(dydt0.cols() == dydt1.cols() && dydt0.cols() == Y[0].cols());

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(N-1);

  for (size_t i = 0; i < N-1; i++) {
    polynomials[i].resize(Y[i].rows(), Y[i].cols());
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4*(N-1), 4*(N-1));
  Eigen::VectorXd b = Eigen::VectorXd::Zero(4*(N-1));
  Eigen::VectorXd solution;

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      // general case
      size_t rowIdx = 0;

      for (size_t i = 0; i < N-1; i++) {
        double duration = T[i+1] - T[i];

        // y_i(x_i) = a0i = Y[i]
        A(rowIdx, 4*i) = 1;
        b(rowIdx++) = Y[i](j, k);

        // y_i(x_{i+1}) = y_{i+1}(x_{i}) =>
        // a0i + a1i*(x_{i+1} - x_i) + a2i(x_{i+1} - x_i)^2 + a3i(x_{i+1} - x_i)^3 = a0{i+1}
        A(rowIdx, 4*i+0) = 1;
        A(rowIdx, 4*i+1) = duration;
        A(rowIdx, 4*i+2) = duration*duration;
        A(rowIdx, 4*i+3) = duration*duration*duration;
        if (i != N-2) {
          A(rowIdx++, 4*(i+1)) = -1;
        }
        else {
          b(rowIdx++) = Y[N-1](j, k);
        }

        // y_i'(x_{i+1}) = y_{i+1}'(x_{i}) =>
        // a1i + 2*a2i(x_{i+1} - x_i) + 3*a3i(x_{i+1} - x_i)^2 = a1{i+1}
        if (i != N-2) {
          A(rowIdx, 4*i+1) = 1;
          A(rowIdx, 4*i+2) = 2*duration;
          A(rowIdx, 4*i+3) = 3*duration*duration;
          A(rowIdx++, 4*(i+1)+1) = -1;
        }

        if (i != N-2) {
          // y_i''(x_{i+1}) = y_{i+1}''(x_{i}) =>
          // 2*a2i + 6*a3i(x_{i+1} - x_i) = 2*a2{i+1}
          A(rowIdx, 4*i+2) = 2;
          A(rowIdx, 4*i+3) = 6*duration;
          A(rowIdx++, 4*(i+1)+2) = -2;
        }
      }

      // dx0
      A(rowIdx, 1) = 1;
      b(rowIdx++) = dydt0(j, k);

      // dx1
      A(rowIdx, 4*(N-2)+1) = 1;
      A(rowIdx, 4*(N-2)+2) = 2*(T[N-1]-T[N-2]);
      A(rowIdx, 4*(N-2)+3) = 3*(T[N-1]-T[N-2])*(T[N-1]-T[N-2]);
      b(rowIdx++) = dydt1(j, k);

      assert(rowIdx == 4*(N-1));
      auto decomposition = A.colPivHouseholderQr();
      assert(decomposition.isInvertible());
      solution = decomposition.solve(b);

      for (size_t i = 0; i < N-1; i++) {
        polynomials[i](j, k) = Polynomial<double>(solution.segment<4>(4*i));
      }
    }
  }

  return PiecewisePolynomial<double>(polynomials, T);
}

template <typename Scalar, int rows, int cols> PiecewisePolynomial<Scalar> GenerateCubicSpline(const std::vector<Scalar> &T, const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }
  size_t N = T.size();
  Eigen::Matrix<Scalar, rows, cols> dydt0, dydt1;
  dydt0 = (Y[1] - Y[0]) / (T[1] - T[0]);
  dydt1 = (Y[N-1] - Y[N-2]) / (T[N-1] - T[N-2]);
  return GenerateCubicSpline(T, Y, dydt0, dydt1);
}

         
