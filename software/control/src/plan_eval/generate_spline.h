#pragma once

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/util/drakeGeometryUtil.h"
#include <iostream>

template <typename Scalar, int rows, int cols>
bool CheckSplineInputs(
    const std::vector<Scalar> &T,
    const std::vector<Eigen::Matrix<double, rows, cols>> &Y) {
  bool ret = T.size() == Y.size();
  ret &= (T.size() >= 2);
  for (size_t t = 0; t < T.size() - 1; t++) {
    ret &= (Y[t].rows() == Y[t + 1].rows() && Y[t].cols() == Y[t + 1].cols());
    ret &= T[t] < T[t + 1];
  }
  return ret;
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateLinearSpline(
    const std::vector<Scalar> &T,
    const std::vector<Eigen::Matrix<double, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);

  for (size_t t = 0; t < N - 1; t++) {
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
  }

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      for (size_t t = 0; t < N - 1; t++) {
        polynomials[t](j, k) = Polynomial<Scalar>(Eigen::Vector2d(
            Y[t](j, k), (Y[t + 1](j, k) - Y[t](j, k)) / (T[t + 1] - T[t])));
      }
    }
  }
  return PiecewisePolynomial<double>(polynomials, T);
}

// copied from matlab source code and wikipedia
template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GeneratePCHIPSpline(
    const std::vector<Scalar> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  if (N == 2) {
    return GenerateLinearSpline(T, Y);
  }

  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);
  std::vector<Eigen::Matrix<Scalar, rows, cols>> m(N - 1);
  std::vector<Eigen::Matrix<Scalar, rows, cols>> c1(N);
  std::vector<Scalar> dt(N - 1);

  for (size_t t = 0; t < N - 1; t++) {
    dt[t] = T[t + 1] - T[t];
    m[t] = (Y[t + 1] - Y[t]) / dt[t];
    polynomials[t].resize(Y[t].rows(), Y[t].cols());
  }

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      for (size_t t = 0; t < dt.size() - 1; t++) {
        if (m[t](j, k) * m[t + 1](j, k) <= 0) {
          c1[t + 1](j, k) = 0;
        } else {
          Scalar common = dt[t] + dt[t + 1];
          c1[t + 1](j, k) = 3 * common / ((common + dt[t + 1]) / m[t](j, k) +
                                          (common + dt[t]) / m[t + 1](j, k));
        }
      }

      // fix end points' slopes
      c1[0](j, k) = ((2 * dt[0] + dt[1]) * m[0](j, k) - dt[0] * m[1](j, k)) /
                    (dt[0] + dt[1]);
      if (c1[0](j, k) * m[0](j, k) <= 0)
        c1[0](j, k) = 0;
      else if (m[0](j, k) * m[1](j, k) <= 0 &&
               abs(c1[0](j, k)) > abs(3 * m[0](j, k)))
        c1[0](j, k) = 3 * m[0](j, k);

      int n = N - 1;
      c1[n](j, k) = ((2 * dt[n - 1] + dt[n - 2]) * m[n - 1](j, k) -
                     dt[n - 1] * m[n - 2](j, k)) /
                    (dt[n - 1] + dt[n - 2]);
      if (c1[n](j, k) * m[n - 1](j, k) <= 0)
        c1[n](j, k) = 0;
      else if (m[n - 1](j, k) * m[n - 2](j, k) <= 0 &&
               abs(c1[n](j, k)) > abs(3 * m[n - 1](j, k)))
        c1[n](j, k) = 3 * m[n - 1](j, k);

      for (size_t t = 0; t < N - 1; t++) {
        Eigen::Vector4d coeffs;
        coeffs[0] = Y[t](j, k);
        coeffs[1] = c1[t](j, k);
        coeffs[3] = ((c1[t + 1](j, k) - c1[t](j, k)) * dt[t] / 2 -
                     Y[t + 1](j, k) + Y[t](j, k) + c1[t](j, k) * dt[t]) *
                    2 / (dt[t] * dt[t] * dt[t]);
        coeffs[2] = (Y[t + 1](j, k) - Y[t](j, k) - c1[t](j, k) * dt[t] -
                     coeffs[3] * (dt[t] * dt[t] * dt[t])) /
                    (dt[t] * dt[t]);
        polynomials[t](j, k) = Polynomial<Scalar>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateCubicSpline(
    const std::vector<Scalar> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y,
    const Eigen::Matrix<Scalar, rows, cols> &dydt0,
    const Eigen::Matrix<Scalar, rows, cols> &dydt1) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }
  assert(dydt0.rows() == dydt1.rows() && dydt0.rows() == Y[0].rows());
  assert(dydt0.cols() == dydt1.cols() && dydt0.cols() == Y[0].cols());

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);

  for (size_t i = 0; i < N - 1; i++) {
    polynomials[i].resize(Y[i].rows(), Y[i].cols());
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4 * (N - 1), 4 * (N - 1));
  Eigen::VectorXd b = Eigen::VectorXd::Zero(4 * (N - 1));
  Eigen::VectorXd solution;

  for (int j = 0; j < Y[0].rows(); j++) {
    for (int k = 0; k < Y[0].cols(); k++) {
      // general case
      size_t rowIdx = 0;

      for (size_t i = 0; i < N - 1; i++) {
        Scalar duration = T[i + 1] - T[i];

        // y_i(x_i) = a0i = Y[i]
        A(rowIdx, 4 * i) = 1;
        b(rowIdx++) = Y[i](j, k);

        // y_i(x_{i+1}) = y_{i+1}(x_{i}) =>
        // a0i + a1i*(x_{i+1} - x_i) + a2i(x_{i+1} - x_i)^2 + a3i(x_{i+1} -
        // x_i)^3 = a0{i+1}
        A(rowIdx, 4 * i + 0) = 1;
        A(rowIdx, 4 * i + 1) = duration;
        A(rowIdx, 4 * i + 2) = duration * duration;
        A(rowIdx, 4 * i + 3) = duration * duration * duration;
        if (i != N - 2) {
          A(rowIdx++, 4 * (i + 1)) = -1;
        } else {
          b(rowIdx++) = Y[N - 1](j, k);
        }

        // y_i'(x_{i+1}) = y_{i+1}'(x_{i}) =>
        // a1i + 2*a2i(x_{i+1} - x_i) + 3*a3i(x_{i+1} - x_i)^2 = a1{i+1}
        if (i != N - 2) {
          A(rowIdx, 4 * i + 1) = 1;
          A(rowIdx, 4 * i + 2) = 2 * duration;
          A(rowIdx, 4 * i + 3) = 3 * duration * duration;
          A(rowIdx++, 4 * (i + 1) + 1) = -1;
        }

        if (i != N - 2) {
          // y_i''(x_{i+1}) = y_{i+1}''(x_{i}) =>
          // 2*a2i + 6*a3i(x_{i+1} - x_i) = 2*a2{i+1}
          A(rowIdx, 4 * i + 2) = 2;
          A(rowIdx, 4 * i + 3) = 6 * duration;
          A(rowIdx++, 4 * (i + 1) + 2) = -2;
        }
      }

      // dx0
      A(rowIdx, 1) = 1;
      b(rowIdx++) = dydt0(j, k);

      // dx1
      A(rowIdx, 4 * (N - 2) + 1) = 1;
      A(rowIdx, 4 * (N - 2) + 2) = 2 * (T[N - 1] - T[N - 2]);
      A(rowIdx, 4 * (N - 2) + 3) =
          3 * (T[N - 1] - T[N - 2]) * (T[N - 1] - T[N - 2]);
      b(rowIdx++) = dydt1(j, k);

      assert(rowIdx == 4 * (N - 1));
      auto decomposition = A.colPivHouseholderQr();
      assert(decomposition.isInvertible());
      solution = decomposition.solve(b);

      for (size_t i = 0; i < N - 1; i++) {
        polynomials[i](j, k) = Polynomial<Scalar>(solution.segment<4>(4 * i));
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateCubicSpline(
    const std::vector<Scalar> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y) {
  if (!CheckSplineInputs(T, Y)) {
    throw std::runtime_error("invalid spline inputs");
  }
  size_t N = T.size();
  Eigen::Matrix<Scalar, rows, cols> dydt0, dydt1;
  dydt0 = (Y[1] - Y[0]) / (T[1] - T[0]);
  dydt1 = (Y[N - 1] - Y[N - 2]) / (T[N - 1] - T[N - 2]);
  return GenerateCubicSpline(T, Y, dydt0, dydt1);
}

template <typename Scalar, int rows, int cols>
PiecewisePolynomial<Scalar> GenerateCubicSpline(
    const std::vector<Scalar> &T,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Y,
    const std::vector<Eigen::Matrix<Scalar, rows, cols>> &Ydot) {
  if (!CheckSplineInputs(T, Y) || Y.size() != Ydot.size()) {
    throw std::runtime_error("invalid spline inputs");
  }

  size_t N = T.size();
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>>
      polynomials(N - 1);

  for (size_t t = 0; t < N - 1; t++) {
    if (Y[t].rows() != Ydot[t].rows() || Y[t].cols() != Ydot[t].cols()) {
      throw std::runtime_error("invalid spline inputs");
    }

    polynomials[t].resize(Y[t].rows(), Y[t].cols());
    Scalar a = T[t + 1] - T[t];
    Scalar b = a * a;
    Scalar c = b * a;
    for (size_t i = 0; i < Y[t].rows(); i++) {
      for (size_t j = 0; j < Y[t].cols(); j++) {
        Scalar c4 = Y[t](i, j);
        Scalar c3 = Ydot[t](i, j);
        Scalar c1 = 1. / b * (Ydot[t + 1](i, j) - c3 -
            2. / a * (Y[t + 1](i, j) - c4 - a * c3));
        Scalar c2 = 1. / b * (Y[t + 1](i, j) - c4 - a * c3 - c * c1);
        // constant term comes first
        Eigen::Matrix<Scalar, 4, 1> coeffs(c4, c3, c2, c1);
        polynomials[t](i, j) = Polynomial<Scalar>(coeffs);
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
}

// poses and vels are task space pose and vel
// QPLocomotionPlanSettings.m: BodyMotionData.from_body_xyzexp_and_xyzexpdot
// -> BodyMotionData.m: pchipDeriv
//    -> drake/util/pchipDeriv.m
template <typename Scalar>
PiecewisePolynomial<Scalar> GenerateCubicCartesianSpline(
    const std::vector<Scalar> &times,
    const std::vector<Eigen::Matrix<Scalar, 7, 1>> &poses,
    const std::vector<Eigen::Matrix<Scalar, 7, 1>> &vels) {
  assert(times.size() == poses.sizes());
  assert(times.size() == vels.sizes());
  assert(times.size() >= 2);

  size_t T = times.size();
  std::vector<Eigen::Matrix<Scalar, 6, 1>> expmap(poses.size());
  std::vector<Eigen::Matrix<Scalar, 6, 1>> expmap_dot(poses.size());

  Eigen::Matrix<Scalar, 4, Eigen::Dynamic> quat(4, T);
  Eigen::Matrix<Scalar, 4, Eigen::Dynamic> quat_dot(4, T);
  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> exp(3, T);
  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> exp_dot(3, T);

  for (size_t t = 0; t < T; t++) {
    quat.col(t) = poses[t].tail(4);
    quat_dot.col(t) = vels[t].tail(4);
  }
  quat2expmapSequence(quat, quat_dot, exp, exp_dot);

  for (size_t t = 0; t < times.size(); t++) {
    expmap[t].head(3) = poses[t].head(3);
    expmap[t].tail(3) = exp.col(t);
    expmap_dot[t].head(3) = vels[t].head(3);
    expmap_dot[t].tail(3) = exp_dot.col(t);
  }

  // need to do the closestExpmap
  for (size_t t = 1; t < times.size(); t++) {
    Eigen::Matrix<Scalar, 3, 1> w_closest;
    Eigen::Matrix<Scalar, 3, 3> dw_closest_dw;
    Eigen::Matrix<Scalar, 3, 1> w1 = expmap[t - 1].tail(3);
    Eigen::Matrix<Scalar, 3, 1> w2 = expmap[t].tail(3);
    closestExpmap1(w1, w2, w_closest, dw_closest_dw);
    expmap[t].tail(3) = w_closest;
    expmap_dot[t].tail(3) = dw_closest_dw * expmap_dot[t].tail(3);
  }

  return GenerateCubicSpline(times, expmap, expmap_dot);
}
