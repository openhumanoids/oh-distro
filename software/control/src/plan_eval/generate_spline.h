#pragma once

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/util/drakeGeometryUtil.h"

template <typename Derived1, typename Derived2>
void closestExpmap1(
    const Eigen::MatrixBase<Derived1>& expmap1,
    const Eigen::MatrixBase<Derived2>& expmap2,
    Eigen::Matrix<typename Derived1::Scalar, 3, 1>& w2_closest, 
    Eigen::Matrix<typename Derived1::Scalar, 3, 3>& dw2_closest) {
  using namespace Eigen;
  using namespace std;
  static_assert(
      Derived1::RowsAtCompileTime == 3 && Derived1::ColsAtCompileTime == 1,
      "Wrong size.");
  static_assert(
      Derived2::RowsAtCompileTime == 3 && Derived2::ColsAtCompileTime == 1,
      "Wrong size.");
  static_assert(
      std::is_same<typename Derived1::Scalar, typename Derived2::Scalar>::value,
      "Scalar types don't match.");
  typedef typename Derived1::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real Real;

  Real expmap1_norm = expmap1.norm();
  Real expmap2_norm = expmap2.norm();
  Eigen::Matrix<Scalar, 3, 1> ret;
  if (expmap2_norm < NumTraits<Scalar>::epsilon()) {
    if (expmap1_norm > NumTraits<Scalar>::epsilon()) {
      auto expmap1_axis = (expmap1 / expmap1_norm).eval();
      auto expmap1_round = round(expmap1_norm / (2 * M_PI));
      w2_closest = expmap1_axis * expmap1_round * 2 * M_PI;
      dw2_closest = Eigen::Matrix<typename Derived1::Scalar, 3, 3>::Zero();
      return;
    } else {
      w2_closest = expmap2;
      dw2_closest = Eigen::Matrix<typename Derived1::Scalar, 3, 3>::Identity();
      return; 
    }
  } else {
    auto expmap2_axis = (expmap2 / expmap2_norm).eval();
    Eigen::Matrix<Scalar,3,3> dw2_axis_dw2 = 
        (expmap2_norm * Eigen::Matrix<Scalar,3,3>::Identity() 
          - (expmap2 * expmap2.transpose()).value() / expmap2_norm) / (expmap2_norm * expmap2_norm);
    auto expmap2_closest_k =
        ((expmap2_axis.transpose() * expmap1).value() - expmap2_norm) /
        (2 * M_PI);
    auto expmap2_closest_k1 = floor(expmap2_closest_k);
    auto expmap2_closest_k2 = ceil(expmap2_closest_k);
    // doesn't match the matlab code /drake/util/closestExpmap.m
    //auto expmap2_closest_k2 = expmap2_closest_k1 + 1.0;
    auto expmap2_closest1 =
        (expmap2 + 2 * expmap2_closest_k1 * M_PI * expmap2_axis).eval();
    auto expmap2_closest2 =
        (expmap2 + 2 * expmap2_closest_k2 * M_PI * expmap2_axis).eval();
    if ((expmap2_closest1 - expmap1).norm() <
        (expmap2_closest2 - expmap1).norm()) {
      w2_closest = expmap2_closest1;
      dw2_closest = Eigen::Matrix<typename Derived1::Scalar, 3, 3>::Identity()
        + 2 * dw2_axis_dw2 * expmap2_closest_k1 * M_PI;
      return;
    } else {
      w2_closest = expmap2_closest2;
      dw2_closest = Eigen::Matrix<typename Derived1::Scalar, 3, 3>::Identity() 
        + 2 * dw2_axis_dw2 * expmap2_closest_k2 * M_PI;
      return;
    }
  }
}
  


template <typename Scalar> struct SimplePose {
  Eigen::Matrix<Scalar, 3, 1> lin;
  Eigen::Quaternion<Scalar> rot;
};

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

// copied from matlab source code and wikipedia
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
  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(N-1);

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
        Scalar duration = T[i+1] - T[i];

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
        polynomials[i](j, k) = Polynomial<Scalar>(solution.segment<4>(4*i));
      }
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, T);
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

// poses and vels are task space pose and vel
// QPLocomotionPlanSettings.m:BodyMotionData.from_body_xyzexp_and_xyzexpdot
// -> BodyMotionData.m:pchipDeriv
//    -> drake/util/pchipDeriv.m
template <typename Scalar> PiecewisePolynomial<Scalar> GenerateCubicCartesianSpline(const std::vector<Scalar> &times, const std::vector<SimplePose<Scalar>> &poses, const std::vector<SimplePose<Scalar>> &vels) {
  assert(times.size() == poses.sizes());
  assert(times.size() == vels.sizes());
  assert(times.size() >= 2);

  size_t T = times.size();
  std::vector<Eigen::Matrix<Scalar, 6, 1>> expmap(poses.size()), expmap_dot(poses.size());
  Eigen::Matrix<Scalar, 3, 1> tmp, tmpd;
  for (size_t t = 0; t < times.size(); t++) {
    quat2expmapSequence(poses[t].rot.coeffs(), vels[t].rot.coeffs(), tmp, tmpd);
    expmap[t].head(3) = poses[t].lin;
    expmap[t].tail(3) = tmp;
    expmap_dot[t].head(3) = vels[t].lin;
    expmap_dot[t].tail(3) = tmpd;
  }

  // need to do the closestExpmap
  for (size_t t = 1; t < times.size(); t++) {
    Eigen::Matrix<Scalar, 3, 1> w_closest;
    Eigen::Matrix<Scalar, 3, 3> dw_closest;
    Eigen::Matrix<Scalar, 3, 1> w1 = expmap[t-1].tail(3);
    Eigen::Matrix<Scalar, 3, 1> w2 = expmap[t].tail(3);
    closestExpmap1(w1, w2, w_closest, dw_closest);
    expmap[t].tail(3) = w_closest;
    expmap_dot[t].tail(3) = dw_closest * expmap_dot[t].tail(3);
  }

  std::vector<Eigen::Matrix<Polynomial<Scalar>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(T-1);
  // copy drake/util/pchipDeriv.m
  for (size_t t = 0; t < T-1; t++) {
    polynomials[t].resize(6, 1);
    Scalar a = times[t+1] - times[t];
    Scalar b = a * a;
    Scalar c = b * a;
    for (size_t i = 0; i < 6; i++) {
      Scalar c4 = expmap[t][i];
      Scalar c3 = expmap_dot[t][i];
      Scalar c1 = 1. / b * (expmap_dot[t][i] - c3 - 2. / a * (expmap[t+1][i] - c4 -a * c3));
      Scalar c2 = 1. / b * (expmap[t+1][i] - c4 - a * c3 - c * c1);
      // constant term comes first
      Eigen::Matrix<Scalar, 4, 1> coeffs(c4, c3, c2, c1);
      polynomials[t](i, 0) = Polynomial<Scalar>(coeffs);
    }
  }

  return PiecewisePolynomial<Scalar>(polynomials, times);
}
         
