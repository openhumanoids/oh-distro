#include "generate_spline.h" 

namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
};
 
PiecewisePolynomial<double> GenerateCubicCartesianSpline(const std::vector<double> &times, const std::vector<SimplePose> &poses, const std::vector<SimplePose> &vels) {
  assert(times.size() == poses.sizes());
  assert(times.size() == vels.sizes());
  assert(times.size() > 0);

  size_t T = times.size();
  std::vector<Eigen::Vector6d> expmap(poses.size()), expmap_dot(poses.size());
  Eigen::Vector3d tmp, tmpd;
  for (size_t t = 0; t < times.size(); t++) {
    quat2expmapSequence(poses[t].rot.coeffs(), vels[t].rot.coeffs(), tmp, tmpd);
    // TODO: need to do the closest expmap mumble
    expmap[t].segment<3>(0) = poses[t].lin;
    expmap[t].segment<3>(3) = tmp;
    expmap_dot[t].segment<3>(0) = vels[t].lin;
    expmap_dot[t].segment<3>(3) = tmpd;
  }

  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> polynomials(T-1);
  // copy drake/util/pchipDeriv.m
  for (size_t t = 0; t < T-1; t++) {
    polynomials[t].resize(6, 1);
    double a = times[t+1] - times[t];
    double b = a * a;
    double c = b * a;
    for (size_t i = 0; i < 6; i++) {
      double c4 = expmap[t][i];
      double c3 = expmap_dot[t][i];
      double c1 = 1. / b * (expmap_dot[t][i] - c3 - 2. / a * (expmap[t+1][i] - c4 -a * c3));
      double c2 = 1. / b * (expmap[t+1][i] - c4 - a * c3 - c * c1);
      // constant term comes first
      Eigen::Vector4d coeffs(c4, c3, c2, c1);
      polynomials[t](i, 0) = Polynomial<double>(coeffs);
    }
  }

  return PiecewisePolynomial<double>(polynomials, times);
}

       
