#include "ExponentialForm.hpp"
#include <Eigen/Dense>

using namespace Eigen;

Polynomial ExponentialForm::taylorExpand(int degree) const {
  VectorXd coefs = VectorXd::Zero(degree+1);
  coefs(0) = m_a + m_c;

  double factorial = 1.0;
  for (int d=1; d < degree + 1; d++) {
    factorial *= d;
    coefs(d) = m_a * std::pow(m_b, d) / factorial;
  }

  return Polynomial(coefs);
}

double ExponentialForm::value(double t) const {
  return m_a * exp(m_b * t) + m_c;
}