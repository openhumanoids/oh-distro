#ifndef DRC_CONTROL_EXPONENTIAL_FORM_HPP_
#define DRC_CONTROL_EXPONENTIAL_FORM_HPP_
#include "drake/Polynomial.h"
#include <Eigen/Dense>

class ExponentialForm {
  private: 
    double m_a;
    double m_b;
    double m_c;

  public:
    ExponentialForm(double a, double b, double c) {
      m_a = a;
      m_b = b;
      m_c = c;
    }

    Polynomial taylorExpand(int degree) const;

    double value(double t) const;
};

#endif