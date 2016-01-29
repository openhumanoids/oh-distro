#ifndef DRC_CONTROL_EXPONENTIAL_FORM_HPP_
#define DRC_CONTROL_EXPONENTIAL_FORM_HPP_
#include "drake/util/Polynomial.h"
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

    Polynomial<double> taylorExpand(int degree) const;

    ExponentialForm operator+(const double x) {
      ExponentialForm expform(this->m_a, this->m_b, this->m_c + x);
      return expform;
    }
    ExponentialForm operator-(const double x) {
      ExponentialForm expform(this->m_a, this->m_b, this->m_c - x);
      return expform;
    };

    double value(double t) const;
};

#endif