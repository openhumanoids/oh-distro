#ifndef __poly_hpp__
#define __poly_hpp__

#include <Eigen/Dense>
#include <iostream>
#include <eigen_utils/eigen_utils.hpp>
#include <lcmtypes/planning_polynomial_t.h>

class Polynomial {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd coeffs;

  /**
   * default constructor, order 1 with 0 coefficient
   */
  Polynomial() :
      coeffs(1)
  {
    this->coeffs.setZero();
  }

  /**
   * order N with zero coefficients
   */
  Polynomial(int N) :
      coeffs(N + 1)
  {
    this->coeffs.setZero();
  }

  /**
   * construct with coefficients
   */
  template<typename Derived>
  Polynomial(const Eigen::MatrixBase<Derived> & coeffs) :
      coeffs(coeffs)
  {
  }

  Polynomial(const planning_polynomial_t * msg) :
      coeffs(Eigen::Map<const Eigen::VectorXd>(msg->coeffs, msg->N_poly))
  {
  }

  /**
   * evaluates the polynomial at t
   */
  double eval(double t);

  /**
   * evaluates derivative at t
   */
  double eval(double t, int derivative);

  /**
   * polulates derivative_values order deterimined by derivative_values.rows().
   * 0 indicates 0th derivative - eval(t) - and so on
   */
  template<typename Derived>
  double eval(double t, Eigen::MatrixBase<Derived> & derivative_values)
  {
    for (int dd = 0; dd < derivative_values.rows(); dd++) {
      derivative_values(dd) = this->eval(t, dd);
    }
  }

  /**
   * returns the derivative polynomial
   */
  Polynomial getDerivative();

  /**
   * scale the independent axis such that poly.eval(t)=poly_scaled(scale*t)
   */
  void scaleIndep(double scale)
  {
    double prod = 1;
    for (int ii = 1; ii < this->coeffs.rows(); ii++) {
      prod *= scale;
      this->coeffs(ii) /= prod;
    }
  }

  /**
   * scale the dependent axis such that scaling * poly.eval(t) = poly_scaled.eval(t)
   */
  void scaleDep(double scale)
  {
    this->coeffs *= scale;
  }

  void to_planning_polynomial_t(planning_polynomial_t * msg)
  {
    msg->N_poly = this->coeffs.rows();
    msg->coeffs = (double *) calloc(msg->N_poly, sizeof(double));
    memcpy(msg->coeffs, this->coeffs.data(), this->coeffs.rows() * sizeof(double));
  }

};

/**
 * Builds a matrix such that A*p = V, where V(ii) is the ii'th derivative at t=tau and p is a vector of polynomial coefficients
 *
 * The number of derivatives is given by A_derivative.rows(), and the order of the polynomial is A_derivative.cols()-1
 */
void polyGetDerivativeMatrix(double tau, Eigen::MatrixXd & A_derivative, double t_scale = 1.0);

/**
 * Builds a cost matrix such Q, that sum_dd der_costs(dd)*p^(dd)^T*Q*p^(dd) is the integral of p^2(t) from 0 to tau
 */
void polyGetCostMatrix(double tau, Eigen::MatrixXd & Q, const Eigen::VectorXd & der_costs);

/**
 * optimizes a polynomial subject to derivative constraints and t=0 and t=tau and cost on the integral of the squared derivatives
 *
 * the 0th derivative is the polynomial itself and so on
 */
Polynomial polyQuadDerOpt(double tau, const Eigen::VectorXd & der_0, const Eigen::VectorXd & der_final,
    const Eigen::VectorXd & der_costs, double * cost = NULL);

int sign(int v);

void polyQaudDerOptPiecewiseIndexMap(int N_extra_constraints, int D, int N_poly, int K,
    Eigen::VectorXi & index_kk_to_BR);

/**
 * Jointly optimizes a set of polynomials
 *
 * intermediate_der is a matrix with fixed derivatives occupying the first intermediate_ders_fixed rows
 * the rest of the rows are occupied by offset constraints.  Leave intermediate_ders_fixed = 0 to only specify offsets,
 * otherwise set it to the correct number to constrain derivatives.
 *
 */
void polyQuadDerOptPiecewise(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
    const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
    Polynomial * polys[], double * cost = NULL, int intermediate_ders_fixed = 0);

// Intermediate der first row is position constraints

std::ostream& operator<<(std::ostream& output, const Polynomial & poly);

#endif
