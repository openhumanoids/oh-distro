/**
 * @file Cholesky.h
 * @brief Cholesky batch factorization using SuiteSparse by Tim Davis.
 * @author Michael Kaess
 * @version $Id: Cholesky.h 6377 2012-03-30 20:06:44Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <Eigen/Dense>

#include "SparseSystem.h"

namespace isam {

class Cholesky {
public:
  virtual ~Cholesky() {}

  /**
   * Factorize a given system Ax=b and optionally solve.
   * @param Ab SparseSystem with measurement Jacobian A and right hand side b.
   * @param delta Optional parameter to return solution of system.
   * @param lambda Adds elements to diagonal of information matrix A'A before
   *        factorization, used for Levenberg-Marquardt algorithm.
   */
  virtual void factorize(const SparseSystem& Ab, Eigen::VectorXd* delta = NULL, double lambda = 0.) = 0;

  /**
   * Copy R into a SparseSystem data structure (expensive, so can be
   * avoided during batch factorization).
   * @param R SparseSystem that upon return will contain the R factor.
   */
  virtual void get_R(SparseSystem& R) = 0;

  /**
   * Access the variable ordering used for Cholesky factorization.
   * @return Pointer to variable ordering.
   */
  virtual int* get_order() = 0;

  static Cholesky* Create();

protected:
  Cholesky() {}
};

}
