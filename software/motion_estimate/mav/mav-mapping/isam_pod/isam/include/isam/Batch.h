/**
 * @file Batch.h
 * @brief General non-linear batch equation solvers.
 * @author Michael Kaess
 * @version $Id: Batch.h 2920 2010-08-27 01:08:18Z kaess $
 *
 * Copyright (C) 2009-2010 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson and John J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "Properties.h"
#include "Vector.h"
#include "SparseSystem.h"
#include "Cholesky.h"

namespace isam {

class BatchFunction {
public:
  virtual ~BatchFunction() {}
  virtual SparseSystem jac_func(const Vector&) = 0;
  virtual Vector weighted_errors(const Vector&) = 0;
};

class Batch {

public:

  Batch() {
    _cholesky = Cholesky::Create();
  }

  ~Batch() {
    delete _cholesky;
  }

  /**
   * Perform one step of Gauss Newton (ie. quadratic solver with Hessian
   * obtained from Jacobian.
   * @param jac Jacobian and rhs.
   * @param x0 Current estimate (linearization point).
   * @param R Optionally returned R factor matrix and modified rhs.
   * @return Updated estimate.
   */
  Vector gauss_newton_step(const SparseSystem& jac, const Vector& x0, SparseSystem* R);

  /**
   * Perform Gauss Newton optimization until convergence
   * @param function Class that contains the target function and its Jacobian for evaluation.
   * @param x0 Initial estimate, also returns final linearization point.
   * @param R Factor matrix and modified rhs.
   * @param prop Properties including stopping criteria max_iterations and epsilon.
   * @param num_iterations Upon return, contains number of iterations performed.
   * @return Solution vector.
   */
  Vector gauss_newton(BatchFunction* function, Vector& x0, SparseSystem& R, const Properties& prop, int* num_iterations = NULL);


private:

  Cholesky* _cholesky;



};

}

