/**
 * @file Batch.cpp
 * @brief General non-linear batch equation solvers.
 * @author Michael Kaess
 * @version $Id: Batch.cpp 2921 2010-08-27 04:23:38Z kaess $
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

#include <cmath>

#include "isam/Vector.h"
#include "isam/SparseSystem.h"
#include "isam/Cholesky.h"

#include "isam/Batch.h"

using namespace std;

namespace isam {

Vector Batch::gauss_newton_step(const SparseSystem& jacobian,
    const Vector& x0, SparseSystem* R = NULL) {
  Vector delta;
  _cholesky->factorize(jacobian, &delta);
  if (R) {
    _cholesky->get_R(*R);
  }

  // delta has new ordering, but x0 default,
  // want to return result with default ordering
  Vector new_x = x0;
  int nrows = new_x.size();
  int reverse_order[nrows];
  const int* order = _cholesky->get_order();
  for (int i=0; i<nrows; i++) {
    reverse_order[order[i]] = i;
  }
  for (int i=0; i<nrows; i++) {
    new_x.set(i, new_x(i) - delta(reverse_order[i]));
  }

  return new_x;
}

Vector Batch::gauss_newton(BatchFunction* function,
    Vector& x0, SparseSystem& R, const Properties& prop,
    int* num_iterations) {
  int num_iter = 0;
  Vector x = x0;
  while(true) {
    x0 = x;
    num_iter++;
    SparseSystem jacobian = function->jac_func(x0);
    x = gauss_newton_step(jacobian, x0);
    double max_change = (x-x0).abs().max();
    if (!prop.quiet) {
      printf("Iteration %i: maximum change %g\n", num_iter, max_change);
    }
    if ((prop.max_iterations>0 && num_iter>=prop.max_iterations)
        || max_change<prop.epsilon) break;
  }
  if (num_iterations) {
    *num_iterations = num_iter;
  }
  _cholesky->get_R(R);
  return x;
}

}
