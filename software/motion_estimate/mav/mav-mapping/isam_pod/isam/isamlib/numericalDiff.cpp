/**
 * @file numericalDiff.cpp
 * @brief Numerical differentiation.
 * @author Michael Kaess
 * @version $Id: numericalDiff.cpp 2921 2010-08-27 04:23:38Z kaess $
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

#include "isam/Vector.h"
#include "isam/Matrix.h"

#include "isam/numericalDiff.h"

#define SYMMETRIC

const double epsilon = 0.0001;

namespace isam {

Matrix numericalDiff(const Function& func, Vector x0) {
  Vector y0 = func.evaluate(x0);
  int m = y0.num_rows();
  int n = x0.num_rows();
  Matrix Jacobian(m,n); // result has one column per variable
  for (int j=0; j<n; j++) {
    Vector x_plus = x0;
    x_plus.set(j, x_plus(j) + epsilon);
    Vector y_plus = func.evaluate(x_plus);
    Vector x_minus = x0;
#ifdef SYMMETRIC
    x_minus.set(j, x_minus(j) - epsilon);
    Vector y_minus = func.evaluate(x_minus);
    // seems unnecessary, but significantly improves accuracy as it
    // takes into account that floating point cannot represent any
    // number, see NR p186
    double real_2epsilon = x_plus(j) - x_minus(j);
    Vector diff = y_plus - y_minus;
    Vector dxj = diff / real_2epsilon;
#else
    Vector dxj = (y_plus - y0) / (x_plus(j) - x0(j));
#endif
    for (int i=0; i<m; i++) {
      Jacobian.set(i, j, dxj(i));
    }
  }
  return Jacobian;
}

}
