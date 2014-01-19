/**
 * @file Properties.h
 * @brief Properties class for easy access to internal parameters.
 * @author Michael Kaess
 * @version $Id: Properties.h 3165 2010-09-27 04:08:25Z kaess $
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

namespace isam {

/**
 * User changeable default parameters.
 */
class Properties {
  // default copy constructor and assignment operator in use
public:
  // print additional information if true
  bool verbose;
  // omit all textual output if true
  bool quiet;

  // ignore any symbolic derivatives provided in factors
  bool force_numerical_jacobian;

  // maximum residual for convergence test
  double epsilon;
  // maximum number of iterations for LM
  int max_iterations;

  // only update R matrix/solution/batch every mod_update steps
  int mod_update;
  // batch solve with variable reordering and relinearization every mod_batch steps
  int mod_batch;
  // for incremental steps, solve by backsubstitution every mod_solve steps
  int mod_solve;

  // default parameters
  Properties() :
    verbose(false),
    quiet(false),

    force_numerical_jacobian(false),

    epsilon(1e-4),
    max_iterations(20),

    mod_update(1),
    mod_batch(100),
    mod_solve(1)
  {}
};

}
