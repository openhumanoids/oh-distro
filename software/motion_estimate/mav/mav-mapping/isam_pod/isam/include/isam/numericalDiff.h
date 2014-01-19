/**
 * @file numericalDiff.h
 * @brief Numerical differentiation.
 * @author Michael Kaess
 * @version $Id: numericalDiff.h 2826 2010-08-20 03:17:43Z kaess $
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

#include "Vector.h"
#include "Matrix.h"

namespace isam {

// Abstract class to enforce interface for function object.
class Function {
public:
  virtual ~Function() {}
  virtual Vector evaluate(const Vector&) const = 0;
};

/**
 * Takes a general vector valued function and returns the
 * Jacobian at the linearization point given by x0.
 * @param func Function object with evaluation function that takes and returns vectors.
 * @param x0 Linearization point.
 * @return Matrix containing the Jacobian of func, with
 *         dim(y) rows and dim(x) columns, where y=func(x).
 */
Matrix numericalDiff(const Function& func, Vector x0);

}
