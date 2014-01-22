/**
 * @file covariance.cpp
 * @brief Recovery of marginal covariance matrix.
 * @author Michael Kaess
 * @version $Id: covariance.h 2736 2010-08-04 20:24:05Z kaess $
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

#include <vector>
#include <list>

#include "Matrix.h"
#include "SparseMatrix.h"

namespace isam {

typedef std::vector< std::vector<int> > index_lists_t;

/**
 * Takes a list of variables, and returns the marginal covariance matrix.
 * @param R Sparse factor matrix.
 * @param index_lists List of lists of indices; a block will be recovered for each list.
 * @param debug Optional parameter to print timing information.
 * @param step Optional parameter to print statistics (default=-1, no stats printed).
 * @return Dense marginal covariance matrix.
 */
std::list<Matrix> cov_marginal(const SparseMatrix& R,
                               const index_lists_t& index_lists,
                               bool debug=false, int step=-1);

}
