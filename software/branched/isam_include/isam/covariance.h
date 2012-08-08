/**
 * @file covariance.h
 * @brief Recovery of marginal covariance matrix.
 * @author Michael Kaess
 * @version $Id: covariance.h 4688 2011-06-08 13:26:07Z hordurj $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <vector>
#include <utility> // pair
#include <list>
#include <Eigen/Dense>

#include "SparseMatrix.h"

namespace isam {

typedef std::vector< std::vector<int> > index_lists_t;

typedef std::vector< std::pair<int, int> > entry_list_t;

/**
 * Takes a list of variable indices, and returns the marginal covariance matrix.
 * @param R Sparse factor matrix.
 * @param index_lists List of lists of indices; a block will be recovered for each list.
 * @param debug Optional parameter to print timing information.
 * @param step Optional parameter to print statistics (default=-1, no stats printed).
 * @return List of dense marginal covariance matrices.
 */
std::list<Eigen::MatrixXd> cov_marginal(const SparseMatrix& R,
                                        const index_lists_t& index_lists,
                                        bool debug=false, int step=-1);

/**
 * Takes a list of pairs of integers and returns the corresonding
 * entries of the covariance matrix.
 * @param R Sparse factor matrix.
 * @param entry_lists List of pairs of integers refering to covariance matrix entries.
 * @param debug Optional parameter to print timing information.
 * @param step Optional parameter to print statistics (default=-1, no stats printed).
 * @return List of doubles corresponding to the requested covariance matrix entries.
 */
std::list<double> cov_marginal(const SparseMatrix& R,
                               const entry_list_t& entry_list);

}
