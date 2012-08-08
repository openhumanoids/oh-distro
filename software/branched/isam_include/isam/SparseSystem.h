/**
 * @file SparseSystem.h
 * @brief Adds rhs functionality to sparse matrix for iSAM.
 * @author Michael Kaess
 * @version $Id: SparseSystem.h 4133 2011-03-22 20:40:38Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <Eigen/Dense>

#include "OrderedSparseMatrix.h"

namespace isam {

class SparseSystem : public OrderedSparseMatrix {
  Eigen::VectorXd _rhs;
public:
  SparseSystem(int num_rows, int num_cols);
  SparseSystem(const SparseSystem& mat);
  SparseSystem(const SparseSystem& mat, int num_rows, int num_cols, int first_row = 0, int first_col = 0);
  SparseSystem(int num_rows, int num_cols, SparseVector_p* rows, const Eigen::VectorXd& rhs);
  virtual ~SparseSystem();
  const SparseSystem& operator= (const SparseSystem& mat);

  const Eigen::VectorXd& rhs() const {return _rhs;}
  void set_rhs(const Eigen::VectorXd& rhs) {_rhs = rhs;}

  // overridden functions

  /**
   * Note: While rows are passed in, the rhs is required to already
   * contain the new entry - necessary because we cannot change the
   * signature of the function.
   */
  void apply_givens(int row, int col, double* c_givens = NULL, double* s_givens = NULL);

  void append_new_rows(int num);

  // new functions

  /**
   * Insert a new row
   * @param new_row The new sparse measurement row to add.
   * @param new_r New right hand side entry.
   */
  virtual void add_row(const SparseVector& new_row, double new_r);

  /**
   * Insert a new measurement row and triangulate using Givens rotations
   * @param new_row The new sparse measurement row to add.
   * @param new_r New right hand side entry.
   * @return Number of Givens rotations applied (for analysis).
   */
  virtual int add_row_givens(const SparseVector& new_row, double new_r);

  /**
   * Solve equation system by backsubstitution.
   * @return Solution for x in Rx=b'
   */
  virtual Eigen::VectorXd solve() const;

};

}
