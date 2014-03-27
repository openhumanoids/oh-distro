/**
 * @file SparseSystem.cpp
 * @brief Adds rhs functionality to sparse matrix for iSAM.
 * @author Michael Kaess
 * @version $Id: SparseSystem.cpp 2921 2010-08-27 04:23:38Z kaess $
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

#include <string>
#include <cstring> // memset()
#include <fstream>
#include <iostream>
#include <cmath>

#include "isam/Matrix.h"
#include "isam/util.h"

#include "isam/SparseSystem.h"

using namespace std;

namespace isam {

SparseSystem::SparseSystem(int num_rows, int num_cols) : OrderedSparseMatrix(num_rows, num_cols), _rhs(Vector(num_rows)) {
}

SparseSystem::SparseSystem(const SparseSystem& mat) : OrderedSparseMatrix(mat), _rhs(mat._rhs) {
}

SparseSystem::SparseSystem(const SparseSystem& mat, int num_rows, int num_cols, int first_row, int first_col) :
    OrderedSparseMatrix(mat, num_rows, num_cols, first_row, first_col), _rhs(first_row, num_rows, mat._rhs) {
}

SparseSystem::SparseSystem(int num_rows, int num_cols, SparseVector_p* rows, const Vector& rhs) :
    OrderedSparseMatrix(num_rows, num_cols, rows)
{
  _rhs = rhs;
}

SparseSystem::~SparseSystem() {
}

const SparseSystem& SparseSystem::operator= (const SparseSystem& mat) {
  if (this==&mat)
    return *this;

  OrderedSparseMatrix::operator=(mat);
  _rhs = mat._rhs;

  return *this;
}

void SparseSystem::apply_givens(int row, int col, double* c_givens, double* s_givens) {
  double c, s;
  SparseMatrix::apply_givens(row, col, &c, &s);
  // modify rhs
  double r1 = _rhs(col);
  double r2 = _rhs(row);
  _rhs.set(col, c*r1 - s*r2);
  _rhs.set(row, s*r1 + c*r2);
}

void SparseSystem::append_new_rows(int num) {
  OrderedSparseMatrix::append_new_rows(num);
  _rhs.add_new_rows(num);
}

void SparseSystem::add_row(const SparseVector& new_row, double new_r) {
  ensure_num_cols(new_row.last()+1);

  append_new_rows(1);
  int row = num_rows() - 1;
  _rhs.set(row, new_r);
  set_row(row, new_row);
}

int SparseSystem::add_row_givens(const SparseVector& new_row, double new_r) {
  // set new row (also translates according to current variable ordering)
  add_row(new_row, new_r);
  int count = 0;

  int row = num_rows() - 1; // last row
  int col = get_row(row).first(); // first entry to be zeroed
  while (col>=0 && col<row) { // stop when we reach the diagonal
    apply_givens(row, col);
    count++;
    col = get_row(row).first();
  }
  if (get_row(row).nnz()==0) {
    // need to remove the new row as it is now empty
    remove_row();
    // and the rhs needs to be cut accordingly
    _rhs = Vector(0, row, _rhs);
  }

  return count;
}

Vector SparseSystem::solve() const {
  require(num_rows() >= num_cols(), "SparseSystem::solve: cannot solve system, not enough constraints");
  require(num_rows() == num_cols(), "SparseSystem::solve: system not triangular");
  int n = num_cols();
  Vector result(n);
  for (int row=n-1; row>=0; row--) {
    const SparseVector& rowvec = get_row(row);
    // start with rhs...
    double terms = _rhs(row);
    // ... and subtract already solved variables multiplied with respective coefficient from R
    for (SparseVectorIter iter(rowvec); iter.valid(); iter.next()) {
      double v;
      int col = iter.get(v);
      if (row!=col) { // skip diagonal entry
        terms = terms - result(col)*v;
      }
    }
    // divide result by diagonal entry of R
    double diag = rowvec(row);
    result.set(row, terms / diag);
  }
  return result;
}

}
