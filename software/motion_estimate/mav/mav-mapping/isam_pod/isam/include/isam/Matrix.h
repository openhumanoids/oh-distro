/**
 * @file Matrix.h
 * @brief Basic dense matrix library.
 * @author Michael Kaess
 * @version $Id: Matrix.h 3160 2010-09-26 20:10:11Z kaess $
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

#include <iostream>
#include "util.h"

namespace isam {

class Matrix {
protected:
  double* _data;
  int _num_rows, _num_cols;

private:
  void _init(int r, int c);

public:
  double* rd() {return _data;}
  
  /**
   * Default constructor, yields matrix with a single entry set to 0.
   */
  Matrix();

  /**
   * Copy constructor.
   */
  Matrix(const Matrix& rhs);

  /**
   * Constructor from array of doubles.
   * @param r Number of rows of matrix.
   * @param c Number of colums of matrix.
   * @param mat Array with r times c entries.
   */
  Matrix(int r, int c, const double * const mat = NULL);

  /**
   * Copy parts of a matrix.
   * @param r0 First row to copy.
   * @param c0 First column to copy.
   * @param dr Number of rows to copy.
   * @param dc Number of columns to copy.
   * @param mat Matrix to copy from.
   */
  Matrix(int r0, int c0, int dr, int dc, const Matrix& mat);

  /**
   * Destructor.
   */
  ~Matrix();

  /**
   * Matrix content as array in row form.
   * @return Pointer to array of doubles, starting with first row.
   */
  const double* data() const {return _data;}

  /**
   * Print content of matrix to screen.
   */
  void print() const;

  /**
   * Read entry.
   * @param r Row of entry.
   * @param c Column of entry.
   * @return Entry.
   */
  inline const double& operator()(int r, int c) const {
    require(r>=0 && c>=0 && r<_num_rows && c<_num_cols, "Matrix::() index outside matrix.");
    return(_data[r*_num_cols+c]);
  }

  /**
   * Assignment operator.
   */
  const Matrix& operator= (const Matrix& rhs);

  /**
   * Collect matrices.
   * @param rhs Matrix to append on the right of this.
   * @return This with rhs appended to the right.
   */
  const Matrix operator| (const Matrix& rhs) const;

  /**
   * Stack matrices.
   * @param rhs Matrix to append below this.
   * @return This with rhs appended below.
   */
  const Matrix operator^(const Matrix& rhs) const;

  /**
   * Add matrices.
   */
  const Matrix operator+(const Matrix& rhs) const;

  /**
   * Subtract matrices.
   */
  const Matrix operator-(const Matrix& rhs) const;

  /**
   * Unary minus, negate sign of all matrix entries.
   */
  const Matrix operator-() const;

  /**
   * Multiply matrices.
   */
  const Matrix operator*(const Matrix& rhs) const;

  /**
   * Set matrix entry.
   * @param r Row of entry.
   * @param c Column of entry.
   * @param val New value of entry.
   */
  void set(int r, int c, double val);

  /**
   * Transpose matrix.
   */
  const Matrix transpose() const;

  /**
   * Calculate trace (sum of diagonal elements).
   */
  double trace() const;

  /**
   * Return number of rows of matrix.
   */
  inline int num_rows() const {return _num_rows;}

  /**
   * Return number of columns of matrix.
   */
  inline int num_cols() const {return _num_cols;}

  /**
   * Create matrix with all 0 entries.
   * @param r Number of rows.
   * @param c Number of columns.
   * @return r by c matrix with all 0 entries.
   */
  static Matrix zeros(int r, int c);

  /**
   * Create square matrix with all 0 entries.
   * @param n Side length of square matrix.
   * @return n by n matrix with all 0 entries.
   */
  static Matrix zeros(int n);

  /**
   * Identity matrix.
   * @param n Side length of square matrix.
   * @return n by n matrix with all diagonal entries 1.
   */
  static Matrix eye(int n);

  /**
   * Create nth unit vector with r rows.
   * @param r Number of rows in vector.
   * @param nth Number of entry to set to 1.
   * @return Zero vector with size r, with nth entry set to 1.
   */
  static Matrix unit(int r, int nth);

  friend class Vector;
};

/**
 * Construct matrix of given entries. Takes r times c additional parameters.
 * CAUTION: Make sure to explicitly specify floats ("1.0" instead of "1").
 * @param r Number of rows of matrix.
 * @param c Number of columns of matrix.
 */
Matrix make_Matrix(int r, int c, ...);

/**
 * Collect matrices, see operator|.
 */
void collect(const Matrix &A, const Matrix &B, Matrix &C);

/**
 * Stack matrices, see operator^.
 */
void stack(const Matrix &A, const Matrix &B, Matrix &C);

/**
 * Write matrix to stream.
 */
std::ostream& operator<< (std::ostream& s, const Matrix &M);

/**
 * Multiple scalar with matrix.
 * @param lhs Scalar.
 * @param rhs Matrix.
 * @return lhs*rhs.
 */
const Matrix operator*(double lhs, const Matrix& rhs);

/**
 * Divide matrix by scalar.
 * @param lhs Matrix.
 * @param rhs Scalar.
 * @return lhs/rhs.
 */
const Matrix operator/(const Matrix& lhs, double rhs);

}
