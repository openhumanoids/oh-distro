/**
 * @file Vector.h
 * @brief Basic dense vector library.
 * @author Michael Kaess
 * @version $Id: Vector.h 3160 2010-09-26 20:10:11Z kaess $
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

#include "Matrix.h"
#include "util.h"

namespace isam {

class Vector : public Matrix {
public:
  /**
   * Create vector with one zero entry.
   */
  Vector();

  /**
   * Convert matrix to vector.
   * @param rhs Matrix that must have only one column.
   */
  Vector(const Matrix& rhs);

  /**
   * Copy constructor.
   * @param rhs Matrix to copy from.
   */
  Vector(const Vector& rhs);

  /**
   * Constructor based on double array.
   * @param r Number of entries.
   * @param mat Double array with vector values.
   */
  Vector(int r, const double * const mat = NULL);

  /**
   * Create sub-vector by constructor.
   * @param first First entry to copy.
   * @param num Number of entries to copy.
   * @param vec Vector to copy from.
   */
  Vector(int first, int num, const Vector& vec);

  /**
   * Destructor.
   */
  ~Vector();

  /**
   * Return number of rows of vector.
   */
  int size() const;

  /**
   * Assignment operator.
   */
  const Vector& operator= (const Vector& rhs);

  /**
   * Read entry.
   * @param r Row of entry.
   * @return Entry.
   */
  const double& operator()(int r) const {
    require(r>=0 && r<num_rows(), "Vector::() index outside vector.");
    return(_data[r]);
  }

  /**
   * Add vectors.
   */
  const Vector operator+(const Vector& rhs) const {return (Vector)Matrix::operator+(rhs);}

  /**
   * Subtract vectors.
   */
  const Vector operator-(const Vector& rhs) const {return (Vector)Matrix::operator-(rhs);}

  /**
   * Unary minus, negate sign of all vector entries.
   */
  const Vector operator-() const {return (Vector)Matrix::operator-();}

  /**
   * Multiply vectors element wise, vectors must be same length.
   */
  const Vector operator*(const Vector& rhs) const;

  /**
   * Dot product.
   */
  double dot(const Vector& rhs) const;

  /**
   * Create matrix with this vector as diagonal.
   */
  const Matrix diag_matrix() const;

  /**
   * Set vector entry.
   * @param r Row of entry.
   * @param val New value of entry.
   */
  inline void set(int r, double val) {
    require(r>=0 && r<num_rows(), "Vector::set() index outside vector.");
    _data[r] = val; 
  }

  /**
   * Element wise absolute value.
   */
  const Vector abs() const;

  /**
   * Return maximum of all entries of vector.
   */
  double max() const;

  /**
   * Return maximum of all absolute numbers in vector.
   */
  double norm_inf() const;

  /**
   * Return squared norm of vector (sum of squares).
   */
  double norm2() const;

  /**
   * Return vector norm (square root of sum of squares).
   */
  double norm() const;

  /**
   * Insert new elements into the vector, initialized with 0.
   * @param num Number of new entries.
   * @param pos Position to insert: 0 means front, -1 or num_rows means at the end.
   */
  void add_new_rows(int num, int pos = -1);
};

/**
 * Construct vector of given entries. Takes r additional entries.
 * CAUTION: Make sure to explicitly specify floats ("1.0" instead of "1").
 * @param r Number of rows of vector.
 */
Vector make_Vector(int r, ...); // make sure to explicitly specify floats ("1.0" instead of "1")

/**
 * Multiply matrix with vector.
 * @param lhs Matrix.
 * @param rhs Vector.
 * @return lhs*rhs.
 */
const Vector operator*(const Matrix& lhs, const Vector& rhs);

/**
 * Divide vector by scalar.
 * @param lhs Vector.
 * @param rhs Scalar.
 * @return lhs/rhs.
 */
const Vector operator/(const Vector& lhs, double rhs);

/**
 * Multiple scalar with vector.
 * @param lhs Scalar.
 * @param rhs Vector.
 * @return lhs*rhs.
 */
const Vector operator*(double lhs, const Vector& rhs);

}
