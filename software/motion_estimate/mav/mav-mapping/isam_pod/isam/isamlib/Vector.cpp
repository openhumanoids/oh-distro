/**
 * @file Vector.cpp
 * @brief Basic dense vector library.
 * @author Michael Kaess
 * @version $Id: Vector.cpp 3216 2010-10-19 14:50:36Z kaess $
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

#include <stdarg.h>
#include <cstring>
#include <iomanip>
#include <cmath>

#include "isam/util.h"

#include "isam/Vector.h"

using namespace std;

namespace isam {

Vector::Vector() : Matrix() {
}

Vector::Vector(const Matrix& rhs) : Matrix(rhs) {
  require(rhs.num_cols()==1, "Vector::Vector Illegal typecast, matrix has more than one column");
}

Vector::Vector(const Vector& rhs) : Matrix(rhs) {
}

Vector::Vector(int r, const double * const mat) : Matrix(r, 1, mat) {
}

Vector::Vector(int first, int num, const Vector& vec) : Matrix(num, 1) {
  memcpy(_data, &(vec._data[first]), num*sizeof(double));
}

Vector::~Vector(){
}

int Vector::size() const {
  return num_rows();
}

const Vector& Vector::operator= (const Vector& rhs) {
  if (this==&rhs)
    return *this;
  delete[] _data;
  _init(rhs.num_rows(), 1);
  memcpy(_data, rhs._data, num_rows()*sizeof(double));
  return *this;
}

const Vector Vector::operator*(const Vector& rhs) const {
  require(num_rows()==rhs.num_rows(), "Vector::operator* implements element-wise product, vectors must have same length.");
    Vector V(num_rows());
    for (int r=0; r<num_rows(); r++) {
      V.set(r, operator()(r)*rhs(r));
    }
    return V;
}

double Vector::dot(const Vector& rhs) const {
  require(num_rows() == rhs.num_rows(), "Vector::dot() vectors differ in length.");
  double ret = 0.;
  for (int r=0; r<num_rows(); r++) {
    ret += operator()(r) * rhs(r);
  }
  return ret;
}

const Matrix Vector::diag_matrix() const {
  int n = num_rows();
  Matrix M = Matrix::zeros(n);
  for (int r=0; r<n; r++) {
    M.set(r, r, operator()(r));
  }
  return M;
}

const Vector Vector::abs() const {
  int n = num_rows();
  Vector v(n);
  for (int r=0; r<n; r++) {
    v.set(r, fabs(operator()(r)));
  }
  return v;
}

double Vector::max() const {
  double m = operator()(0);
  for (int r=0; r<num_rows(); r++) {
    m = fmax(m, operator()(r));
  }
  return m;
}

double Vector::norm_inf() const {
  double res = fabs(operator()(0));
  for (int r=0; r<num_rows(); r++) {
    res = fmax(res, fabs(operator()(r)));
  }
  return res;
}

double Vector::norm2() const {
  double res = 0.;
  for (int r=0; r<num_rows(); r++) {
    double v = operator()(r);
    res += v*v;
  }
  return res;
}

double Vector::norm() const {
  return std::sqrt(norm2());
}

void Vector::add_new_rows(int num, int pos) {
  if (pos<0) { //--
    pos = num_rows();
  }
  require(num>0, "Vector::add_new_rows() need to add at least one row.");
  require(pos>=0 && pos<=num_rows(), "Vector::add_new_rows() pos out of range");
  int new_num_rows = _num_rows + num;
  double* new_data = new double[new_num_rows];
  int first = pos;
  int last = _num_rows-pos;
  memcpy(new_data, _data, sizeof(double)*first);
  memcpy(new_data+(pos+num), _data+(pos), sizeof(double)*last);
  for (int i=pos; i<pos+num; i++) {
    new_data[i] = 0;
  }
  _num_rows = new_num_rows;
  delete[] _data;
  _data = new_data;
}

// make sure to explicitly specify floats ("1.0" instead of "1")
Vector make_Vector(int r, ...) {
  require(r>0, "Vector::Vector requires positive number of entries.");
  Vector V(r);
  va_list vs;
  va_start(vs, r); // r is the last known parameter of this function
  for (int _r = 0; _r<r; _r++) {
    V.set(_r, va_arg(vs, double));
  }
  va_end(vs);
  return V;
}

const Vector operator*(const Matrix& lhs, const Vector& rhs) {
  return (lhs * static_cast<Matrix>(rhs));
}

const Vector operator/(const Vector& lhs, double rhs) {
  int n = lhs.num_rows();
  double inv_rhs = 1. / rhs; // only one division, more efficient
  Vector V(n);
  for (int r=0; r<n; r++) {
    V.set(r, lhs(r) * inv_rhs);
  }
  return V;
}

const Vector operator*(double lhs, const Vector& rhs) {
  return (lhs * (Matrix)rhs);
}

}
