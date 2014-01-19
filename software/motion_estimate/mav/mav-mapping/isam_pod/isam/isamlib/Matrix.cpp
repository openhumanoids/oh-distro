/**
 * @file Matrix.cpp
 * @brief Basic dense matrix library.
 * @author Michael Kaess
 * @version $Id: Matrix.cpp 3160 2010-09-26 20:10:11Z kaess $
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

#include "isam/util.h"
#include "isam/Vector.h"

#include "isam/Matrix.h"

using namespace std;

namespace isam {

void collect(const Matrix &A, const Matrix &B, Matrix &C) {
  require(A.num_rows()==B.num_rows(), "Matrix::collect(): Requires same number of rows.");
  require(A.num_cols()+B.num_cols()==C.num_cols() && A.num_rows()==C.num_rows(), "Matrix::collect(): Wrong output matrix.");
  for (int r=0; r<A.num_rows(); r++) {
    for (int c=0; c<A.num_cols(); c++) {
      C.set(r, c, A(r,c));
    }
    for (int c=0; c<B.num_cols(); c++) {
      C.set(r, c+A.num_cols(), B(r,c));
    }
  }
}

void stack(const Matrix &A, const Matrix &B, Matrix &C) {
  require(A.num_cols()==B.num_cols(), "Matrix::stack(): Requires same number of columns.");
  require(A.num_rows()+B.num_rows()==C.num_rows() && A.num_cols()==C.num_cols(), "Matrix::stack(): Wrong output matrix.");
  for (int c=0; c<A.num_cols(); c++) {
    for (int r=0; r<A.num_rows(); r++) {
      C.set(r, c, A(r,c));
    }
    for (int r=0; r<B.num_rows(); r++) {
      C.set(r+A.num_rows(), c, B(r,c));
    }
  }
}

ostream& operator<< (ostream& s, const Matrix &M) {
  s << setprecision(4) << fixed;
  for (int r=0; r<M.num_rows(); r++) {
    for (int c=0; c<M.num_cols(); c++) {
      s << setw(9) << M(r,c) << ' ';
    }
    s << endl;
  }
  return s;
}

// member functions

void Matrix::_init(int r, int c) {
  _data = new double[c*r];
  _num_cols = c;
  _num_rows = r;
}

Matrix::Matrix() {
  _init(1,1);
  _data[0] = 0.;
}

Matrix::Matrix(const Matrix& rhs) {
  _init(rhs.num_rows(),rhs.num_cols());
  memcpy(_data, rhs._data, num_rows()*num_cols()*sizeof(double));
}

Matrix::Matrix(int r, int c, const double * const doubles) {
  require(r>=1 && c>=1, "Matrix::Matrix requires positive, non-zero number of entries.");
  _init(r,c);
  if (doubles) {
    memcpy(_data, doubles, r*c*sizeof(double));
  } else {
    for (int _r=0; _r<r; _r++) {
      for (int _c=0; _c<c; _c++) {
        _data[_r*c+_c] = 0.;
      }
    }
  }
}

Matrix::Matrix(int r0, int c0, int dr, int dc, const Matrix& mat) {
  require(r0>=0 && c0>=0 && r0<mat.num_rows() && c0<mat.num_cols(), "Matrix::Matrix r0/c0 outside matrix.");
  require(dr>0 && dc>0 && r0+dr<=mat.num_rows() && c0+dc<=mat.num_cols(), "Matrix::Matrix dr/dc outside matrix.");
  _init(dr,dc);
  for (int r=0; r<dr; r++) {
    for (int c=0; c<dc; c++) {
      _data[r*dc+c] = mat(r0+r, c0+c);
    }
  }
}

Matrix::~Matrix(){
  delete[] _data;
}

void Matrix::print() const {
  for (int r=0; r<num_rows(); r++) {
    for (int c=0; c<num_cols(); c++) {
      printf("%9.4f ", _data[r*num_cols()+c]);
    }
    printf("\n");
  }
}

const Matrix& Matrix::operator= (const Matrix& rhs) {
  if (this==&rhs)
    return *this;
  delete[] _data;
  _init(rhs.num_rows(), rhs.num_cols());
  memcpy(_data, rhs._data, num_rows()*num_cols()*sizeof(double));
  return *this;
}

const Matrix Matrix::operator| (const Matrix& rhs) const {
  Matrix C(num_rows(), num_cols()+rhs.num_cols());
  collect(*this, rhs, C);
  return C;
}

const Matrix Matrix::operator^ (const Matrix& rhs) const {
  Matrix C(num_rows()+rhs.num_rows(), num_cols());
  stack(*this, rhs, C);
  return C;
}

const Matrix Matrix::operator+(const Matrix& rhs) const {
  require(num_cols() == rhs.num_cols() && num_rows() == rhs.num_rows(),
          "Matrix::operator-: matrices incompatible.");
  Matrix M(num_rows(), num_cols());
  for (int r=0; r<M.num_rows(); r++) {
    for (int c=0; c<M.num_cols(); c++) {
      M.set(r, c, operator()(r,c) + rhs(r,c));
    }
  }
  return M;
}

const Matrix Matrix::operator-(const Matrix& rhs) const {
  require(num_cols() == rhs.num_cols() && num_rows() == rhs.num_rows(),
          "Matrix::operator-: matrices incompatible.");
  Matrix M(num_rows(), num_cols());
  for (int r=0; r<M.num_rows(); r++) {
    for (int c=0; c<M.num_cols(); c++) {
      M.set(r, c, operator()(r,c) - rhs(r,c));
    }
  }
  return M;
}

const Matrix Matrix::operator-() const {
  Matrix M(num_rows(), num_cols());
  for (int r=0; r<M.num_rows(); r++) {
    for (int c=0; c<M.num_cols(); c++) {
      M.set(r, c, -operator()(r,c));
    }
  }
  return M;
}

const Matrix Matrix::operator*(const Matrix& rhs) const {
  require(_num_cols == rhs.num_rows(), "Matrix::operator*: matrices incompatible.");
  Matrix M(_num_rows, rhs._num_cols);
  for (int r=0; r<_num_rows; r++) {
    for (int c=0; c<rhs._num_cols; c++) {
      double res = 0;
      for (int i=0; i<_num_cols; i++) {
        res += /*operator()(r,i)*/_data[r*_num_cols+i] * rhs._data[i*rhs._num_cols+c];
      }
      M.set(r, c, res);
    }
  }
  return M;
}

void Matrix::set(int r, int c, double val) {
  require(r>=0 && c>=0 && r<_num_rows && c<_num_cols, "Matrix::set() index outside matrix.");
  _data[r*_num_cols+c] = val;
}

const Matrix Matrix::transpose() const {
  Matrix M(num_cols(), num_rows());
  for (int r=0; r<num_rows(); r++) {
    for (int c=0; c<num_cols(); c++) {
      M.set(c, r, operator()(r,c));
    }
  }
  return M;
}

double Matrix::trace() const {
  require(num_rows()==num_cols(), "Matrix::trace: requires square matrix");
  double trace = 0.;
  for (int r=0; r<num_rows(); r++) {
    trace += operator()(r,r);
  }
  return trace;
}

Matrix Matrix::zeros(int r, int c) {
  require(r>=1 && c>=1, "Matrix::zeros: requires positive non-zero size.");
  Matrix M(r,c);
  for (int _r=0; _r<r; _r++) {
    for (int _c=0; _c<c; _c++) {
      M._data[_r*c+_c]=0.;
    }
  }
  return M;
}

Matrix Matrix::zeros(int n) {
  return zeros(n, n);
}

Matrix Matrix::eye(int n) {
  require(n>=1, "Matrix::eye: requires positive non-zero size.");
  Matrix M = Matrix::zeros(n);
  for (int i=0; i<n; i++) {
    M._data[i*M.num_cols()+i]=1.;
  }
  return M;
}

Matrix Matrix::unit(int r, int nth) {
  require(r>=1 && nth>=0 && r>nth, "Matrix::unit: wrong argument");
  Matrix M = Matrix::zeros(r,1);
  M.set(nth, 0, 1.);
  return M;
}

// make sure to explicitly specify floats ("1.0" instead of "1")
Matrix make_Matrix(int r, int c, ...) {
  require(r>=1 && c>=1, "Matrix::Matrix requires positive number of rows and columns.");
  Matrix M(r,c);
  va_list vs;
  va_start(vs, c); // c is the last known parameter of this function
  for (int _r = 0; _r<r; _r++) {
    for (int _c = 0; _c<c; _c++) {
      M.set(_r, _c, va_arg(vs, double));
    }
  }
  va_end(vs);
  return M;
}

const Matrix operator*(double lhs, const Matrix& rhs) {
  int m = rhs.num_rows();
  int n = rhs.num_cols();
  Matrix res(m, n);
  for (int r=0; r<m; r++) {
    for (int c=0; c<n; c++) {
      res.set(r, c, lhs*rhs(r,c));
    }
  }
  return res;
}

const Matrix operator/(const Matrix& lhs, double rhs) {
  return operator*(1./rhs, lhs);
}

}
