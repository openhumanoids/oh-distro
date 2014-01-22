/**
 * @file Cholesky.cpp
 * @brief Cholesky batch factorization using CHOLMOD by Tim Davis.
 * @author Michael Kaess
 * @version $Id: Cholesky.cpp 3204 2010-10-06 16:02:07Z kaess $
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

#include <string.h>

#include "isam/util.h"
#include "isam/SparseMatrix.h"
#include "isam/SparseSystem.h"

#include "isam/Cholesky.h"

#include "cs.h"
#include "cholmod.h"

// use CSparse instead of CHOLMOD (default)
//#define USE_CSPARSE

// use CSparse-QR instead of CSparse-Cholesky, much slower, only for testing
//#define USE_CSPARSE_QR

using namespace std;

namespace isam {

class CholeskyImpl : public Cholesky {
  cholmod_sparse* _L;
  cholmod_dense* _rhs;
  int* _order;

  cholmod_common Common;

public:

  CholeskyImpl() : _L(NULL), _rhs(NULL), _order(NULL) {
    cholmod_start(&Common);
  }

  virtual ~CholeskyImpl() {
    reset();
    cholmod_finish(&Common);
  }

  void factorize(const SparseSystem& Ab, Vector* delta = NULL) {
    tic("Cholesky");

    reset(); // make sure _L, _rhs, _order are empty

    cholmod_sparse* At = to_cholmod_transp(Ab);
    int nrow = At->ncol;
    int ncol = At->nrow;
//    cholmod_sparse* A = cholmod_transpose(At, 1, &Common);

    // Cholesky factorization
    // cholmod factors AA' instead of A'A - so we need to pass in At!
    cholmod_factor *L_factor;
#if 0
    // restrict ordering (default is trying several)
    Common.nmethods = 1;
    Common.method[0].ordering = CHOLMOD_COLAMD;
    Common.postorder = false;
#endif
    L_factor = cholmod_analyze(At, &Common);
    tic("cholmod_factorize");
    cholmod_factorize(At, L_factor, &Common);
    toc("cholmod_factorize");
    // make sure factorization is in correct format (LL, simplicial, packed, ordered)
    cholmod_change_factor(CHOLMOD_REAL, true, false, true, true, L_factor, &Common);

    // calculate new rhs (y in R'y = A'b)
    // note: original rhs is size nrow, Atb and new rhs are size ncol
    cholmod_dense* A_rhs = cholmod_zeros(nrow, 1, CHOLMOD_REAL, &Common);
    memcpy(A_rhs->x, Ab.rhs().data(), nrow*sizeof(double));
    cholmod_dense* Atb = cholmod_zeros(ncol, 1, CHOLMOD_REAL, &Common);
    double alpha[2] = {1., 0.}; // Atb = 1 * (At*A_rhs)
    double beta[2] = {0., 0.}; // + 0 * Atb
    cholmod_sdmult(At, 0, alpha, beta, A_rhs, Atb, &Common);
    // permute Atb according to L_factor
    cholmod_dense* Atb_perm = cholmod_solve(CHOLMOD_P, L_factor, Atb, &Common);
    // forward sub to obtain new (modified) rhs
    _rhs = cholmod_solve(CHOLMOD_L, L_factor, Atb_perm, &Common);

    // optionally solve the triangular system
    if (delta) {
      cholmod_dense* delta_ = cholmod_solve(CHOLMOD_Lt, L_factor, _rhs, &Common);
      *delta = Vector(_rhs->nrow, (double*)delta_->x);
      cholmod_free_dense(&delta_, &Common);
    }

    // create R/L with ordering and rhs
    // WARNING: L_factor becomes symbolic!! (numeric L is literally pulled out for efficiency)
    _order = new int[ncol];
    memcpy(_order, (int*)L_factor->Perm, ncol*sizeof(int));
    _L = cholmod_factor_to_sparse(L_factor, &Common);

    cholmod_free_dense(&Atb_perm, &Common);
    cholmod_free_dense(&Atb, &Common);
    cholmod_free_dense(&A_rhs, &Common);
    cholmod_free_factor(&L_factor, &Common);
//    cholmod_free_sparse(&A, &Common);
    cholmod_free_sparse(&At, &Common);

    toc("Cholesky");
  }

  void get_R(SparseSystem& R) {
    // we need R but have L, so the transpose works out fine
    of_cholmod_transp(_L, R, _order);
    R.set_rhs(Vector(_L->nrow, (double*)_rhs->x));
  }

  int* get_order() {
    return _order;
  }

private:

  void reset() {
    if (_L) cholmod_free_sparse(&_L, &Common);
    if (_rhs) cholmod_free_dense(&_rhs, &Common);
    if (_order) delete[] _order;
  }

  // internal, allocates new cholmod matrix and copies transpose of SparseSystem over
  // (SparseSystem is row-based, cholmod column-based)
  cholmod_sparse* to_cholmod_transp(const SparseSystem& A) {
    // note: num_cols/num_rows swapped for transpose
    cholmod_sparse* T = cholmod_allocate_sparse(A.num_cols(), A.num_rows(), A.nnz(),
                                                true, true, 0, CHOLMOD_REAL, &Common);

    int* p = (int*)T->p;
    int* i = (int*)T->i;
    double* x = (double*)T->x;
    int n = 0;
    *p = n;
    for (int row=0; row<A.num_rows(); row++) {
      const SparseVector& r = A.get_row(row);
      int nnz = r.nnz();
      // easy: CSparse and SparseVector indices are both 0-based
      r.copy_raw(i, x);
      i += nnz;
      x += nnz;
      n += nnz;
      p++;
      *p = n;
    }

    return T;
  }

  // internal, returns SparseSystem containing a copy of cholmod matrix transposed for efficiency
  void of_cholmod_transp(const cholmod_sparse* T, SparseSystem& A, int* order) {
    int nrow = T->ncol; // swapped for transpose
    int ncol = T->nrow;
    SparseVector_p rows[nrow];
    int *p = (int*)T->p;
    int *i = (int*)T->i;
    double* x = (double*)T->x;
    for (int row = 0; row < nrow; row++) {
      int nnz = *(p+1) - *p;
      rows[row] = new SparseVector(i, x, nnz);
      i += nnz;
      x += nnz;
      p++;
    }
    A.import_rows_ordered(nrow, ncol, rows, order);
  }

};

class CholeskyImplCSparse : public Cholesky {
  cs* _L;
  double* _rhs;
  int* _order;

public:

  CholeskyImplCSparse() : _L(NULL), _rhs(NULL), _order(NULL) {
  }

  virtual ~CholeskyImplCSparse() {
    reset();
  }

  void factorize(const SparseSystem& Ab, Vector* delta = NULL) {
    tic("Cholesky");

    reset(); // make sure _L, _rhs, _order are empty

    // QR with correct rhs
    cs* csAt = to_csparse_transp(Ab);
    cs* csA = cs_transpose(csAt, 1);
    int m = csA->m;
    int n = csA->n;
#ifdef USE_CSPARSE_QR
    // QR factorization, slower than Cholesky below
    // symbolic QR with reordering
    css *S = cs_sqr(3, csA, 1); // first argument: 0=no reoder, 3=reorder
    // numerical QR based on symbolic factorization
    csn *N = cs_qr(csA, S);
    // note: R factor in U for QR, variable ordering in S->q
    // note that L (N->U) is not square! taking subset below
    cs* csTemp = cs_spalloc(n, n, N->U->nzmax, 1, 0);
    memcpy(csTemp->p, N->U->p, (n+1) * sizeof(int));
    memcpy(csTemp->i, N->U->i, N->U->nzmax * sizeof(int));
    memcpy(csTemp->x, N->U->x, N->U->nzmax * sizeof(double));
    _L = cs_transpose(csTemp, 1);
    cs_spfree(csTemp);
    const int* p = S->q;
#else
    // Cholesky factorization
    cs* csAtA = cs_multiply(csAt, csA);
    css *S = cs_schol(1, csAtA);
    csn *N = cs_chol(csAtA, S);
    // straight copy
    _L = cs_spalloc(n, n, N->L->nzmax, 1, 0);
    memcpy(_L->p, N->L->p, (n+1) * sizeof(int));
    memcpy(_L->i, N->L->i, N->L->nzmax * sizeof(int));
    memcpy(_L->x, N->L->x, N->L->nzmax * sizeof(double));
    cs_spfree(csAtA);
    int* p = cs_pinv(S->pinv, csAtA->n);
#endif
    _order = new int[n];
    memcpy(_order, p, n*sizeof(int));
    // reorder rhs
    const double* Ab_rhs = Ab.rhs().data();
    // result Atb needs to be initialized - gets added!
    double *Atb = new double[m];
    for (int i=0; i<n; i++) {
      Atb[i] = 0.;
    }
    cs_gaxpy(csAt, Ab_rhs, Atb);

    // note that rhs is size n, Atb and B_rhs are longer (size m)
    _rhs = new double[n];
    for (int i=0; i<n; i++) {
      _rhs[i] = Atb[p[i]]; // permute Atb
    }

    // forward sub to include R
    cs_lsolve(_L, _rhs);

    // optionally solve the triangular system
    if (delta) {
      double* delta_ = new double[n];
      memcpy(delta_, _rhs, n*sizeof(double));
      cs_ltsolve(_L, delta_);
      *delta = Vector(n, delta_);
      delete[] delta_;
    }

    // clean up
#ifndef USE_CSPARSE_QR
    cs_free(p);
#endif
    cs_spfree(csA);
    cs_spfree(csAt);
    cs_sfree(S);
    cs_nfree(N);
    delete[] Atb;
  }

  void get_R(SparseSystem& R) {
    of_csparse_transp(_L, R, _order);
    R.set_rhs(Vector(_L->n, _rhs));
  }

  int* get_order() {
    return _order;
  }

private:

  void reset() {
    if (_L) cs_spfree(_L);
    if (_rhs) delete[] _rhs;
    if (_order) delete[] _order;
  }

  cs* to_csparse_transp(const SparseMatrix& A) const {
    // note: num_cols/num_rows swapped for transpose
    cs* T = cs_spalloc(A.num_cols(), A.num_rows(), A.nnz(), 1, 0);
    int* p = (int*)T->p;
    int* i = (int*)T->i;
    double* x = (double*)T->x;
    int n = 0;
    *p = n;
    for (int row=0; row<A.num_rows(); row++) {
      const SparseVector& r = A.get_row(row);
      int nnz = r.nnz();
      // easy: CSparse and SparseVector indices are both 0-based
      r.copy_raw(i, x);
      i += nnz;
      x += nnz;
      n += nnz;
      p++;
      *p = n;
    }

    return T;
  }

  void of_csparse_transp(const cs* T, SparseSystem& A, int* order) {
    int nrow = T->n; // swapped for transpose
    int ncol = T->m;
    SparseVector_p rows[nrow];
    int *p = (int*)T->p;
    int *i = (int*)T->i;
    double* x = (double*)T->x;
    for (int row = 0; row < nrow; row++) {
      int nnz = *(p+1) - *p;
      rows[row] = new SparseVector(i, x, nnz);
      i += nnz;
      x += nnz;
      p++;
    }
    A.import_rows_ordered(nrow, ncol, rows, order);
  }

};


Cholesky* Cholesky::Create() {
#ifdef USE_CSPARSE
  return new CholeskyImplCSparse();
#else
  return new CholeskyImpl();
#endif
}

}
