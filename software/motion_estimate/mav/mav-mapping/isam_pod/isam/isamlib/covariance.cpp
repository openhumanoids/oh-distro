/**
 * @file covariance.cpp
 * @brief Recovery of marginal covariance matrix, for details see Kaess09ras.
 * @author Michael Kaess
 * @version $Id: covariance.cpp 2921 2010-08-27 04:23:38Z kaess $
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

#include <vector>
#include "isam/covariance.h"
#include "isam/util.h"

using namespace std;

#if 1
#include <tr1/unordered_map>
using namespace tr1;
#else
#include "boost/unordered_map.hpp"
using namespace boost;
#endif

namespace isam {

typedef unordered_map<int, double> umap;

umap entries;

// precalculated diagonal inverses
vector<double> _diag;

// recovering rows is expensive, buffer results
vector<SparseVector> rows;

// avoid having to cleanup buffers each time by explicitly marking entries as valid
vector<unsigned int> rows_valid;

// avoid having to cleanup valid entries by using different indices each time
unsigned int current_valid = 1;

// stats
int num_calc;

void prepare(const SparseMatrix& R) {
  // reset hash table
  entries.clear();
  // speedup row access
  int n = R.num_cols();
  rows.resize(n);
  rows_valid.resize(n);
  current_valid++; // invalidate previous entries
  // wrapped back to 0? then we do have to reset the table
  if (current_valid==0) {
    for (int i=0; i<n; i++) {
      rows_valid[i] = 0;
    }
    current_valid = 1;
  }
  // precalculate inverses
  _diag.resize(n);
  for (int i=0; i<n; i++) {
    _diag[i] = 1. / R(i,i);
  }
  // stats
  num_calc = 0;
}

const SparseVector& get_row(const SparseMatrix& R, int i) {
  if (rows_valid[i] != current_valid) {
    // retrieve and store row
    rows[i] = R.get_row(i);
    rows_valid[i] = current_valid;
  }
  return rows[i];
}

// for recursive call
double recover(const SparseMatrix& R, int n, int i, int l);

inline double sum_j(const SparseMatrix& R, int n, int l, int i) {
  double sum = 0;
  for (SparseVectorIter iter(get_row(R, i)); iter.valid(); iter.next()) {
    double rij;
    int j = iter.get(rij);
    if (j!=i) {
      double lj;
      if (j>l) {
        lj = recover(R, n, l, j);
      } else {
        lj = recover(R, n, j, l);
      }
      sum += rij * lj;
    }
  }
  return sum;
}

double recover(const SparseMatrix& R, int n, int i, int l) {
  if (i>l) {int tmp=i; i=l; l=tmp;}
  int id = i*n + l; // flatten index for hash table
  umap::iterator it = entries.find(id);
  double res;
  if (it == entries.end()) {
    //printf("calculating entry %i,%i\n", i, l);
    // need to calculate entry
    if (i==l) {
      res = _diag[l] * (_diag[l] - sum_j(R, n, l, l));
    } else {
      res = -sum_j(R, n, l, i) * _diag[i];
    }
    // insert into hash
    pair<int, double> entry(id, res);
    entries.insert(entry);
    num_calc++;
  } else {
    //printf("retrieved entry %i,%i\n", i, l);
    // retrieve value from hash
    res = it->second;
  }
  return res;
}

list<Matrix> cov_marginal(const SparseMatrix& R, const index_lists_t& index_lists, bool debug, int step) {
  prepare(R);
  list<Matrix> Cs;

  // debugging
  int requested = 0;
  double t0 = tic();

  for (unsigned int i=0; i<index_lists.size(); i++) {

    const vector<int>& indices = index_lists[i];
    unsigned int n_indices = indices.size();
    Matrix C(n_indices, n_indices);
    // recover entries of marginal cov
    int n = R.num_cols();
    for (int r=n_indices-1; r>=0; r--) {
      for (int c=n_indices-1; c>=r; c--) {
        C.set(r, c, recover(R, n, indices[r], indices[c]));
      }
    }
    for (unsigned int r=1; r<n_indices; r++) {
      for (unsigned int c=0; c<r; c++) {
        C.set(r, c, C(c,r));
      }
    }
    Cs.push_back(C);
    requested += indices.size()*indices.size();
  }

  if (debug) {
    double time = toc(t0);
    // timing
    printf("cov: %i calculated for %i requested in %f seconds\n",
           num_calc, requested, time);
    if (step>=0) {
      // stats for gnuplot
      printf("ggg %i %i %i %i %i %i %f ",
             step,
             R.num_cols(), // side length
             R.num_cols()*R.num_cols(), // #entries dense
             R.nnz(), // #entries sparse
             num_calc, // #entries calculated
             requested, // #entries requested
             time); // #execution time
    }
  }

  return Cs;
}

}
