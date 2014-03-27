/**
 * @file Slam.cpp
 * @brief SLAM implementation using iSAM
 * @author Michael Kaess
 * @author Hordur Johannsson
 * @version $Id: Slam.cpp 3204 2010-10-06 16:02:07Z kaess $
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

#include <iomanip>
#include <vector>
#include <map>
#include <list>

#include "isam/util.h"
#include "isam/Batch.h"
#include "isam/SparseSystem.h"
#include "isam/Matrix.h"
#include "isam/covariance.h"

#include "isam/Slam.h"

using namespace std;

namespace isam {

// for numbering of factors and nodes
int Node::_next_id = 0;
int Factor::_next_id = 0;

// for getting correct starting positions in matrix for each node,
// only needed after removing nodes
void Slam::update_starts() {
  int start = 0;
  const list<Node*>& nodes = get_nodes();
  for (list<Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
    Node* node = *it;
    node->_start = start;
    start += node->dim();
  }
}

void Slam::access_vectors(const Vector* delta_x, const Vector* x, const Vector* x0, Vector* get_x, Vector* get_x0) {
  int start = 0;
  // add difference to current estimates in nodes
  const list<Node*>& nodes = get_nodes();
  for (list<Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
    Node* node = *it;
    Vector v  = node->vector();
    Vector v0 = node->vector0();
    for (int c=0; c<node->dim(); c++) {
      int idx = start+c;
      if (delta_x) {
        int trans_idx;
        if (idx >= _R.num_cols()) {
          trans_idx = idx;
        } else {
          trans_idx = _R.a_to_r()[idx];
        }
        v.set(c, v0(c) - (*delta_x)(trans_idx));
      }
      if (x) {
        v.set(c, (*x)(idx));
      }
      if (x0) {
        v0.set(c, (*x0)(idx));
      }
      if (get_x) {
        (*get_x).set(idx,  v(c));
      }
      if (get_x0) {
        (*get_x0).set(idx, v0(c));
      }
    }
    if (delta_x || x) {
      node->update(v);
    }
    if (x0) {
      node->update0(v0);
    }
    start += node->dim();
  }
}

void Slam::update_estimate(const Vector& delta_x) {
  access_vectors(&delta_x, NULL, NULL);
}

void Slam::set_estimate(const Vector& x) {
  access_vectors(NULL, &x, NULL);
}

Vector Slam::get_estimate() {
  Vector x(_dim_nodes);
  access_vectors(NULL, NULL, NULL, &x);
  return x;
}

void Slam::update_linearization() {
  const list<Node*>& nodes = get_nodes();
  for (list<Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
    Node* node = *it;
    Vector v = node->vector();
    node->update0(v);
  }
}

void Slam::set_linearization(const Vector& x0) {
  access_vectors(NULL, NULL, &x0);
}

Vector Slam::get_linearization() {
  Vector x0(_dim_nodes);
  access_vectors(NULL, NULL, NULL, NULL, &x0);
  return x0;
}

SparseSystem Slam::jac_func(const Vector& x0) {
  Vector x_orig = get_linearization();
  set_linearization(x0);
  SparseSystem jac = jacobian();
  set_linearization(x_orig);
  return jac;
}

Slam::Slam()
  : Graph(),
    _step(0), _prop(Properties()), _require_batch(false), _cost_func(NULL),
    _batch(Batch()), _dim_nodes(0), _dim_measure(0),
    _num_new_measurements(0), _num_new_rows(0), _R(1,1)
{}

Slam::~Slam()
{}

void Slam::save(const string fname) const {
  ofstream out(fname.c_str(), ios::out | ios::binary);
  require(out, "Slam.save: Cannot open output file.");
  write(out);
  out.close();
}

void Slam::add_node(Node* node) {
  Graph::add_node(node);
  _dim_nodes += node->dim();
}

void Slam::add_factor(Factor* factor) {
  // adds itself to factor lists of adjacent nodes; also initialized linked nodes if necessary
  factor->initialize_internal();
  // needed to change cost function
  factor->init_ptr(&_cost_func);
  Graph::add_factor(factor);
  _num_new_measurements++;
  _num_new_rows += factor->dim();
  _dim_measure += factor->dim();
}

void Slam::remove_node(Node* node) {
  // make a copy, as the original will indirectly be modified below in remove_factor()
  list<Factor*> factors = node->factors(); 
  for (list<Factor*>::iterator factor = factors.begin(); factor!=factors.end(); factor++) {
    remove_factor(*factor);
  }
  _dim_nodes -= node->dim();
  Graph::remove_node(node);
  _require_batch = true;
}

void Slam::remove_factor(Factor* factor) {
  vector<Node*> nodes = factor->nodes();
  for (vector<Node*>::iterator node = nodes.begin(); node!=nodes.end(); node++) {
    (*node)->remove_factor(factor);
  }
  _dim_measure -= factor->dim();
  Graph::remove_factor(factor);
  _require_batch = true;
}

void Slam::incremental_update() {
  // incremental update not possible after removing nodes or factors
  // (might change in the future)
  if (_require_batch) {
    batch_optimization_step();
  } else if (_num_new_measurements > 0) {
    SparseSystem jac_new = jacobian(_num_new_measurements);
    _num_new_measurements = 0;
    _num_new_rows = 0;
    for (int i=0; i<jac_new.num_rows(); i++) {
      SparseVector new_row = jac_new.get_row(i);
      _R.add_row_givens(new_row, jac_new.rhs()(i));
    }
  }
}

void Slam::solve() {
  // obtain difference by backsubstitution (note that rhs in R does
  // not change as we don't update the linearization point!)
  Vector delta_x = _R.solve();
  update_estimate(delta_x);
}

UpdateStats Slam::update() {
  UpdateStats stats;
  stats.batch = false;
  stats.solve = false;
  if (_step%_prop.mod_update == 0) {
    if (_step%_prop.mod_batch == 0) {
      // batch solve periodically to avoid fill-in
      if (!_prop.quiet) {
        cout << endl;
        cout << "step " << _step;
      }
      batch_optimization_step();
      stats.batch = true;
    } else {
      // for efficiency, incrementally update most of the time.
      if (!_prop.quiet) {
        cout << ".";
        fflush(stdout);
      }
      incremental_update();
      if (_step%_prop.mod_solve == 0) {
        stats.solve = true;
        solve();
      }
    }
  }
  _step++;
  stats.step = _step;
  return stats;
}

void Slam::batch_optimization_step() {
  _require_batch = false;
  // update linearization point x0 with current estimate x
  update_linearization();
  // prepare factorization
  SparseSystem jac = jacobian();
  _num_new_measurements = 0;
  _num_new_rows = 0;
  // factorization and new rhs based on new linearization point will be in _R
  Vector x0 = get_linearization();
  Vector x = _batch.gauss_newton_step(jac, x0, &_R); // modifies _R
  set_estimate(x);
}

int Slam::batch_optimization() {
  Vector x0 = get_estimate();
  int iterations = 0;
  Vector x;
    x = _batch.gauss_newton(this, x0, _R, _prop, &iterations); // modifies x0,_R
  set_linearization(x0);
  set_estimate(x);
  return iterations;
}

list<Matrix> Slam::marginal_covariance(const node_lists_t& node_lists) {
  update_starts();

  vector < vector<int> > index_lists(node_lists.size());

  if (_R.num_rows()>1) { // skip if _R not calculated yet (eg. before batch step)
    int n=0;
    const int* trans = _R.a_to_r();
    for (node_lists_t::const_iterator it_list = node_lists.begin();
        it_list != node_lists.end();
        it_list++, n++) {
      // assemble list of indices
      vector<int> indices;
      for (list<Node*>::const_iterator it = it_list->begin(); it!=it_list->end(); it++) {
        int start = (*it)->_start;
        for (int i=0; i<(*it)->_dim; i++) {
          index_lists[n].push_back(trans[start+i]);
        }
      }
    }
    return cov_marginal(_R, index_lists);
  }
  list<Matrix> empty_list;
  return empty_list;
}

Matrix Slam::marginal_covariance(const list<Node*>& nodes) {
  node_lists_t node_lists;
  node_lists.push_back(nodes);
  return marginal_covariance(node_lists).front();
}

Vector Slam::weighted_errors(const Vector& x) {
  update_starts();
  // actual assembly of error vector follows
  int rows = _dim_measure;
  Vector error(rows);
  int row = 0;
  const list<Factor*>& factors = get_factors();
  for (list<Factor*>::const_iterator it = factors.begin(); it!=factors.end(); it++) {
    Factor* factor = *it;
    // assemble vector x
    int dim = 0;
    for (unsigned int n=0; n<factor->nodes().size(); n++) {
      dim += factor->nodes()[n]->dim();
    }
    Vector x_node(dim);
    int pos = 0;
    for (unsigned int n=0; n<factor->nodes().size(); n++) {
      Node* node = factor->nodes()[n];
      int start = node->_start;
      int size = node->dim();
      for (int i=0; i<size; i++) {
        x_node.set(pos, x(start+i));
        pos++;
      }
    }
    Vector err = factor->error(x_node); // already includes sqrtinf!
    for (int r=0; r<err.num_rows(); r++) {
      error.set(row+r, err(r));
    }
    row += factor->dim();
  }
  return error;
}

double Slam::weighted_sum_of_squared_errors() {
  return weighted_errors(get_estimate()).norm2();
}


double Slam::normalized_chi2() {
  return weighted_sum_of_squared_errors() / (double)(_dim_measure - _dim_nodes);
}

const SparseSystem& Slam::get_R() const {
  return _R;
}

SparseSystem Slam::jacobian(int last_n) {
  update_starts();
  // actual assembly of Jacobian
  int num_rows = _dim_measure;
  if (last_n > 0) {
    num_rows = _num_new_rows;
  }
  SparseVector* rows[num_rows];
  Vector rhs(num_rows);
  int row = 0;
  const list<Factor*>& factors = get_factors();
  list<Factor*>::const_iterator it = factors.begin();
  if (last_n != -1) {
    // skip all entries except for last_n
    for (int n = num_factors(); n>last_n; n--, it++);
  }
  for (; it!=factors.end(); it++) {
    Factor* factor = *it;
    Jacobian jac = factor->jacobian_internal(_prop.force_numerical_jacobian);
    for (int r=0; r<jac.residual.num_rows(); r++) {
      rhs.set(row+r, jac.residual(r));
      rows[row+r] = new SparseVector(jac.dimtotal()); // do not delete, will be pulled into SparseSystem below
    }
    for (Terms::iterator it=jac.terms.begin(); it!=jac.terms.end(); it++) {
      int offset = it->node()->_start;
      int c = it->term().num_cols();
      int nr = it->term().num_rows();
      int o = 0;
      double* v = it->term().rd();
      for (int r=0; r<nr; r++) {
        rows[row+r]->set(offset, v+o, c);
        o += c;
      }
    }
    row += factor->dim();
  }
  return SparseSystem(num_rows, _dim_nodes, rows, rhs);
}

void Slam::print_stats() {
  double nnz = _R.nnz();
  double dim = _dim_nodes;
  double per_col = nnz/dim;
  double fill_in = nnz/(dim*dim);
  cout << "iSAM statistics:" << endl;
  cout << "  Normalized chi-square value: " << normalized_chi2() << endl;
  cout << "  Weighted sum of squared errors: " << weighted_sum_of_squared_errors() << endl;
  cout << "  Number of nodes: " << num_nodes() << endl;
  cout << "  Number of factors: " << num_factors() << endl;
  cout << "  Number of variables: " << _dim_nodes << endl;
  cout << "  Number of measurements: " << _dim_measure << endl;
  cout << "  Number of non-zero entries: " << nnz << endl;
  cout << "    per column: " << per_col << endl;
  cout << "    fill in: " << fill_in << "%" << endl;
}

}
