/**
 * @file Factor.h
 * @brief Graph factor for iSAM.
 * @author Michael Kaess
 * @version $Id: Factor.h 3160 2010-09-26 20:10:11Z kaess $
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

#include <vector>
#include <string>

#include <math.h> // for sqrt

#include "util.h"
#include "Element.h"
#include "Vector.h"
#include "Node.h"
#include "numericalDiff.h"

namespace isam {

typedef double (*cost_func_t)(double);

// Elementary Jacobian for one specific variable/node, potentially containing multiple measurement rows.
class Term {
  Node* _node;
  Matrix _term;
public:
  Node* node() {return _node;}
  Matrix& term() {return _term;}
  Term(Node* node, const Matrix& term) : _node(node), _term(term) {}
  Term(Node* node, const double * term, int r, int c) : _node(node), _term(r,c,term) {}
};

typedef std::list<Term> Terms;

// Jacobian consisting of multiple blocks.
class Jacobian {
  int _dimtotal;
public:
  Terms terms;
  //const 
  Vector residual;
  Jacobian() : _dimtotal(0), residual()  {}
  Jacobian(Vector& residual) : _dimtotal(0), residual(residual) {}
  inline Jacobian(const double * residual, int r) : _dimtotal(0), residual(r,residual) {}
  void add_term(Node* node, const Matrix& term) {
    terms.push_back(Term(node, term));
  _dimtotal += node->dim();
  }

  inline void add_term(Node* node, const double* term, int r, int c) {
    terms.push_back(Term(node, term, r, c));
  _dimtotal += node->dim();
  }
  
  int dimtotal() const { return _dimtotal; }
};

// Factor of the graph of measurements between Nodes.
class Factor : public Element, Function {
  friend std::ostream& operator<<(std::ostream& output, const Factor& e) {
    e.write(output);
    return output;
  }

  cost_func_t *ptr_cost_func;

  static int _next_id;
protected:
  const Matrix _sqrtinf;
  std::vector<Node*> _nodes; // list of nodes affected by measurement
public:
  virtual Vector error(const Vector& x) const {
    // partition vector according to nodes
    std::vector<Vector> xs(_nodes.size());
    int pos = 0;
    for (unsigned int i=0; i<_nodes.size(); i++) {
      int size = _nodes[i]->dim();
      xs[i] = Vector(pos, size, x);
      pos += size;
    }
    // actual evaluation of error
    Vector err = _sqrtinf * basic_error(xs);
    // optional modified cost function
    if (*ptr_cost_func) {
      for (int i=0; i<err.size(); i++) {
        double val = err(i);
        err.set(i, ((val>=0)?1.:(-1.)) * sqrt((*ptr_cost_func)(val)));
      }
    }
    return err;
  }
  std::vector<Node*> nodes() {return _nodes;}
  Factor(const char* name, int dim, const Matrix& sqrtinf) : Element(name, dim), ptr_cost_func(NULL), _sqrtinf(sqrtinf) {
#ifndef NDEBUG
    // all lower triagular entries below the diagonal must be 0
    for (int r=0; r<_sqrtinf.num_rows(); r++) {
      for (int c=0; c<r; c++) {
        require(_sqrtinf(r,c)==0, "Factor::Factor: sqrtinf must be upper triangular!");
      }
    }
#endif
    _id = _next_id++;
  }
  virtual ~Factor() {}
  virtual void initialize() = 0;
  virtual void initialize_internal() {
    for (unsigned int i=0; i<_nodes.size(); i++) {
      _nodes[i]->add_factor(this);
    }
    initialize();
  }
  virtual void init_ptr(cost_func_t* ptr) {ptr_cost_func = ptr;}
  virtual Vector basic_error(const std::vector<Vector>& x) const = 0;
  virtual const Matrix& sqrtinf() const {return _sqrtinf;}
  Vector evaluate(const Vector& x) const {
    return error(x);
  }
  virtual Jacobian jacobian_internal(bool force_numerical) {
    if (force_numerical) {
      // ignore any symbolic derivative provided by user
      return Factor::jacobian();
    } else {
      return jacobian();
    }
  }
  // can be replaced by symbolic derivative by user
  virtual Jacobian jacobian() {
    Vector x0 = _nodes[0]->vector0(); // Vector x0 yields a vector with one entry!
    for (unsigned int i=1; i<_nodes.size(); i++) { // note: skips first entry
      x0 = x0 ^ _nodes[i]->vector0();
    }
    Matrix H = numerical_jacobian(x0);
    Vector r = error(x0);
    Jacobian jac(r);
    int position = 0;
    int n_measure = dim();
    for (unsigned int i=0; i<_nodes.size(); i++) {
      int n_var = _nodes[i]->dim();
      Matrix Hi = Matrix(0, position, n_measure, n_var, H);
      position += n_var;
      jac.add_term(_nodes[i], Hi);
    }
    return jac;
  }
  virtual void write(std::ostream &out) const {
    Element::write(out);
    for (unsigned int i=0; i<_nodes.size(); i++) {
      if (_nodes[i]) {
        out << " " << _nodes[i]->unique_id();
      }
    }
  }

private:
  virtual Matrix numerical_jacobian(const Vector& x) {
    return numericalDiff(*this, x);
  }

};

}
