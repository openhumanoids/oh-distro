/**
 * @file Slam.h
 * @brief SLAM implementation using iSAM
 * @author Michael Kaess
 * @version $Id: Slam.h 3204 2010-10-06 16:02:07Z kaess $
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

#include <string>
#include <list>

#include "SparseSystem.h"
#include "Node.h"
#include "Factor.h"
#include "Graph.h"
#include "Properties.h"
#include "Batch.h"

namespace isam {

/**
 * Return type of Slam::update() to allow future extensions without
 * having to change the interface.
 */
class UpdateStats {
public:
  // current step number
  int step;

  // was batch performed?
  bool batch;

  // was the solution updated?
  bool solve;
};

/**
 * The actual SLAM interface.
 */
class Slam: public Graph, BatchFunction {
  // Graph prohibits copy construction and assignment operator

  int _step;

  Properties _prop;

public:

  //-- manipulating the graph -----------------------------

  /**
   * Default constructor.
   */
  Slam();

  /**
   * Destructor.
   */
  virtual ~Slam();

  /**
   * Returns a copy of the current properties.
   */
  Properties properties() {
    return _prop;
  }

  /**
   * Sets new properties.
   */
  void set_properties(Properties prop) {
    _prop = prop;
  }

  /**
   * Saves the graph (nodes and factors).
   * @param fname Filename with optional path to save graph to.
   */
  void save(const std::string fname) const;

  /**
   * Adds a node (variable) to the graph.
   * @param node Pointer to new node.
   */
  void add_node(Node* node);

  /**
   * Adds a factor (measurement) to the graph.
   * @param factor Pointer to new factor.
   */
  void add_factor(Factor* factor);

  /**
   * Removes a node (variable) and all adjacent factors from the graph.
   * Note that the node itself is not deallocated.
   * @param node Pointer to node.
   */
  void remove_node(Node* node);

  /**
   * Removes an factor (measurement) from the graph.
   * Note that the factor itself is not deallocated.
   * Be careful not to leave unconnected nodes behind.
   * @param factor Pointer to factor.
   */
  void remove_factor(Factor* factor);

  //-- solving the system -----------------------------

  /**
   * Update the graph by finding new solution; depending on properties
   * this might simply be a Givens update, could include a solve step,
   * or be a full batch step with reordering.
   * @return Update statistics.
   */
  virtual UpdateStats update();

  /**
   * Fully solve the system, iterating until convergence.
   * @return Number of iterations performed.
   */
  virtual int batch_optimization();

  //-- misc -----------------------------

  typedef std::list<std::list<Node*> > node_lists_t;

  /**
   * Calculates marginal covariance over a list of
   * lists. Significantly more efficient than calling
   * marginal_covariance multiple times with separate lists, as
   * intermediate results are being reused.
   * @param node_lists List of list of nodes.
   * @return List of marginal covariance matrix.
   */
  virtual std::list<Matrix> marginal_covariance(const node_lists_t& node_lists);

  /**
   * Calculates marginal covariance over a list of nodes.
   * @param nodes List of nodes.
   * return Marginal covariance matrix.
   */
  virtual Matrix marginal_covariance(const std::list<Node*>& nodes);

  /**
   * Calculates the normalized chi-square value (weighted sum of squared
   * errors divided by degrees of freedom [# measurements - # variables])
   * for the estimate x.
   */
  double normalized_chi2();

  /**
   * Weighted sum of squared errors at the current estimate.
   */
  double weighted_sum_of_squared_errors();

  /**
   * Returns the current factor matrix.
   */
  virtual const SparseSystem& get_R() const;

  /**
   * Returns the measurement Jacobian of the SLAM system.
   * @param last_n Only return Jacobians of last n measurements (default: -1, return all)
   * @return Measurement Jacobian.
   */
  virtual SparseSystem jacobian(int last_n = -1);

  /**
   * Print statistics for debugging.
   */
  virtual void print_stats();

private:

  /**
   * Evaluate the weighted non-squared error function for the estimate x.
   */
  Vector weighted_errors(const Vector& x);

  /**
   * Update estimate if delta_x present, and update linearization point if x0 present.
   */
  void access_vectors(const Vector* delta_x, const Vector* x0, const Vector* x,
      Vector* get_x = NULL, Vector* get_x0 = NULL);

  /**
   * Update the estimate x using the current linearization point and the
   * delta_x vector, based on the current ordering in R.
   */
  void update_estimate(const Vector& delta_x);

  /**
   * Set the estimate to the given vector x, using default odering.
   */
  void set_estimate(const Vector& x);

  /**
   * Return current estimate, using default ordering.
   */
  Vector get_estimate();

  /**
   * Set the linearization point x0 to the current estimate x.
   */
  void update_linearization();

  /**
   * Set the linearization point to the given vector x0, using default ordering.
   */
  void set_linearization(const Vector& x0);

  /**
   * Return current linearization point, using default ordering.
   */
  Vector get_linearization();

  /**
   * Fulfills BatchFunction requirement - needed to evaluate Jacobian in Batch.
   * @param x Evaluation point.
   * @return Jacobian at the evaluation point.
   */
  SparseSystem jac_func(const Vector& x);

  /**
   * Fulfills BatchFunction requirement - needed to evaluate the target function in Batch.
   * @param x Evaluation point.
   * @return Cost function at the evaluation point.
   */
  Vector f_func(const Vector& x);

  /**
   * Update the system with any newly added measurements. The measurements will be
   * appended to the existing factor matrix, and the factor is transformed into
   * triangular form again using Givens rotations.
   * Very efficient for exploration O(1), but can be more expensive otherwise >O(n).
   */
  virtual void incremental_update();

  /**
   * Solve the current system using backsubstitution. Only useful in combination with
   * incremental_update. Note that incremental_update can be performed multiple times
   * without performing backsubstitution, in order to spread out the updating, while
   * not performing backsubstition in every step.
   * At least O(n)!
   */
  virtual void solve();

  /**
   * Resolve the system with linearization based on current estimate;
   * perform variable reordering for efficiency.
   */
  virtual void batch_optimization_step();


  // internal variable used for operations such as removing of parts of
  // the graph that currently cannot be done incrementally
  bool _require_batch;

  cost_func_t _cost_func;

  void update_starts();

  Batch _batch;

protected:
  int _dim_nodes;
  int _dim_measure;
  int _num_new_measurements;
  int _num_new_rows;
  SparseSystem _R;

};

}
