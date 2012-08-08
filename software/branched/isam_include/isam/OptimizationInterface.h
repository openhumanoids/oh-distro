/**
 * @file OptimizationInterface.h
 * @brief Abstract base class for nonlinear optimizer.
 * @author Michael Kaess
 * @author David Rosen
 * @version $Id: OptimizationInterface.h 6371 2012-03-29 22:22:23Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <Eigen/Dense>

#include "SparseSystem.h"
#include "Node.h"


namespace isam {

/**
 * Abstract base class providing an interface between the nonlinear system
 * to be optimized (stored in the Nodes of the Graph constructed in the SLAM)
 * and the Optimization class that actually performs the optimizations.
 */
class OptimizationInterface {

protected:

  /** Factored Jacobian about the current linearization point.*/
  SparseSystem _R;

public:
  virtual SparseSystem jacobian() = 0;
  virtual void apply_exmap(const Eigen::VectorXd& delta) = 0;
  virtual void self_exmap(const Eigen::VectorXd& delta) = 0;
  virtual void estimate_to_linpoint() = 0;
  virtual void linpoint_to_estimate() = 0;
  virtual void swap_estimates() = 0;
  virtual Eigen::VectorXd weighted_errors(Selector s = ESTIMATE) = 0;

  OptimizationInterface(): _R(1,1) {}

  virtual ~OptimizationInterface() {}

  friend class Optimizer;
};

}
