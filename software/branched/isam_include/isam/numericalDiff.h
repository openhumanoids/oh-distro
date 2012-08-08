/**
 * @file numericalDiff.h
 * @brief Numerical differentiation.
 * @author Michael Kaess
 * @version $Id: numericalDiff.h 4038 2011-02-26 04:31:00Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <vector>
#include <Eigen/Dense>

#include "isam/Node.h"

namespace isam {

// Abstract class to enforce interface for function object.
class Function {
public:
  virtual ~Function() {}
  virtual int num_measurements() const = 0;
  virtual Eigen::VectorXd evaluate() const = 0;
  virtual std::vector<Node*>& nodes() = 0;
};

/**
 * Takes a general vector valued function and returns the
 * Jacobian at the linearization point given by x0.
 * @param func Function object with evaluation function that takes and returns vectors.
 * @return Matrix containing the Jacobian of func, with
 *         dim(y) rows and dim(x) columns, where y=func(x).
 */
Eigen::MatrixXd numericalDiff(Function& func);

}
