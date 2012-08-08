/**
 * @file Noise.h
 * @brief Various noise models.
 * @author Michael Kaess
 * @version $Id: Noise.h 5797 2011-12-07 03:50:41Z kaess $
 *
 * [insert iSAM license]
 *
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/LU> 

namespace isam {

// general noise model class
class Noise {
public:
  Eigen::MatrixXd _sqrtinf;
  const Eigen::MatrixXd& sqrtinf() const {return _sqrtinf;}
};

// noise model based on square root information matrix
class SqrtInformation : public Noise {
public:
  SqrtInformation(const Eigen::MatrixXd& sqrtinf) {_sqrtinf = sqrtinf;}
};

// noise model based on information matrix
class Information : public Noise {
public:
  Information(const Eigen::MatrixXd& inf) {
    _sqrtinf = inf.llt().matrixU();
  }
};

// noise model based on covariance matrix
class Covariance : public Noise {
public:
  Covariance(const Eigen::MatrixXd& cov) {
    _sqrtinf = cov.inverse().llt().matrixU();
  }
};

}
