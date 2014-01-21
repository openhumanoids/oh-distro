
#ifndef __QUATERNIONLIB_H__
#define __QUATERNIONLIB_H__

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <math.h>

Eigen::Vector3d q2e_new(const Eigen::Quaterniond q);
Eigen::Quaterniond C2q(const Eigen::Matrix3d C);
Eigen::Matrix3d q2C(const Eigen::Quaterniond q_);
Eigen::Vector3d C2e(const Eigen::Matrix3d C);
Eigen::Quaterniond e2q(const Eigen::Vector3d &E);
Eigen::Matrix3d e2C(Eigen::Vector3d Ec);
void skew(Eigen::Vector3d const &v_, Eigen::Matrix<double,3,3> &skew);

Eigen::Quaterniond qprod(const Eigen::Quaterniond &_a, const Eigen::Quaterniond &_b);
Eigen::Vector3d qrot(const Eigen::Quaterniond &_aQb, const Eigen::Vector3d &_v);

namespace InertialOdometry 
{


}

#endif
