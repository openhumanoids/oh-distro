
#ifndef __QUATERNIONLIB_H__
#define __QUATERNIONLIB_H__

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <math.h>

namespace InertialOdometry 
{
  class QuaternionLib {
  public:
    // The result is pushed back into a2 (overwrites what was in there)
    static void QuaternionProduct(double *a1, double *a2);

    // Result is pushed back by overwritting the argument variable 
    static void QuaternionAdjoint(double *q);

    // q is the rotation quaternion
    // v is the vector to be rotated, assumed to be size 3
    // Result is pushed back by overwriting the v variable
    //static void VectorRotation(double *q, double *v);
    static void VectorRotation(Eigen::Vector4d q, Eigen::Vector3d &v);

    //This function is specific to NED and forward right down coordinate frames
    static void q2e(Eigen::Vector4d q_, Eigen::Vector3d &E);
    static void q2e(const Eigen::Quaterniond &q_, Eigen::Vector3d &E);
    static void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll);
    
    //get the direction cosine matrix equivalent to quaternion (scalar, vector)
    static void q2C(Eigen::Vector4d const &q_, Eigen::Matrix<double,3,3> &C);
    static Eigen::Matrix<double,3,3> q2C(Eigen::Quaterniond const &q_);
    
    // Places the 3x3 skew symmetric matrix of 3x1 vector v in skew
    static void skew(Eigen::Vector3d const &v_, Eigen::Matrix<double,3,3> &skew);
    
    static void e2C(Eigen::Vector3d const &E, Eigen::Matrix<double, 3, 3> &C);
    
    static void printEulerAngles(std::string prefix, const Eigen::Isometry3d &isom);
  };

}

#endif
