/*
 * bottransforms.hpp
 *
 *  Created on: Jun 8, 2013
 *      Author: drc
 */

#ifndef BOTTRANSFORMS_HPP_
#define BOTTRANSFORMS_HPP_

#include <Eigen/Dense>

class BotTransforms {
private:
  Eigen::Isometry3d lc2p;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BotTransforms();

  void setLCam2Pelvis(const Eigen::Quaterniond &q,const Eigen::Vector3d &trans);
  void setLCam2Pelvis(const Eigen::Isometry3d &c2b);

  Eigen::Vector3d lcam2pelvis(const Eigen::Vector3d &vec_lcam);

};


#endif /* BOTTRANSFORMS_HPP_ */
