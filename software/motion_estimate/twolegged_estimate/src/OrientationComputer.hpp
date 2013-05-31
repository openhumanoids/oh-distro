/*
 * OrientationComputer.hpp
 *
 *  Created on: May 31, 2013
 *      Author: drc
 */

#ifndef ORIENTATIONCOMPUTER_HPP_
#define ORIENTATIONCOMPUTER_HPP_

#include "QuaternionLib.h"
#include "InertialOdometry_Types.hpp"

namespace InertialOdometry {

class OrientationComputer {
private:
	Eigen::Quaterniond q_state;
	unsigned long long latest_uts;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	OrientationComputer();

	void updateOrientation(const unsigned long long &uts, const Eigen::Quaterniond &set_to_q);
	Eigen::Vector3d ResolveBodyToRef(const Eigen::Vector3d &vec_b);
	void updateOutput(InertialOdomOutput* _out);

};

}

#endif /* ORIENTATIONCOMPUTER_HPP_ */
