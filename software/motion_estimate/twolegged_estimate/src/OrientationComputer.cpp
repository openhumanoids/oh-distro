/*
 * OrientationComputer.cpp
 *
 *  Created on: May 31, 2013
 *      Author: drc
 */

#include "OrientationComputer.hpp"

namespace InertialOdometry {


	OrientationComputer::OrientationComputer() {
		latest_uts = 0;
		q_state.setIdentity();

	}

	void OrientationComputer::updateOrientation(const unsigned long long &uts, const Eigen::Quaterniond &set_to_q) {

		if (set_to_q.norm() <=0.95) {
			std::cerr << "OrientationComputer::updateOrientation -- asking to set the quaternion to a non-unit quaternion. Skipping this assignment.\n";
			return;
		}

		q_state = set_to_q;

		latest_uts = uts;
	}

	Eigen::Vector3d OrientationComputer::ResolveBodyToRef(const Eigen::Vector3d &vec_b) {

		Eigen::Vector3d vec_ref;

		vec_ref = q2C(q_state) * vec_b;

		return vec_ref;
	}

	void OrientationComputer::updateOutput(InertialOdomOutput* _out) {
		_out->quat = q_state;
	}

}
