/*
 * OrientationComputer.cpp
 *
 *  Created on: May 31, 2013
 *      Author: drc
 */

#include <inertial-odometry/OrientationComputer.hpp>

namespace InertialOdometry {


	OrientationComputer::OrientationComputer() {
		latest_uts = 0;
		q_state.setIdentity();

		prevWb.setZero();

	}

	void OrientationComputer::updateOrientation(const unsigned long long &uts, const Eigen::Quaterniond &set_to_q) {

		if (set_to_q.norm() <=0.95) {
			std::cerr << "OrientationComputer::updateOrientation -- asking to set the quaternion to a non-unit quaternion. Skipping this assignment.\n";
			return;
		}

		q_state = set_to_q;

		latest_uts = uts;
	}

	void OrientationComputer::updateOrientation(const unsigned long long &uts, const Eigen::Vector3d &w_b) {

		if (uts <= latest_uts) {
			std::cout << "OrientationComputer::updateOrientation -- warning, you are requesting a quaternion update backwards in time, ignoring step." << std::endl;
			return;
		}

		Eigen::Matrix3d R;
		R = q2C(q_state);

		// we should be using an integration module for this
		double dt;
		dt = 1.E-6*((double)(uts - latest_uts));

		// We now need to implement our own rotation computation. We shall use the exponential map to do this
		std::cout << "OrientationComputer::updateOrientation -- q " << "before" << q_state.w() << ", " << q_state.z() << ", R is " << std::endl << R << std::endl;

		exmap( 0.5*dt*(w_b + prevWb), R); // trapezoidal integration, before application through the exponential map

		// Convert back to quaternion world and store state data
		Eigen::Quaterniond tempq;

		tempq = C2q(R);

		if (tempq.norm() <=0.95) {
			std::cerr << "OrientationComputer::updateOrientation -- Setting the quaternion to a non-unit quaternion.\n";
		}

		q_state = tempq;
		latest_uts = uts;
		prevWb = w_b;

		std::cout << "OrientationComputer::updateOrientation -- q " << "after" << tempq.w() << ", " << tempq.z() << ", R is " << std::endl << R << std::endl;


		//		std::cout << "OrientationComputer::updateOrientation -- updating orientation estimate with rates: " << w_b.transpose() << std::endl;
		std::cout << "OrientationComputer::updateOrientation -- quaternion now is " << C2q(R).w() << ", " << C2q(R).x() << ", " << C2q(R).y() << ", " << C2q(R).z() << std::endl;
	}


	Eigen::Vector3d OrientationComputer::ResolveBodyToRef(const Eigen::Vector3d &vec_b) {

		Eigen::Vector3d vec_ref;

		vec_ref = q2C(q_state) * vec_b;

		return vec_ref;
	}

	void OrientationComputer::updateOutput(InertialOdomOutput* _out) {
		_out->quat = q_state;
	}

	Eigen::Quaterniond OrientationComputer::q() {
		return q_state;
	}

	void OrientationComputer::exmap(const Eigen::Vector3d &w_k0, Eigen::Matrix3d &R) {

		//		std::cout << "OrientationComputer::exmap -- w_k0 " << std::endl << w_k0.transpose() << std::endl;

		Eigen::Matrix3d Gam;
		Eigen::Matrix3d coeff;
		Gam.setZero();
		coeff.setZero();

		//		std::cout << "OrientationComputer::exmap -- R" << std::endl << R << std::endl;

		Gam = vec2skew(w_k0);
		double wNorm;
		wNorm = w_k0.norm();

		if (wNorm>1E-5) { //% 1E-7 is chosen because double numerical LSB is around 1E-15 for nominal values [-pi..pi]
			coeff = Eigen::Matrix3d::Identity() - (sin(wNorm)/wNorm)*Gam + (1-cos(wNorm))/(wNorm*wNorm)*Gam*Gam;
			//			coeff = eye(3) - sin(v_norm)/v_norm*Gam + (1-cos(v_norm))/(v_norm^2)*Gam^2;
		} else {
			coeff = Eigen::Matrix3d::Identity() - Gam + (Gam*Gam);
		}

		Eigen::Matrix3d aliastemp;
		aliastemp = R;
		R = coeff*aliastemp;

	}

	Eigen::Matrix3d OrientationComputer::vec2skew(const Eigen::Vector3d &v) {
		Eigen::Matrix3d ret;
		ret.setZero();
		ret(0,1) = -v(2);
		ret(0,2) = v(1);
		ret(1,2) = -v(0);

		ret(1,0) = v(2);
		ret(2,0) = -v(1);
		ret(2,1) = v(0);

		return ret;
	}



}
