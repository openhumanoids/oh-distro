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
		lQb.setIdentity();

		prevWb.setZero();

	}

	void OrientationComputer::updateOrientation(const unsigned long long &uts, const Eigen::Quaterniond &set_to_q) {

		if (set_to_q.norm() <= 0.998) {
			std::cerr << "OrientationComputer::updateOrientation -- asking to set the quaternion to a non-unit quaternion. Skipping this assignment.\n";
			return;
		}

		lQb = set_to_q;

		latest_uts = uts;
	}

	void OrientationComputer::updateOrientationWithAngle(const unsigned long long &uts, const Eigen::Vector3d &delta_ang) {

		if (uts <= latest_uts) {
			std::cout << "OrientationComputer::updateOrientation -- warning, you are requesting a quaternion update backwards in time, ignoring step." << std::endl;
			return;
		}

		lQb = exmap(delta_ang, lQb);

		latest_uts = uts;
		//		std::cout << "OrientationComputer::updateOrientation -- lQb " << "after" << lQb.w() << ", " << lQb.x() << ", " << lQb.y() << ", " << lQb.z() << std::endl;

	}

	// This member function implements a trapezoidal integration for converting rotation rates into delta angles
	void OrientationComputer::updateOrientationWithRate(const unsigned long long &uts, const Eigen::Vector3d &w_b) {

		double dt;
		dt = 1.E-6*((double)(uts - latest_uts));
		if (latest_uts == 0) { // assumption for initial condition
			dt = 0.01;
		}

		Eigen::Vector3d temp;
		temp = dt*(w_b); // TODO -- This should be an integration module from SignalTap

		updateOrientationWithAngle(uts, temp); // We use midpoint integration to obtain a delta angle

		//std::cout << "OrientationComputer::updateOrientationWithRate -- dt " << dt << std::endl;

		latest_uts = uts;
		prevWb = w_b;
	}

	void OrientationComputer::rotateOrientationUpdate(const Eigen::Vector3d &_dE_l) {

		//		INSCompensator.dlQl = qprod(e2q(-limitedFB*x(1:3)),INSCompensator.dlQl);
		//		pose.lQb = qprod(pose__.lQb,(INSCompensator.dlQl)); -- we do the limitedFB and sign flip on _dE_l at output of the EKF
		Eigen::Quaterniond tmp;
		tmp = qprod(lQb, e2q(_dE_l));
		lQb = tmp;
	}


	Eigen::Vector3d OrientationComputer::ResolveBodyToRef(const Eigen::Vector3d &vec_b) {

		return qrot(lQb.conjugate(), vec_b);
	}

	void OrientationComputer::updateOutput(InertialOdomOutput &_out) {
		_out.quat = lQb;
	}

	Eigen::Quaterniond OrientationComputer::q() {
		return lQb;
	}


	// Must move this out to a separate library
	/*void OrientationComputer::exmap(const Eigen::Vector3d &w_k0, Eigen::Matrix3d &R) {

		//		std::cout << "OrientationComputer::exmap -- w_k0 " << std::endl << w_k0.transpose() << std::endl;

		Eigen::Matrix3d Gam;
		Eigen::Matrix3d coeff;
		Gam.setZero();
		coeff.setZero();

		//		std::cout << "OrientationComputer::exmap -- R" << std::endl << R << std::endl;

		Gam = vec2skew(w_k0);
		std::cout << "OrientationComputer::exmap -- Gam " << std::endl << Gam << std::endl;
		double wNorm;
		wNorm = w_k0.norm();

		if (wNorm>1E-7) { //% 1E-7 is chosen because double numerical LSB is around 1E-15 for nominal values [-pi..pi]
			coeff = Eigen::Matrix3d::Identity() - (sin(wNorm)/wNorm)*Gam + (1-cos(wNorm))/(wNorm*wNorm)*Gam*Gam;
			//			coeff = eye(3) - sin(v_norm)/v_norm*Gam + (1-cos(v_norm))/(v_norm^2)*Gam^2;
		} else {
			coeff = Eigen::Matrix3d::Identity() - Gam + (Gam*Gam);
		}

		Eigen::Matrix3d aliastemp;
		aliastemp = R;
		R = coeff*aliastemp;

	}*/


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
