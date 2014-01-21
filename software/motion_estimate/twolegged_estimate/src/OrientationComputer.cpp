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

		//		Eigen::Matrix3d R;
		//		R = q2C(lQb);

		// We now need to implement our own rotation computation. We shall use the exponential map to do this
		//		std::cout << "OrientationComputer::updateOrientation -- q " << "before" << lQb.w() << ", " << lQb.x() << ", " << lQb.y() << ", " << lQb.z() << std::endl;

		lQb = exmap(delta_ang, lQb);
		//		exmap(delta_ang, R); // midpoint integration, before application through the exponential map
		//
		//		// Convert back to quaternion world and store state data
		//		Eigen::Quaterniond tempq;
		//
		//		tempq = C2q(R);
		//
		//		if (tempq.norm() <=0.95) {
		//			std::cerr << "OrientationComputer::updateOrientation -- Setting the quaternion to a non-unit quaternion.\n";
		//		}
		//
		//		lQb = tempq;
		latest_uts = uts;
		//		std::cout << "OrientationComputer::updateOrientation -- lQb " << "after" << lQb.w() << ", " << lQb.x() << ", " << lQb.y() << ", " << lQb.z() << std::endl;

	}

	// This member function implements a trapezoidal integration for converting rotation rates into delta angles
	void OrientationComputer::updateOrientationWithRate(const unsigned long long &uts, const Eigen::Vector3d &w_b) {

		double dt;
		dt = 1.E-6*((double)(uts - latest_uts));

		Eigen::Vector3d temp;
		temp = 0.5*dt*(w_b + prevWb); // TODO -- This should be an integration module from SignalTap

		updateOrientationWithAngle(uts, temp); // We use midpoint integration to obtain a delta angle

		std::cout << "OrientationComputer::updateOrientationWithRate -- dt " << dt << std::endl;

		latest_uts = uts;
		prevWb = w_b;
	}

	void OrientationComputer::rotateOrientationUpdate(const Eigen::Vector3d &_dE_l) {

		//		INSCompensator.dlQl = qprod(e2q(limitedFB*x(1:3)),INSCompensator.dlQl);
		//		pose.lQb = qprod(pose__.lQb,qconj(INSCompensator.dlQl)); -- we do the limitedFB and sign flip on _dE_l at output of the EKF
		lQb = qprod(lQb, e2q(_dE_l));
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

	void OrientationComputer::exmap(const Eigen::Vector3d &w_k0, Eigen::Matrix3d &R) {

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

	}

	Eigen::Quaterniond OrientationComputer::exmap(const Eigen::Vector3d &dE_l, Eigen::Quaterniond &lQb) {
		//		w_dt = w*dt; -- This is dE_l (delta rotation in the local frame)
		//		w_norm = norm(w);
		//		w_norm_dt = w_norm*dt;
		//
		//		if (w_norm>1E-6) % 1E-7 is chosen because double numerical LSB is around 1E-18 for nominal values [-pi..pi]
		//		    % and (1E-7)^2 is at 1E-14, but 1E-7 rad/s is 0.02 deg/h
		//		    r = [cos(w_norm*dt/2);...
		//		         w./w_norm*sin(w_norm*dt/2)];
		//
		//		    aQb_k0 = qprod(r,aQb_k1);
		//		else
		//		    r = [cos(w_norm_dt/2);...
		//		         w_dt*(0.5-w_norm_dt*w_norm_dt/48)];
		//
		//		    aQb_k0 = qprod(r,aQb_k1);
		//		end
		//
		//		if (abs(1-norm(aQb_k0))>1E-13)
		//		    disp 'zeroth_int_Quat_closed_form -- normalizing time propagated quaternion'
		//		    aQb_k0 = aQb_k0./norm(aQb_k0);
		//		end


		double dE_lNorm;
		dE_lNorm = dE_l.norm();

		Eigen::Quaterniond r;
		Eigen::Quaterniond result;
		r.setIdentity();
		result.setIdentity();

		double sindE;
		sindE = sin(0.5*dE_lNorm)/dE_lNorm;

		if (dE_lNorm>1E-7) { //% 1E-7 is chosen because double numerical LSB is around 1E-15 for nominal values [-pi..pi]
			r.w() = cos(0.5*dE_lNorm);
			r.x() = dE_l(0)*sindE;
			r.y() = dE_l(1)*sindE;
			r.z() = dE_l(2)*sindE;
		} else {
			double tmp;
			tmp = (0.5 - dE_lNorm*dE_lNorm*0.020833333333333333);

			r.w() = cos(0.5*dE_lNorm);
			r.x() = dE_l(0)*tmp;
			r.y() = dE_l(1)*tmp;
			r.z() = dE_l(2)*tmp;
		}

		result = qprod(r,lQb);

		if (abs(1-result.norm()) > 1E-13){
			std::cout << "OrientationComputer::exmap -- had to renormalize quaternion" << std::endl;
			result = result.normalized();
		}
		return result;
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
