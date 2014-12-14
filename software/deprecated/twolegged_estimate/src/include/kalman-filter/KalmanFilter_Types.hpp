#ifndef KALMANFILTER_TYPES_HPP_
#define KALMANFILTER_TYPES_HPP_

#include <Eigen/Dense>
#include <kalman-filter/kf_conversion_definitions.hpp>

namespace KalmanFilter_Types {

struct Priori {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	unsigned long utime;
	
	VAR_MATRIXd M;  // state covariances
	VAR_VECTORd mu; // state mean
};

struct Posterior {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	unsigned long utime;
	
	VAR_MATRIXd P;  // state covariances
	VAR_VECTORd mu; // state mean
	VAR_MATRIXd S; // Innovation information matrix
	VAR_MATRIXd K; // Kalman Gain, note that this is computed from M covariance from the priori (slight break of abstraction here -- but computational saving)
	VAR_VECTORd innov;
};

struct State {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	unsigned long utime;
	int last_update_type;
	
	VAR_MATRIXd Cov;
	VAR_VECTORd X;
};


}// namespace Kalmanfilter_Types

#endif /*KALMANFILTER_TYPES_HPP_*/


