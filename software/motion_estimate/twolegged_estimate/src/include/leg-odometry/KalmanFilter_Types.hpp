#ifndef KALMANFILTER_TYPES_HPP_
#define KALMANFILTER_TYPES_HPP_

#include <Eigen/Dense>
#include <leg-odometry/kf_conversion_definitions.hpp>

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


