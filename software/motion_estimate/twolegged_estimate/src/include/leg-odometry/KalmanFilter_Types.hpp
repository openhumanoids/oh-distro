#ifndef KALMANFILTER_TYPES_HPP_
#define KALMANFILTER_TYPES_HPP_

#include <Eigen/Dense>
#include <leg-odometry/kf_conversion_definitions.hpp>

namespace KalmanFilter_Types {

struct Priori {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	VAR_MATRIXd M;
	VAR_VECTORd x;
	
};

struct Posterior {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	VAR_MATRIXd P;
	VAR_VECTORd x;
	
};

struct Model {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	
	
	int a;
	
};

}// namespace Kalmanfilter_Types

#endif /*KALMANFILTER_TYPES_HPP_*/
