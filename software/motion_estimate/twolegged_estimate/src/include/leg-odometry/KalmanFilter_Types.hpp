#ifndef KALMANFILTER_TYPES_HPP_
#define KALMANFILTER_TYPES_HPP_

#include <Eigen/Dense>

namespace KalmanFilter_Types {

struct Priori {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Eigen::MatrixXd M;
	Eigen::VectorXd x;
};

struct Posterior {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Eigen::MatrixXd P;
	Eigen::VectorXd x;
	
};


}

#endif /*KALMANFILTER_TYPES_HPP_*/
