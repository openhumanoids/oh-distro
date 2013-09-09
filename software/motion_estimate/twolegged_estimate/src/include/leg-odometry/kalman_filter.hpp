#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>

#include <leg-odometry/KalmanFilter_Types.hpp>


class KalmanFilter {
private:
	
	int state_size;
	KalmanFilter_Types::Priori priori;
	KalmanFilter_Types::Posterior posterior;
	
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	KalmanFilter();
	
	void Initialize();
	
	
	
	
};




#endif /*KALMAN_FILTER_HPP_*/
