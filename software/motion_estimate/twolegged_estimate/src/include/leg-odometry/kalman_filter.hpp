#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <leg-odometry/KalmanFilter_Types.hpp>
#include <leg-odometry/KF_Models.hpp>

class KalmanFilter {
private:
	
	int state_size;
	KalmanFilter_Types::Priori priori;
	KalmanFilter_Types::Posterior posterior;
	
	KalmanFilter_Models::Joint_Model model;
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	KalmanFilter();
	
	void Initialize();
	
	KalmanFilter_Types::Priori propagatePriori(const KalmanFilter_Types::Posterior &post, const unsigned long &ut);
	
	KalmanFilter_Types::Posterior propagatePosterior();
	
	void define_model();
	
	//void define_model(/* callback for continuous f, */ /* callback for continuous h */);
	
};




#endif /*KALMAN_FILTER_HPP_*/
