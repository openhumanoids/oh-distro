
#include <leg-odometry/kalman_filter.hpp>

#include <iostream>

KalmanFilter::KalmanFilter() {
	std::cout << "KalmanFilter::KalmanFilter() -- object created" << std::endl;
	
	state_size = 0;
}

void KalmanFilter::Initialize() {
	
	
	return;
}


KalmanFilter_Types::Priori KalmanFilter::propagatePriori( const KalmanFilter_Types::Posterior &post, const unsigned long &ut) {
	
	KalmanFilter_Types::Priori temp;
	
	
	return temp;
}



KalmanFilter_Types::Posterior KalmanFilter::propagatePosterior() {
	
	KalmanFilter_Types::Posterior temp;
	
	
	return temp;
}


void KalmanFilter::define_model() {
	
	
	return;
}

