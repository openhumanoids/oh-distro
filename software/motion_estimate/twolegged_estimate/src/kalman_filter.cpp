
#include <leg-odometry/kalman_filter.hpp>

#include <iostream>

KalmanFilter::KalmanFilter() {

}


KalmanFilter::KalmanFilter(KalmanFilter_Models::BaseModel &def_model) {
	std::cout << "KalmanFilter::KalmanFilter(KalmanFilter_Models::BaseModel -- saying hi." << std::endl;
	
	// Dont want to do this
	if (_model = dynamic_cast<KalmanFilter_Models::Joint_Model*>(&def_model) ) {
		// we have a joint filter
		std::cout << "KalmanFilter::KalmanFilter -- Copying pointer for Joint filter" << std::endl;
	} else if (_model = dynamic_cast<KalmanFilter_Models::DataFusion_Model*>(&def_model)) {
		// we have a data fusion filter
		std::cout << "KalmanFilter::KalmanFilter -- Copying pointer for DataFusion filter" << std::endl;
	}
}

KalmanFilter::~KalmanFilter() {
	//free()
	
}


void KalmanFilter::Initialize() {
	_model->identify();
	
	return;
}


KalmanFilter_Types::Priori KalmanFilter::propagatePriori(const unsigned long &ut_now, const KalmanFilter_Types::Posterior &post) {
	
	KalmanFilter_Types::Priori priori;
	
	// We want to propagate a current state mean and covariance estimate according to the some defined system model
	
	// propagate mu
	priori.mu = _model->propagation_model(post.mu);
	
	
	// Prepare process covariance matrix
	// Compute dynamics matrix
	// Compute state transition and discrete process covariance matrices
	
	// Compute priori covariance matrix
	
	// Compute Kalman Gain matrix
	
	
	return priori;
}



KalmanFilter_Types::Posterior KalmanFilter::propagatePosterior() {
	
	KalmanFilter_Types::Posterior temp;
	
	
	return temp;
}


void KalmanFilter::define_model() {
	
	
	return;
}

