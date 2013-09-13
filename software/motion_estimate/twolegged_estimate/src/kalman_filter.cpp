
#include <leg-odometry/kalman_filter.hpp>

#include <iostream>

KalmanFilter::KalmanFilter() {
	int model_type;
	
	std::cout << "KalmanFilter::KalmanFilter() -- object created" << std::endl;
	
	state_size = 0;
	
	switch (model_type) {
	case JOINT_MODEL:
		
		
		
		break;
	case DATAFUSION_MODEL:
		
		
		
		break;
	default:
		std::cerr << "KalmanFilter::KalmanFilter -- Invalid model type defined" << std::endl;
		
		break;
	}
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

