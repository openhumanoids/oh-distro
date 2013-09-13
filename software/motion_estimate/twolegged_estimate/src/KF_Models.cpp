// Define the functions for differnt Kalman Filter models

#include <leg-odometry/KF_Models.hpp>

namespace KalmanFilter_Models {

// Joint Model

KalmanFilter_Types::Priori Joint_Model::propagation_model(const unsigned long &utime, const KalmanFilter_Types::Posterior &post) { 
	KalmanFilter_Types::Priori priori;
	
	// Now we need to do some stuff to change the current state estimate to the next one.
	// This will generally involve integrations, therefore we will need the have timestamps of the various events
	
	double dt;
	
	dt = (1E-6)*(utime - post.utime);
	
	
	return priori;
}

KalmanFilter_Types::Priori Joint_Model::propagation_model(KalmanFilter_Types::Priori prev_priori) { 
	KalmanFilter_Types::Priori temp;
	
	return temp;
}

VAR_VECTORd Joint_Model::measurement_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

void Joint_Model::identify() { std::cout << "This is a Joint Model." << std::endl;}








// Data Fusion Model========================================================================================================

KalmanFilter_Types::Priori DataFusion_Model::propagation_model(const unsigned long &utime, const KalmanFilter_Types::Posterior &post) { 
	KalmanFilter_Types::Priori temp;
	
	return temp;
}

KalmanFilter_Types::Priori DataFusion_Model::propagation_model(KalmanFilter_Types::Priori prev_priori) { 
	KalmanFilter_Types::Priori temp;
	
	return temp;
}

VAR_VECTORd DataFusion_Model::measurement_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

void DataFusion_Model::identify() { std::cout << "This is a DataFusion Model." << std::endl;}

}
