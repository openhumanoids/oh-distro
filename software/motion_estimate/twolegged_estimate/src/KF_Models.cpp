// Define the functions for differnt Kalman Filter models

#include <leg-odometry/KF_Models.hpp>

namespace KalmanFilter_Models {

// Joint Model

VAR_VECTORd Joint_Model::propagation_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

VAR_VECTORd Joint_Model::measurement_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

void Joint_Model::identify() { std::cout << "This is a Joint Model." << std::endl;}








// Data Fusion Model

VAR_VECTORd DataFusion_Model::propagation_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}
VAR_VECTORd DataFusion_Model::measurement_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

void DataFusion_Model::identify() { std::cout << "This is a DataFusion Model." << std::endl;}

}
