// Define the functions for differnt Kalman Filter models

#include <leg-odometry/KF_Models.hpp>

namespace KalmanFilter_Models {

MatricesUnit::MatricesUnit() {
	A.setZero();
	B.setZero();
	C.setZero();
	D.setZero();
	Q.setZero();
	R.setZero();
	
	std::cout << "Continuous_Matrices::Continuous_Matrices() -- happened" << std::endl;
}


// Base Model===============================================================================================================

MatricesUnit BaseModel::getContinuousMatrices(const VAR_VECTORd &state) {
	
	// Compute the new jacobians
	
	//std::cout << "BaseModel::getContinuousMatrices -- calling for new Jacobians" << std::endl;
	continuous_matrices.A = anaylitical_jacobian(state);
	
	
	
	// input shaping matrix
	// set once in the constructor
	
	
	// Continuous process noise
	
	
	return continuous_matrices;
}


// Joint Model=============================================================================================================



Joint_Model::Joint_Model() {
	settings.propagate_with_linearized = true;
	settings.analytical_jacobian_available = true;
	
	// state = [pos, vel]
	settings.state_size = 2;
	continuous_matrices.A.resize(2,2);
	continuous_matrices.B.resize(2,2);
	continuous_matrices.B.setIdentity();
	
	// noise matrix
	continuous_matrices.Q.resize(2,2);
	continuous_matrices.Q(0,0) = 0.001;
	continuous_matrices.Q(1,1) = 0.001;
	
	
}

VAR_MATRIXd Joint_Model::anaylitical_jacobian(const VAR_MATRIXd &state) {
	
	VAR_MATRIXd F(settings.state_size,settings.state_size);
	F.setZero();
	//continuous_matrices.A.setZero(); //  -- Inefficient
	
	// joint velocity is the first derivative of joint position
	F(1,1) = 1;
	std::cout << "Joint_Model::anaylitical_jacobian -- a new Jacobian was computed." << std::endl;
	
	return F;
}


VAR_VECTORd Joint_Model::propagation_model(const VAR_VECTORd &post) { 
	VAR_VECTORd priori;
	
	// Now we need to do some stuff to change the current state estimate to the next one.
	// This will generally involve integrations, therefore we will need the have timestamps of the various events
	
	priori = post;	
	
	return priori;
}




VAR_VECTORd Joint_Model::measurement_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

void Joint_Model::identify() { std::cout << "This is the Joint Model class." << std::endl;}








// Data Fusion Model========================================================================================================



DataFusion_Model::DataFusion_Model() {
	settings.propagate_with_linearized = true;
	settings.analytical_jacobian_available = true;
	settings.state_size = 15;

}


VAR_MATRIXd DataFusion_Model::anaylitical_jacobian(const VAR_MATRIXd &state) {

	
	
	return continuous_matrices.A;
}

VAR_VECTORd DataFusion_Model::propagation_model(const VAR_VECTORd &post) { 
	VAR_VECTORd priori;
		
		// Now we need to do some stuff to change the current state estimate to the next one.
		// This will generally involve integrations, therefore we will need the have timestamps of the various events
		
		std::cout << "DataFusion_Model::propagation_model -- step 1" << std::endl;
		
		return priori;
}


VAR_VECTORd DataFusion_Model::measurement_model(VAR_VECTORd Param) { 
	VAR_VECTORd temp;
	
	return temp;
}

void DataFusion_Model::identify() { std::cout << "This is a DataFusion Model." << std::endl;}

}
