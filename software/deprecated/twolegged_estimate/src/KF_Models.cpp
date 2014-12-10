// Define the functions for differnt Kalman Filter models

#include <iostream>

#include <kalman-filter/KF_Models.hpp>
#include <kalman-filter/KF_Tuning_Parameters.hpp>

namespace KalmanFilter_Models {

MatricesUnit::MatricesUnit() {
	A.setZero();
	B.setZero();
	C.setZero();
	D.setZero();
	Q.setZero();
	R.setZero();
	
	// TODO -- review all creation instances of this
	//std::cout << "Continuous_Matrices::Continuous_Matrices() -- happened" << std::endl;
}


// Base Model===============================================================================================================

MatricesUnit BaseModel::getContinuousMatrices(const VAR_VECTORd &state) {
	
	// Compute the new jacobians
	
	//std::cout << "BaseModel::getContinuousMatrices -- calling for new Jacobians" << std::endl;
	if (settings.analytical_jacobian_available == true) {
		continuous_matrices.A = anaylitical_jacobian(state);
	} else {
		std::cerr << "BaseModel::getContinuousMatrices -- " << "\33[0;31m" << "oops, numerical Jacobian functions not implemented yet." << "\33[0m" << std::endl;
		
	}
	
	
	// input shaping matrix
	// set once in the constructor
	
	
	// Continuous process noise
	continuous_matrices.Q = continuous_process_noise(state);
	
	// Measurement noise
	continuous_matrices.R = measurement_noise_cov(state);
	
	// setup the process noise shaping matrix V
	continuous_matrices.V = noise_shaping_matrix(state);
	
	return continuous_matrices;
}

void BaseModel::setup(KalmanFilter_Types::Priori &priori, KalmanFilter_Types::Posterior &posterior) {
	//std::cout << "BaseModel::setSizes -- setting sizes :" << continuous_matrices.A.cols() << std::endl;
	priori.mu.resize(continuous_matrices.A.cols());
	//priori.M.resize(continuous_matrices.A.cols(),continuous_matrices.A.cols());
	
	priori.M.resize(settings.state_size,settings.state_size);
	priori.M.setZero();
	
	posterior.P.resize(settings.state_size,settings.state_size);
	posterior.P.setZero();
	
	// We assume a large joint state covariance uncertainty
	for (int i=0;i < settings.state_size;i++) {
		priori.M(i,i) = 99999.;
		posterior.P(i,i) = -1.;
	}
	
	//std::cout << "BaseModel::setSizes -- State Covariance initialized to large uncertainty." << std::endl;
	
}

void BaseModel::getKFDebugData() {
	
	//	TODO -- incomplete
	
	return;
}

// Joint Model=============================================================================================================


Joint_Model::Joint_Model() {
	settings.propagate_with_linearized = true;
	settings.analytical_jacobian_available = true;
	settings.use_linearized_measurement = true;
	
	// state = [pos, vel]
	settings.state_size = 2;
	
	
	VAR_VECTORd dummy_state(settings.state_size);
	dummy_state.setZero();
	
	continuous_matrices.A.resize(settings.state_size,settings.state_size);// This is used to drive the size of the KalmanFilter priori and posterior variables
	continuous_matrices.A.setZero();
	continuous_matrices.A(0,1) = 1;
	
	continuous_matrices.B.resize(settings.state_size,settings.state_size);
	continuous_matrices.B.setIdentity();
	
	// Only a single joint position measurement
	continuous_matrices.C.resize(1,settings.state_size);
	continuous_matrices.C.setZero();
	continuous_matrices.C(0,0) = 1;
	
	// noise matrices
	continuous_matrices.Q.resize(2,2);
	continuous_matrices.Q = continuous_process_noise(dummy_state);
	continuous_matrices.R.resize(1,1);
	continuous_matrices.R = measurement_noise_cov(dummy_state);
	
}

VAR_MATRIXd Joint_Model::continuous_process_noise(const VAR_MATRIXd &state) {
	
	continuous_matrices.Q.setZero();
	
	continuous_matrices.Q(0,0) = PROCESS_NOISE_JOINT_POSITIONS;
	continuous_matrices.Q(1,1) = PROCESS_NOISE_JOINT_VELOCITIES;
	
	return continuous_matrices.Q;
}

// Return a 2x2 R matrix for Atlas joints
VAR_MATRIXd Joint_Model::measurement_noise_cov(const VAR_VECTORd &state) {
	
	//std::cout << "Joint_Model::measurement_noise -- has been called upon from somewhere in the ecosystem." << std::endl;
	
	continuous_matrices.R(0,0) = JOINT_POSITION_MEASUREMENT_NOISE;
	
	return continuous_matrices.R;
}


VAR_MATRIXd Joint_Model::anaylitical_jacobian(const VAR_MATRIXd &state) {
	
	// joint velocity is the first derivative of joint position
	continuous_matrices.A(0,1) = 1;
	
	// d/dt x = Fx
	// x = [joint_pos; joint_vel]
	// F = [0 1;0 0]
	
	//F = [0 1 0;0 0 1;0 0 0]
	
	return continuous_matrices.A;
}


VAR_VECTORd Joint_Model::propagation_model(const VAR_VECTORd &post) { 
	VAR_VECTORd priori;
	
	// Now we need to do some stuff to change the current state estimate to the next one.
	// This will generally involve integrations, therefore we will need the have timestamps of the various events
	
	std::cerr << "Joint_Model::propagation_model -- not implemented yet!" << std::endl;
	priori = post;	
	
	return priori;
}


VAR_VECTORd Joint_Model::measurement_model(VAR_VECTORd Param) { 
	
	std::cout << "Joint_Model::measurement_model -- " << NOT_IMPL << std::endl;
	VAR_VECTORd temp;
	
	return temp;
}


void Joint_Model::identify() { 
	//std::cout << "Joint_Model::identify -- This is the Joint Model class." << std::endl;
}


VAR_MATRIXd Joint_Model::noise_shaping_matrix(const VAR_VECTORd &state) {
	
	//std::cout << "Joint_Model::noise_shaping_matrix -- has been set." << std::endl;
	
	VAR_MATRIXd eye(continuous_matrices.Q.rows(),continuous_matrices.Q.cols());
	eye.setIdentity();
	
	//std::cout << "Joint_Model::noise_shaping_matrix -- returning" << eye << std::endl;
	
	return eye;
}



// Data Fusion Model========================================================================================================



DataFusion_Model::DataFusion_Model() {
	settings.propagate_with_linearized = true;
	settings.analytical_jacobian_available = true;
	settings.use_linearized_measurement = true;
	
	settings.state_size = 15;

}



VAR_MATRIXd DataFusion_Model::continuous_process_noise(const VAR_MATRIXd &state) {
	
	VAR_MATRIXd temp;
	
	return temp;
}

VAR_MATRIXd DataFusion_Model::measurement_noise_cov(const VAR_VECTORd &state) {
	VAR_MATRIXd temp;
	
	return temp;
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

void DataFusion_Model::identify() { 
	std::cout << "DataFusion_Model::identify -- This is a DataFusion Model." << std::endl;
}

VAR_MATRIXd DataFusion_Model::noise_shaping_matrix(const VAR_VECTORd &state) {
	VAR_MATRIXd temp;
	
	std::cerr << "DataFusion_Model::noise_shaping_matrix -- " << NOT_IMPL << std::endl;
	
	return temp;
}

}










