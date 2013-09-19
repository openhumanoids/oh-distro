#include <iostream>
#include <leg-odometry/kalman_filter.hpp>
#include <unistd.h>
#include <tr1/functional>
#include <leg-odometry/KalmanFilter_Types.hpp>
#include <cstdlib>

using namespace std;



	    
	    

int main() {
	
	cout << "This is a test application to stimulate and test some of the kalman_filter class functionality." << endl << endl;
	
	cout << "Starting tests..." << endl << endl;
	
	
	KalmanFilter_Types::Posterior posterior_estimate;
	posterior_estimate.mu.resize(2);
	posterior_estimate.mu.setZero();
	
	KalmanFilter_Models::Joint_Model joint_model;
	KalmanFilter kf(joint_model);
	
	kf.Initialize();
	
	Eigen::VectorXd parameters;
	Eigen::VectorXd joint_positions;
	
	parameters.resize(1);
	joint_positions.resize(1);
	
	//KalmanFilter_Types::Priori priori;
	
	
	// Something like here is the model
	//kf.setModel(*prop, *meas); // for numerical derived jacobian
	//kf.setModel(*prop, *meas, *trans_Jacobian, *meas_Jacobian); // This will use analytical derivatives
	
	// then we need data
	
	// we iterate through all the events
	// publish the output from this process
	unsigned long utime;
	utime = 0;
	for (int i=0; i<1;i++ ) {
		utime += 1000;
		
		// Synthetic data
		joint_positions.setZero();// 
		
		joint_positions(0) = (rand() % 1000 - 500)/1000. ;
		
		// propate the system, this could be done in several ways -- we need to handle all of them
		// step in time with IMU measurements
		// step in time with joint position measurements
		// step in time with leg odometry position measurements
		// step in time with joint measurements to resolve positions internally
		//kf.step(joint_positions)
		
		if (false) {
			kf.step(utime,parameters);// This is a time update
		} else {
			kf.step(utime,parameters,joint_positions);
		}
		
		// IMU/LO/VO data fusion will have similar step function, using the different model
		
		// Fill in all other values
		// Transmit ERS message
	}
	
	
	cout << "Success" << endl;
	return 0;
}

