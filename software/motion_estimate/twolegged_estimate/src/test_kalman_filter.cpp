#include <iostream>
#include <leg-odometry/kalman_filter.hpp>
#include <unistd.h>
#include <tr1/functional>
#include <leg-odometry/KalmanFilter_Types.hpp>

using namespace std;



	    
	    

int main() {
	
	cout << "This is a test application to stimulate and test some of the kalman_filter class functionality." << endl << endl;
	
	cout << "Starting tests..." << endl << endl;
	
	KalmanFilter_Models::Joint_Model joint_model;
	
	KalmanFilter kf(joint_model);
	
	kf.Initialize();
	
	// Something like here is the model
	//kf.setModel(*prop, *meas); // for numerical derived jacobian
	//kf.setModel(*prop, *meas, *trans_Jacobian, *meas_Jacobian); // This will use analytical derivatives
	
	// then we need data
	
	// we iterate through all the events
	// publish the output from this process
	

	
	
	cout << "Success" << endl;
	return 0;
}

