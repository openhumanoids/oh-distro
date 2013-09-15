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
	KalmanFilter_Types::Posterior posterior_estimate;
	
	KalmanFilter kf(joint_model);
	
	kf.Initialize();
	
	Eigen::Vector2d u;
	u.setZero();
	
	KalmanFilter_Types::Priori priori;
	
	priori = kf.propagatePriori(0, posterior_estimate,u);
	
	// Something like here is the model
	//kf.setModel(*prop, *meas); // for numerical derived jacobian
	//kf.setModel(*prop, *meas, *trans_Jacobian, *meas_Jacobian); // This will use analytical derivatives
	
	// then we need data
	
	// we iterate through all the events
	// publish the output from this process
	

	// propate the system, this could be done in several ways -- we need to handle all of them
	// step in time with IMU measurements
	// step in time with joint position measurements
	// step in time with leg odometry position measurements
	// step in time wiht joint measurements to resolve positions internally
	//kf.step()
	

	
	
	cout << "Success" << endl;
	return 0;
}

