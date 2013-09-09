#include <iostream>
#include <leg-odometry/kalman_filter.hpp>

using namespace std;

int main() {
	
	cout << "This is a test application to stimulate and test some of the kalman_filter class functionality." << endl << endl;
	
	cout << "Starting tests..." << endl << endl;
	
	KalmanFilter kf;
	
	kf.Initialize();
	
	
	cout << "Success" << endl;
	return 0;
}

