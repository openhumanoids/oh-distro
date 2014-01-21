/*
 * OrientationComputerUnitTest.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: dehann
 */



#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>

#include <Eigen/Dense>

#include <leg-odometry/QuaternionLib.h>
#include <inertial-odometry/OrientationComputer.hpp>

#define ITERATIONS 		1000
//#define PI 				   3.14159265358979323

using namespace std;

void setupData(double data[][3]);

int main() {

	cout << endl << "OrientationComputerUnitTest process." << endl << "====================================" << endl << endl;

	InertialOdometry::OrientationComputer orc;
	DataFileLogger log("OrientationComputerUnitTestLog.csv");

	Eigen::Vector3d v;
	Eigen::Matrix3d R;
	Eigen::Quaterniond q;


	v << 1, 2, 3;
	R.setZero();

	R = orc.vec2skew(v);

	cout << "vec2skew function on: " << v.transpose()  << " gives:" << endl << R << endl << endl;

	cout << "Now we check the the exponential mapping function." << endl << endl;

	q.setIdentity();
	v << 50*0.015708, 0., 0.; // This is affectively a one shot pi/4 rotation
	R = q2C(q);

	cout << "Starting with delta rotation: " << v.transpose() << "; we rotate from R" << endl << R << endl;
	orc.exmap(v, R);
	cout << "exmap(v, R)" << endl << R << endl << endl;

	q = C2q(R);

	cout << "rotated quaterion: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << endl << endl;

	std::cout << "OrientationComputer::exmap -- R after" << std::endl << R << std::endl << std::endl;

	std::cout << "Now we run through some data to ensure the exmap is working" << std::endl << std::endl;

	double dt;
	dt = 0.001;
	double data[ITERATIONS][3];
	Eigen::Vector3d E;

	setupData(data);

	// Initialize the rotation matrix to be used
	R.setIdentity();
	E.setZero();

	string str;

	int k;
	for (k=0;k<ITERATIONS;k++) {
		v << data[k][0], data[k][1], data[k][2];
		orc.exmap(v*dt,R);
		//		std::cout << "OrientationComputer::exmap -- R iteration " << k << std::endl << R << std::endl << std::endl;
		E = C2e(R);
		//		std::cout << "OrientationComputer::exmap -- E " << E.transpose() << std::endl;
		log << to_string(E(0)) + ", " + to_string(E(1)) + ", " + to_string(E(2)) + "\n";

	}

	// Close out the log file
	log.Close();

	cout << "Now we test the quaternion feedback update function " << endl;

	orc.updateOrientation(0,Eigen::Quaterniond::Identity());

    E << 0.1, 0, 0;
    orc.rotateOrientationUpdate(E);


    cout << "orc.q() is " << orc.q().w() << ", " << orc.q().x() << ", " << orc.q().y() << ", " << orc.q().z() << endl;
    cout << "q2e(orc.q()) = " << q2e_new(orc.q()).transpose() << endl;


	return 0;
}



void setupData(double data[][3]) {
	int i,j;
	for (i=0;i<ITERATIONS;i++) {
		for (j=0;j<3;j++) {
			data[i][j] =0.;
		}
	}

	for (i=50;i<100;i++) {
		data[i][0] = 5*PI__;
	}

}

