#include <iostream>
#include <Eigen/Dense>
#include "QuaternionLib.h"
#include "SignalTap.hpp"
#include "Filter.hpp"

using namespace std;
using namespace InertialOdometry;

#define PI  3.14159265358

int main() {
	cout << "Testing Isometries\n";
	
	Eigen::Vector3d E;
	
	E << 0., 0., PI/3.;
	
	cout << "The reference Euler angles are: " << E.transpose() << endl;
	
	Eigen::Isometry3d iso_0;
	Eigen::Isometry3d iso_1(QuaternionLib::e2q(E));
	Eigen::Isometry3d iso_2(QuaternionLib::e2q(E));
	Eigen::Isometry3d iso_3(QuaternionLib::e2q(E));
	Eigen::Isometry3d iso_4(QuaternionLib::e2q(Eigen::Vector3d::Zero()));
	Eigen::Isometry3d iso_5(QuaternionLib::e2q(Eigen::Vector3d(0.,0.,PI)));
	
	iso_1.translation() << 1., 0., 0.;
	iso_2.translation() << 1., 0., 0.;
	iso_3.translation() << 1., 0., 0.;
	iso_4.translation() << -1., -1.73205, 0.;
		
	iso_0 = iso_4*iso_1*iso_2*iso_3*iso_5;
	
	cout << "Translation is: " << iso_0.translation().transpose() << endl;
	cout << "Rotation is:\n" << iso_0.rotation() << endl;
	cout << "Linear is:\n" << iso_0.linear() << endl;
	cout << "Diff between them is:\n" << (iso_0.rotation() - iso_0.linear()) << endl;
	cout << "Orientation in Euler: " << QuaternionLib::C2e(iso_0.rotation()).transpose() << endl;
	Eigen::Quaterniond q;
	q = QuaternionLib::C2q(iso_0.rotation());
	cout << "Orientation in Quaternion: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() <<endl;
	cout << "Checking q2e: " << QuaternionLib::q2e(q).transpose() << endl;
	
	
	cout << "Testing the C2q function\n";
	
	Eigen::Matrix3d C;
	Eigen::Vector3d refE;
	
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;
	
	refE << -0.5,0.2,-1.;
	C = InertialOdometry::QuaternionLib::e2C(refE);
	
	q1 = InertialOdometry::QuaternionLib::e2q(refE);
	cout << "Reference:\n";
	cout << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << endl;
	cout << InertialOdometry::QuaternionLib::q2e(q1).transpose() << endl;
	cout << "Testing:\n";
	q2 = InertialOdometry::QuaternionLib::C2q(C);
	//q2 = InertialOdometry::QuaternionLib::bot_matrix_to_quat(C);
	cout << "TEST RESULT: " << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << endl;
	cout << InertialOdometry::QuaternionLib::q2e(q1).transpose() << endl;
		
	cout << "Testing the Schmitt trigger ============\n";
	
	SchmittTrigger trigger(9., 12., 4);
	// check hig side trigger
	for (int i=0;i<20;i++) {
		trigger.UpdateState(i, i);
		if (trigger.getState()) {
			cout << "Trigger is true at: " << i << ", Value is: " << trigger.getCurrentValue() << endl;
		}
	}
	// check low side trigger
	for (int i=0;i<20;i++) {
		trigger.UpdateState(20+i, 20.-i);
		if (!trigger.getState()) {
			cout << "Trigger is false at: " << 20+i << ", Value is: " << trigger.getCurrentValue() << endl;
		}
	}
	for (int i=0;i<25;i++) {
		if (i>14) {
			trigger.UpdateState(40+i-2, i-5);
		} else {
			trigger.UpdateState(40+i, i);
		}
		
		if (trigger.getState()) {
			cout << "Trigger is true at: " << i << ", Value is: " << trigger.getCurrentValue() << endl;
		}
	}


	LowPassFilter _filter;

	for (int i =0;i<10;i++) {
		_filter.processSample(50.0);
	}

	cout << "Testing the integral\n";


	TrapezoidalInt integrator;
	NumericalDiff differ;
	integrator.setSize(1);
	differ.setSize(1);

	double dx[1];
	double val[1];

	dx[0] = 0.12;

	for (int i=0;i<50;i++) {
		dx[0] = rand();
		dx[0] = (dx[0]/1E8);
		dx[0] = 2.*((dx[0]-(int)dx[0])-0.5) + 10;

		//cout << "val: " << dx[0] <<endl;


		val[0] = integrator.integrate((unsigned long long)1E3*i, 1,dx)(0);
		cout << " | " << fixed << dx[0];
		if ((i+1)%5==0) {
			cout << " diff: ";
			differ.diff((unsigned long long)1E3*i,1,val);
			cout << val[0] << endl;
		}

	}


	cout << endl;

	cout << "============TESTING DistributedDiff+++++++++++++\n";

	DistributedDiff diffs;
	diffs.setSize(1);

	int tapsize = 12;
	Eigen::VectorXd w(tapsize);
	Eigen::VectorXd ut(tapsize);

	Eigen::VectorXd sample(1);

	w.setZero();
	w(0) = .55;
	w(3) = .25;
	w(7) = .05;
	w(8) = .05;
	w(9) = .05;
	w(11) = .05;

	for (int i=1;i<tapsize+1;i++) {
		ut(i-1) = i*1E6;
	}


	diffs.InitializeTaps(tapsize, 1E6, w,ut);
	//diffs.ParameterFileInit();

	assert(diffs.ready());

	for (int i=1;i<20;i++) {
		sample << 10/(double)i;
		cout << "Returned diff: " << diffs.diff(i*1E6,sample) << "\n";
	}


	cout << endl;


	cout << "================ Testing Median Filtering =====================\n";

	MedianFilter mf;
	mf.setLength(7);

	double noiseval[20] = {0.0975,
		    0.2785,
		    0.5469,
		    0.9575,
		    0.9649,
		    0.1576,
		    0.9706,
		    0.9572,
		    0.4854,
		    0.8003,
		    0.1419,
		    0.4218,
		    0.9157,
		    0.7922,
		    0.9595,
		    0.6557,
		    0.0357,
		    0.8491,
		    0.9340,
		    0.6787};

	for (int i=0;i<20;i++) {
		cout << mf.processSample(noiseval[i]) << ", ";
	}
	cout << endl;



	return 0;
}
