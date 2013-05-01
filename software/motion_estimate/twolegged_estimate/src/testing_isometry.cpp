#include <iostream>
#include <Eigen/Dense>
#include "QuaternionLib.h"

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
		
	
	
	return 0;
}