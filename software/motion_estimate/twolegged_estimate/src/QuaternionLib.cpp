
#include "QuaternionLib.h"

namespace InertialOdometry 
{
  
  // The result is pushed back into a2 (overwrites what was in there)
  void QuaternionLib::QuaternionProduct(double *a1, double *a2)
  {  
    Eigen::Matrix4d b;
    Eigen::Vector4d c;
    Eigen::Vector4d r;

    b(0,0) = a1[0];
    b(0,1) = -a1[1];
    b(0,2) = -a1[2];
    b(0,3) = -a1[3];

    b(1,0) = a1[1];
    b(1,1) = a1[0];
    b(1,2) = -a1[3];
    b(1,3) = a1[2];

    b(2,0) = a1[2];
    b(2,1) = a1[3];
    b(2,2) = a1[0];
    b(2,3) = -a1[1];

    b(3,0) = a1[3];
    b(3,1) = -a1[2];
    b(3,2) = a1[1];
    b(3,3) = a1[0];

    for (int i=0;i<4;i++)
      c(i) = a2[i]; 

    r = b*c;

    for (int i=0;i<4;i++)
      a2[i] = r(i); 

  }

  // Result is pushed back by overwritting the argument variable 
  void QuaternionLib::QuaternionAdjoint(double *q)
  {
    for (int i=1;i<4;i++)
      q[i] = -q[i];
  }
  

  // q is the rotation quaternion
  // v is the vector to be rotated, assumed to be size 3
  // Result is pushed back by overwriting the v variable
//  void QuaternionLib::VectorRotation(double *q, double *v)
  void QuaternionLib::VectorRotation(Eigen::Vector4d q, Eigen::Vector3d &v)
  {
    double v_[4];
    double q_[4];
    v_[0] = 0.0;

    int i;

    for (int i=0;i<4;i++)
      q_[i] = q(i);
    
    for (int i=1;i<4;i++)
      v_[i] = v(i-1);

    QuaternionProduct(q_,v_);
    QuaternionAdjoint(q_);
    QuaternionProduct(v_,q_);

    // Put the result back in the pointer to push back to caller
    for (int i=1;i<4;i++)
      v(i-1) = q_[i];

    //std::cout << " QuaternionLib::VectorRotation happened." << std::endl;
    
  }

  //This function is specific to NED and forward right down coordinate frames
  void QuaternionLib::q2e(Eigen::Vector4d q_, Eigen::Vector3d &E)
  {
  	//Euler = [phi;theta;psi];


  	double a,b,c,d;

  	a = q_(0);
  	b = q_(1);
  	c = q_(2);
  	d = q_(3);

  	/*
  	phi = atan2(2*(c*d + b*a),a^2-b^2-c^2+d^2);
  	theta = asin(-2*(b*d - a*c));
  	psi = atan2(2*(b*c + d*a),a^2+b^2-c^2-d^2);*/

  	/*
  	 * As Michael does it..
  	roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  	pitch = asin(2*(q0*q2-q3*q1));
  	yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
	*/
  	/*
  	//Equivalent test
  	E(0) = atan2(2.0*(c*d + b*a),1.0-2.0*(b*b+c*c));
  	E(1) = asin(-2.0*(b*d - a*c));
  	E(2) = atan2(2.0*(b*c + d*a),1.0-2.0*(c*c+d*d));
  	*/

  	// This should be numerically more stable than the 1-(^2+ ^2) version..
  	// seems to be good
  	E(0) = atan2(2.0*(c*d + b*a),a*a-b*b-c*c+d*d);
  	E(1) = asin(-2.0*(b*d - a*c));
  	E(2) = atan2(2.0*(b*c + d*a),a*a+b*b-c*c-d*d);

  	return;
  }

  void QuaternionLib::skew(Eigen::Vector3d const &v_, Eigen::Matrix<double,3,3> &skew)
  {
	  
	  skew << 0., -v_(2), v_(1), 
	  		  v_(2), 0., -v_(0),
	  		  -v_(1), v_(0), 0.;
	  
	  return;
  }
  
  void QuaternionLib::q2C(Eigen::Vector4d const &q_, Eigen::Matrix<double,3,3> &C)
  {
	  // scalar vector format
	  //http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	  // 3/10/2013
	  const double w = q_(0);
	  const double x = q_(1);
	  const double y = q_(2);
	  const double z = q_(3);
	  
	  C(0,0) = w*w + x*x - y*y - z*z;
	  C(1,0) = 2.*(x*y - w*z);
	  C(2,0) = 2.*(w*y + x*z);
	  C(1,1) = w*w - x*x + y*y - z*z;
	  C(0,1) = 2.*(x*y + w*z);
	  C(2,1) = 2.*(y*z - w*x);
	  C(2,2) = w*w - x*x - y*y + z*z;
	  C(0,2) = 2.*(x*z - w*y);
	  C(1,2) = 2.*(w*x + y*z);
	  std::cout << "TODO: QuaternionLib::q2C(..) is UNTESTED" << std::endl;
	  
	  return;
  }
  
  void QuaternionLib::e2C(Eigen::Vector3d const &E, Eigen::Matrix<double, 3, 3> &C)
  {
	  double st, sp, sps;
	  double ct, cp, cps;
	  
	  sp = sin(E(0));
	  st = sin(E(1));
	  sps = sin(E(2));
	  
	  cp = cos(E(0));
	  ct = cos(E(1));
	  cps = cos(E(2));
	  
	  C(0,0) = cps*ct;
	  C(1,0) = sps*ct;
	  C(2,0) = -st;
	  
	  C(0,1) = -sps*cp + cps*st*sp;
	  C(1,1) = cps*cp + sps * st * sp;
	  C(2,1) = ct*sp;
	  
	  C(0,2) = sps * sp + cps * st * cp;
	  C(1,2) = -cps*sp + sp * st * cp;
	  C(2,2) = ct * cp;
  }
}

