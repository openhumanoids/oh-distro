#include "statistics.h"

/*Compute 2D transformation errors given transformation from ground truth
and transformation from registration in the form [x,y,theta] (m,m,deg).*/

float translationError(float xg, float yg, float xr, float yr)
{
  float error = sqrt( pow (xg - xr, 2.0) + pow (yg - yr, 2.0) );

  return error;
}

float rotationError(float xg, float yg, float thetag, float xr, float yr, float thetar)
{
  Eigen::Matrix3f transf_gtruth = parseTransformation(xg,yg,thetag);
  Eigen::Matrix3f transf_registr = parseTransformation(xr,yr,thetar);

  Eigen::Matrix3f deltaTransf = transf_registr * transf_gtruth.inverse();

  float trace = deltaTransf(0,0)+deltaTransf(1,1)+deltaTransf(2,2);

  float error = acos ( (trace-1)/2 ) * 180.0 / M_PI;
  
  return error;
}

Eigen::Matrix3f parseTransformation(float x, float y, float theta) 
{
  Eigen::Matrix3f parsedTransf = Eigen::Matrix3f::Identity(3, 3);

  parsedTransf(0,0) = cos ( theta * M_PI / 180.0 );
  parsedTransf(0,1) = - sin ( theta * M_PI / 180.0 );
  parsedTransf(1,0) = sin ( theta * M_PI / 180.0 );
  parsedTransf(1,1) = cos ( theta * M_PI / 180.0 );

  parsedTransf(0,2) = x;
  parsedTransf(1,2) = y;

  //cout << "Parsed transform:" << endl << parsedTransf << endl;

  return parsedTransf;
}