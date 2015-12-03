#include "boost/filesystem.hpp"

#include <Eigen/Dense>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <math.h>       /* cos, sin */

using namespace std;

/*Compute 2D transformation errors given transformation from ground truth
and transformation from registration in the form [x,y,theta] (m,m,deg).*/

float translationError(float xg, float yg, float xr, float yr);

float rotationError(float xg, float yg, float thetag, float xr, float yr, float thetar);

/*Get Eigen 3X3 transformation matrix in the form [x,y,theta] (m,m,deg), 
where theta is the rotation about the z azis.*/

Eigen::Matrix3f parseTransformation(float x, float y, float theta); 