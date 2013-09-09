#ifndef TWOLEGSESTIMATE_TYPES_H_
#define TWOLEGSESTIMATE_TYPES_H_

#include <Eigen/Dense>
#include <iostream>

//#include "TwoLegOdometry.h"

#define LEFTFOOT  0
#define RIGHTFOOT 1

namespace TwoLegs {

	
	typedef struct {
		Eigen::Vector3d position;
		Eigen::Quaterniond orientation;
	} state;
	
	typedef struct {
		Eigen::Vector3d offset;
		Eigen::Quaterniond rotation;
	} transformation;
	
	typedef struct {
		Eigen::Isometry3d footprintlocation;
		int foot;
	} footstep;
	
	typedef struct {
	  float x;
	  float y;
	  float z;
	} footforces;
	
}

#endif /*TWOLEGSESTIMATE_TYPES_H_*/
