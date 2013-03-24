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
		state footprintlocation;
		int foot;
	} footstep;
	
}

#endif /*TWOLEGSESTIMATE_TYPES_H_*/
