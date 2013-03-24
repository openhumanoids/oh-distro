#ifndef TWOLEGSESTIMATE_TYPES_H_
#define TWOLEGSESTIMATE_TYPES_H_

#include <Eigen/Dense>
#include <iostream>

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
	} footprint;
	
	
	class Footsteps {
		private:
			//std::vector<int> footstep_list;
			int thisisatestvariable;
		public:
			Footsteps() {
				std::cout << "New Footsteps object created" << std::endl;
			}
			void addFootstep(transformation const &RelativeFrameLocation, int foot) 
			{
				std::cout << "addFootstep function was called for Footsteps class" << std::endl;
				
			}
		
	};
	
}

#endif /*TWOLEGSESTIMATE_TYPES_H_*/
