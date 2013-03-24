#ifndef FOOTSTEPS_H_
#define FOOTSTEPS_H_

#include "TwoLegsEstimate_types.h"

namespace TwoLegs {

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

#endif /*FOOTSTEPS_H_*/
