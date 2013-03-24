#ifndef FOOTSTEPS_H_
#define FOOTSTEPS_H_

#include "TwoLegsEstimate_types.h"
#include <vector>

namespace TwoLegs {

class Footsteps {
	private:
		footstep active_step;
		std::vector<footstep> footstep_hist;
		int thisisatestvariable;
		
	public:
		Footsteps();
		void addFootstep(state const &RelativeFrameLocation, int foot);
	
};

}

#endif /*FOOTSTEPS_H_*/
