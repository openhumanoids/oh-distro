#ifndef TWOLEGODOMETRY_H_
#define TWOLEGODOMETRY_H_

#include "TwoLegsEstimate_types.h"
#include "Footsteps.h"

namespace TwoLegs {

class TwoLegOdometry {
	private:
		Footsteps footsteps;
		
		// Convert the LCM message containing robot pose information to that which is requried by the leg odometry calculations
		void parseRobotInput();
		
	public:
		TwoLegOdometry();
		
};


}

#endif /*TWOLEGODOMETRY_H_*/
