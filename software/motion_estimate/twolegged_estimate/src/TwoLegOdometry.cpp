

#include <iostream>

#include "TwoLegOdometry.h"


using namespace TwoLegs;
using namespace std;

TwoLegOdometry::TwoLegOdometry()
{
	cout << "A new TwoLegOdometry object was created" << endl;
	
	// This is just here initially for initial testing and setup of the code structure
	// TODO
	state tommy;
	footsteps.addFootstep(tommy,LEFTFOOT);
	

	
}

void TwoLegOdometry::parseRobotInput() {
	
	cout << "TwoLegOdometry::parseLCMInput() called, not implemented" << endl;
	
	return;
}



