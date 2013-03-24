
#include <iostream>
#include "TwoLegsEstimate_types.h"

using namespace std;
using namespace TwoLegs;

int main() {
	cout << "Test driver main function for the twoleg_estimate pod" << endl;

	Footsteps footsteps;
	transformation tommy;
	
	footsteps.addFootstep(tommy,LEFTFOOT);
	
	
	return 0;
}
