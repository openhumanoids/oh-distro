
#include <iostream>


#include "TwoLegOdometry.h"
#include "LegOdometry_LCM_Handler.hpp"

using namespace std;

TwoLegs::TwoLegOdometry* _leg_odo; // excessive, as this gets tested in _legs_motion_estimate
LegOdometry_Handler* _legs_motion_estimate;

int main() {
	cout << "Test driver main function for the twoleg motion estimate pod" << endl;

	
	_leg_odo = new TwoLegs::TwoLegOdometry(); // This is excessive, as the class is also invoked by LegOdometry_Handler() object.
	_legs_motion_estimate = new LegOdometry_Handler();
	
	
	// Do some stuff with the objects to test them. Preferably here you must call the internal testing functions of the different objects created..
	_legs_motion_estimate->run(true); // true means it will operate in testing mode and not listen LCM messages
	
	
	delete _leg_odo;
	delete _legs_motion_estimate;
	
	cout << "Everything ends in test_driver for legs_motion_estimate program" << endl; 
	
	return 0;
}


// here somewhere we need to listen to the LCM packets and call back the function in the TwoLegOdometry class as to update its internal states
