

#include <iostream>

#include "TwoLegOdometry.h"

#include "TwoLegsEstimate_types.h"


using namespace TwoLegs;
using namespace std;

TwoLegOdometry::TwoLegOdometry()
{
	cout << "A new TwoLegOdometry object was created" << endl;
	
	// This is just here initially for initial testing and setup of the code structure
	// TODO
	// Assuming the robot is standing still.
	state first;
	footsteps.addFootstep(first,LEFTFOOT);
	standing_foot = LEFTFOOT;
	
	foottransitionintermediateflag = true;
	
	// TODO - expected weight must be updated from live vehicle data and not be hard coded like this
	expectedweight = 900.f;
	
	leftforces.x = 0.f;
	leftforces.y = 0.f;
	leftforces.z = 0.f;
	
	rightforces.x = 0.f;
	rightforces.y = 0.f;
	rightforces.z = 0.f;
	
	lcmutime = 0;
	deltautime = 0;
	transition_timespan = 0;
}

void TwoLegOdometry::parseRobotInput() {
	
	cout << "TwoLegOdometry::parseLCMInput() called, NOT implemented" << endl;

		
	return;
}

/*
 * Updates the internal states of the odometry object and under the correct circumstances transistions the standing foot as the robot is walking
*/
void TwoLegOdometry::CalculateBodyStates_Testing(int counter) {
	cout << "_leg_odo::CalculateBodyStates() NOT implemented yet" << endl;
	
	// calculate the new pelvis and head states
	// create some data and pass it to this function
	
	/*data = ? */
	
	CalculateBodyStates(/*data*/);
	
	return;
}

int TwoLegOdometry::secondary_foot() {
	if (standing_foot == LEFTFOOT)
		return RIGHTFOOT;
	if (standing_foot == RIGHTFOOT)
		return LEFTFOOT;
	
}

void TwoLegOdometry::CalculateBodyStates(/*data*/) {
	cout << "TwoLegOdometry::CalculateBodyStates() NOT IMPLEMENTED YET" << endl << endl;
	
	// Here we use the active footstep 
	// and add the transform to compute the pelvis postion from the active foot
	// from the pelvis calculate the other foot state
	// from the pelvis compute the head state
	
	// all states of the robot should now be updated in the local object states
	
	
	// calculate the new pelvis and head states
	// create some data and pass it to this function
	updateInternalStates(/*data*/); // some data must be passed to this function
	
	// Determine whether to transition feet
	//FootTransitionLogic(); - depreciated
	
	return;
}

void TwoLegOdometry::FootTransitionLogic() {
	/*
	if (DetemineIfFootTransistionRequired())
	{
#ifdef EARLY_DEV_COUTS
		cout << "Next footstep has been made, transitioning footsteps." << endl;
#endif
		
		// calculate new footstate relative to current best estimate of the robot states
		// This is now going to become the primary foot
		footsteps.addFootstep(getSecondaryFootState(),secondary_foot());
		setStandingFoot(secondary_foot());
		
	}
	*/
}

// now this seems excessive and should maybe be a more general transform update thing used for legs and head and pelvis
state TwoLegOdometry::getSecondaryFootState() {
	state secondaryfoot_state;
	
	//secondaryfoot_state = ??
	
	return secondaryfoot_state;
}

void TwoLegOdometry::setStandingFoot(int foot) {
	standing_foot = foot;
}

void TwoLegOdometry::updateInternalStates(/*data*/) {
	cout << "void TwoLegOdometry::updateInternalStates(); nothing updated - NOT implemented" << endl;
	
	return;
}

int TwoLegOdometry::primary_foot() {
	return standing_foot;
}

bool TwoLegOdometry::DetectFootTransistion(long utime, float leftz, float rightz) {
	
	deltautime =  utime - lcmutime;
	lcmutime = utime;
	leftforces.z = leftz;
	rightforces.z = rightz;
	
	
	if (getSecondaryFootZforce() - SCHMIDT_LEVEL*expectedweight > getPrimaryFootZforce()) {
	  transition_timespan += deltautime;
	}
	else
	{
	  transition_timespan = 0.;
	  foottransitionintermediateflag = true;
	}
	  
	if (transition_timespan > TRANSITION_TIMEOUT && foottransitionintermediateflag) 
	{
		footsteps.addFootstep(getSecondaryFootState(),secondary_foot());
	  setStandingFoot(secondary_foot());
	  
	  foottransitionintermediateflag = false;
	  return true;
	}
	
	return false;
}

float TwoLegOdometry::getPrimaryFootZforce() {
  if (standing_foot == LEFTFOOT)
    return leftforces.z;
  return rightforces.z;
}

float TwoLegOdometry::getSecondaryFootZforce() {
	if (standing_foot == LEFTFOOT)
	    return rightforces.z;
	  return leftforces.z;
}

void TwoLegOdometry::setLegTransforms(const drc::transform_t &left, const drc::transform_t &right) {
	pelvis_to_left = left;
	pelvis_to_right = right;
	
	// TODO
	// Only translation inverses are set in this function, 
	// must still add the quaternions by inverting the vector portion of the quaternion
	
	left_to_pelvis.translation.x = left.translation.x;
	left_to_pelvis.translation.y = left.translation.y;
	left_to_pelvis.translation.z = left.translation.z;
	
	right_to_pelvis.translation.x = right.translation.x;
	right_to_pelvis.translation.y = right.translation.y;
	right_to_pelvis.translation.z = right.translation.z;
}
