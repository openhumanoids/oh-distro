

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
	Eigen::Isometry3d first;
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
		Eigen::Isometry3d transform;
		

		Eigen::Isometry3d temp;
		temp = getSecondaryInLocal();
#ifdef VERBOSE_DEBUG
		std::cout << "Adding footstep with values: " << temp.translation().transpose() << std::endl;
#endif
		footsteps.addFootstep(temp,secondary_foot());
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

void TwoLegOdometry::setLegTransforms(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right) {
	pelvis_to_left = left;
	pelvis_to_right = right;
	
	
	// TODO - do the rotation assignments for for the inverse directions
	left_to_pelvis.translation() = -left.translation();
	//left_to_pelvis.rotation() = left.inverse().rotation();
	right_to_pelvis.translation() = -right.translation();
	//right_to_pelvis.rotation() = right.rotation();

}

Eigen::Isometry3d TwoLegOdometry::getPelvisFromStep() {
	Eigen::Isometry3d returnval;
	
	//Eigen::Isometry3d testcase;
	//Eigen::Isometry3d testcase2;
	//Eigen::Isometry3d addresult;
			
	//testcase.translation() << 10,10,10;
	//testcase2.translation() << 1,1,1;

	
	//std::cout << "TwoLegOdometry::getPelvisFromStep(): Last step=" << footsteps.getLastStep().translation().transpose() << std::endl;
	//std::cout << "Test case: " << testcase.translation().transpose() << std::endl;
	
	//addresult = add(testcase,testcase2);
	//std::cout << "Add case: " << addresult.translation().transpose() << std::endl;
	
	if (primary_foot() == LEFTFOOT)
	{
		if (footsteps.lastFoot() != LEFTFOOT) {
		  cout << "LEFT RIGHT ACTIVE FOOT SEQUENCING IS INCONSISTENT TwoLegOdometry::getPelvisFromStep()\n";
		}
		//returnval = add(footsteps.getLastStep(),left_to_pelvis);
		returnval = add(footsteps.getLastStep(),pelvis_to_left);
				
		//std::cout << " left  ";
		//return addTransforms(footsteps.getLastStep(), left_to_pelvis);
		
	}
	else
	{
		if (footsteps.lastFoot() != RIGHTFOOT) {
		  cout << "LEFT RIGHT ACTIVE FOOT SEQUENCING IS INCONSISTENT TwoLegOdometry::getPelvisFromStep()\n";	
		}
		//returnval = add(footsteps.getLastStep(),right_to_pelvis);
		returnval = add(footsteps.getLastStep(),pelvis_to_right);
		//std::cout << " right ";
		//return addTransforms(footsteps.getLastStep(), right_to_pelvis);
	}
	
	//std::cout << "returnval: " << returnval.translation().transpose() << std::endl;
	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getSecondaryInLocal() {
	
	//std::cout << "pelvis to left: " << (pelvis_to_left*local_to_pelvis).translation().transpose() << std::endl;
	
	Eigen::Isometry3d returnval;
	returnval.translation() << -99999999999., -99999999999.,-99999999999.;
	
	//std::cout << "TwoLegOdometry::getSecondaryInLocal(): local_to_pelvis: " << local_to_pelvis.translation().transpose() << std::endl;
	
	switch (secondary_foot()) {
	case LEFTFOOT:
		returnval = add(add(getPrimaryInLocal(),left_to_pelvis),pelvis_to_right);
		break;
	case RIGHTFOOT:
		returnval = add(add(getPrimaryInLocal(),right_to_pelvis),pelvis_to_left);
		break;
	default:
		std::cout << "THIS SHOULD NEVER HAPPEN - TwoLegOdometry::getSecondaryFootTransform()" << std::endl;
		break;
	}
	//std::cout << "TwoLegOdometry::getSecondaryInLocal(): " << returnval.translation().transpose() << std::endl;
	
	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getPrimaryInLocal() {
	return footsteps.getLastStep();
	
}

void TwoLegOdometry::setPelvisPosition(Eigen::Isometry3d transform) {
	local_to_pelvis = transform;
}


Eigen::Isometry3d TwoLegOdometry::add(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs) {
	Eigen::Isometry3d add;
	
	add.translation() = lhs.translation() + rhs.translation();
	
	//TODO - Test and confirm the correct rotation sequence has been followed here
	add.rotate(lhs.rotation());
	add.rotate(rhs.rotation());
	
	return add;
}

Eigen::Quaterniond TwoLegOdometry::mult(Eigen::Quaterniond lhs, Eigen::Quaterniond rhs) {
	Eigen::Quaterniond result;
	
	Eigen::Vector4d q;
	Eigen::Vector4d p;
	Eigen::Vector4d res;
	//q*p - MARS Lab, Trawny Roumeliotis - Quaternion Algebra Tutorial Tech Report
	q << rhs.x(), rhs.y(), rhs.z(), rhs.w();
	
	
	Eigen::Matrix<double,4,4> Q;
	
	Q << 	q(3), q(2), -q(1), q(0),
		   -q(2), q(3), q(0), q(1),
		   q(1), -q(0), q(3), q(2),
		   -q(0), -q(1), -q(2), q(3);
	
	p << lhs.x(), lhs.y(), lhs.z(), lhs.w();
	
	res = Q*p;
	
	result.w() = res(3);
	result.x() = res(0);
	result.y() = res(1);
	result.z() = res(2);
	
	return result;
}

void TwoLegOdometry::ResetInitialConditions() {
	// this function assumes the pelvis is at the 0 position.
	// The left foot is used as the initial condition for the system.
	
	Eigen::Vector3d zero;
	zero << 0.,0.,0.;
	
	local_to_pelvis.translation() = zero;
	
	footsteps.reset();
}

