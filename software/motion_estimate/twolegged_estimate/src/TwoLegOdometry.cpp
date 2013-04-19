

#include <iostream>
#include <cmath>

#include "TwoLegOdometry.h"

#include "TwoLegsEstimate_types.h"

#include "QuaternionLib.h"


using namespace TwoLegs;
using namespace std;

TwoLegOdometry::TwoLegOdometry()
{
	cout << "A new TwoLegOdometry object was created" << endl;
	
	// This is just here initially for initial testing and setup of the code structure
	// TODO
	// Assuming the robot is standing still.
	//Eigen::Isometry3d first;
	footsteps.reset();
	standing_foot = -1;
	
	foottransitionintermediateflag = true;
	standingintermediate = true;
	
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
	standing_timer = 0;
	standing_delay = 0;
	
	stepcount = 0;
	
	both_feet_in_contact = true;
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



// now this seems excessive and should maybe be a more general transform update thing used for legs and head and pelvis
state TwoLegOdometry::getSecondaryFootState() {
	state secondaryfoot_state;
	
	//secondaryfoot_state = ??
	
	return secondaryfoot_state;
}

void TwoLegOdometry::setStandingFoot(int foot) {
	standing_foot = foot;
}

// TODO - unused function, to be depreciated
void TwoLegOdometry::updateInternalStates(/*data*/) {
	cout << "void TwoLegOdometry::updateInternalStates(); nothing updated - NOT implemented" << endl;
	
	return;
}

int TwoLegOdometry::primary_foot() {
	return standing_foot;
}

footstep TwoLegOdometry::DetectFootTransistion(int64_t utime, float leftz, float rightz) {
	
	deltautime =  utime - lcmutime;
	lcmutime = utime;
	leftforces.z = leftz;
	rightforces.z = rightz;
	
	footstep newstep;
	newstep.foot = -1;
	
	if (getSecondaryFootZforce() - SCHMITT_LEVEL*expectedweight > getPrimaryFootZforce()) {
	  transition_timespan += deltautime;
	}
	else
	{
	  transition_timespan = 0.;
	  foottransitionintermediateflag = true;
	  
	  // in the intermediate zone
	  // potentially standing
	}
	  
	if (transition_timespan > TRANSITION_TIMEOUT && foottransitionintermediateflag) 
	{
		Eigen::Isometry3d transform;
		
		
		Eigen::Isometry3d temp;
		temp = getSecondaryInLocal();
#ifdef VERBOSE_DEBUG
		std::cout << "Adding footstep with values: " << temp.translation().transpose() << std::endl;
#endif
		
		// This is to be depreciated
		//footsteps.addFootstep(temp,secondary_foot());
	  //setStandingFoot(secondary_foot());
	  //new modular function call

	  foottransitionintermediateflag = false;
	  

	  std::cout << "NEW STEP ON " << ((secondary_foot()==LEFTFOOT) ? "LEFT" : "RIGHT") << " stepcount: " << stepcount << " at x= " << getSecondaryInLocal().translation().x() << std::endl;

	  stepcount++;
	  newstep.foot = secondary_foot();
	  //newstep.footprintlocation = getSecondaryInLocal();
	  newstep.footprintlocation = AccumulateFootPosition(getPrimaryInLocal(), primary_foot());
	  
	  //standing must be removed in time from a foot transition event
  	  
  	  standing_delay = 5*STANDING_TRANSITION_TIMEOUT;
	  
	} else {

		
			double separation, loadsplit;
			
			loadsplit = abs(leftz*leftz - rightz*rightz) < (SCHMITT_LEVEL*expectedweight*SCHMITT_LEVEL*expectedweight);
			separation = abs(pelvis_to_left.translation().x()) + abs(pelvis_to_right.translation().x());
			
			// Second layer of logic testing to isolate the robot standing condition
			if (loadsplit && separation < MIN_STANDING_FEET_X_SEP && leftz > MIN_STANDING_FORCE && rightz > MIN_STANDING_FORCE) {
				
				if (standing_delay > 0) {
					standing_delay -= deltautime;
					//std::cout << "reducing standing delay to: " << standing_delay << std::endl;
				} else {
					standing_delay = 0;
				}
				
				//25346000 - 25153000 = 193000
				//25346000 - 25158000 = 188000
				//25345000 - 25153000 = 192000
				standing_timer += deltautime;
			}
			else
			{
				if (standing_timer>STANDING_TRANSITION_TIMEOUT)
				{
					standing_timer = STANDING_TRANSITION_TIMEOUT;
					standingintermediate = true;
				}
				standing_timer -= deltautime;
				if (standing_timer<0) {
					standingintermediate = false;
					standing_timer = 0;
					
					both_feet_in_contact = false;
				}
				else
				{
					//std::cout << "Standing timer reduced to : " << standing_timer << "\n";
				}
			}
			
			if ((standing_timer > STANDING_TRANSITION_TIMEOUT && standing_delay<=0) || standingintermediate) {
				//std::cout << "Standing for: " << standing_timer <<  "\n";
				both_feet_in_contact = true;
			}
			else
			{
				
			}
			
			if (both_feet_in_contact)
			{
				std::cout << "Standing for: " << standing_timer <<  "\n";
			}
				
	}
	
	return newstep;
}

bool TwoLegOdometry::FootLogic(long utime, float leftz, float rightz) {
  footstep newstep;
  newstep = DetectFootTransistion(utime, leftz, rightz);
  
  if (newstep.foot == LEFTFOOT || newstep.foot == RIGHTFOOT) {
	  std::cout << "FootLogic adding Footstep " << (newstep.foot == LEFTFOOT ? "LEFT" : "RIGHT") << std::endl;
	  standing_foot = newstep.foot;
	  footsteps.newFootstep(newstep);
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
	
	//std::cout << "Setting leg transforms\n";
	
	// TODO - do the rotation assignments have not been tested yet
	left_to_pelvis.translation() = -left.translation();
	left_to_pelvis.linear() = left.linear().transpose();
	right_to_pelvis.translation() = -right.translation();
	right_to_pelvis.linear() = right.linear().transpose();
}

Eigen::Isometry3d TwoLegOdometry::getSecondaryFootToPelvis() {
	//std::cout << "Taking primary as: " << (footsteps.lastFoot()==LEFTFOOT ? "LEFT" : "RIGHT") << std::endl;
	if (footsteps.lastFoot() == LEFTFOOT)
		return right_to_pelvis;
	if (footsteps.lastFoot() == RIGHTFOOT)
		return left_to_pelvis;
	return Eigen::Isometry3d();
}

Eigen::Isometry3d TwoLegOdometry::getPrimaryFootToPelvis() {
	if (footsteps.lastFoot() == LEFTFOOT)
		return left_to_pelvis;
	if (footsteps.lastFoot() == RIGHTFOOT)
		return right_to_pelvis;
	return Eigen::Isometry3d();
}

Eigen::Isometry3d TwoLegOdometry::getPelvisFromStep() {
	//std::cout << "Last step: " << footsteps.getLastStep().translation().transpose() << "\n";
	//std::cout << "Primary to Pelvis: " << getPrimaryFootToPelvis().translation().transpose() << "\n";
	//std::cout << "Add: " << add(footsteps.getLastStep(),getPrimaryFootToPelvis()).translation().transpose() << "\n";
	
	return add(footsteps.getLastStep(),getPrimaryFootToPelvis());
}

Eigen::Isometry3d TwoLegOdometry::AccumulateFootPosition(const Eigen::Isometry3d &from, const int foot_id) {
	Eigen::Isometry3d returnval;
	returnval.translation() << -99999999999., -99999999999.,-99999999999.;
	
	switch (foot_id) {
	case LEFTFOOT:
		returnval = add(add(from,left_to_pelvis),pelvis_to_right);
		break;
	case RIGHTFOOT:
		//std::cout << "Going right: " << getPrimaryInLocal().translation().x() << ", " << right_to_pelvis.translation().x() << ", " << pelvis_to_left.translation().x() << std::endl;
		returnval = add(add(from,right_to_pelvis),pelvis_to_left);
		break;
	default:
		std::cout << "THIS SHOULD NEVER HAPPEN - TwoLegOdometry::AccumulateFootPosition()" << std::endl;
		break;
	}
	
	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getSecondaryInLocal() {
	
	//std::cout << "pelvis to left: " << (pelvis_to_left*local_to_pelvis).translation().transpose() << std::endl;
	//std::cout << (secondary_foot()==LEFTFOOT ? "LEFT" : "RIGHT") << " is secondary_foot()" << std::endl;
	Eigen::Isometry3d returnval;
	returnval = AccumulateFootPosition(getPrimaryInLocal(),primary_foot());
	
	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getPrimaryInLocal() {
	return footsteps.getLastStep();
	
}

Eigen::Isometry3d TwoLegOdometry::getLeftInLocal() {
	return add(getPelvisFromStep(), pelvis_to_left);
}


Eigen::Isometry3d TwoLegOdometry::getRightInLocal() {
	return add(getPelvisFromStep(), pelvis_to_right);
}

void TwoLegOdometry::setPelvisPosition(Eigen::Isometry3d transform) {
	local_to_pelvis = transform;
}


Eigen::Isometry3d TwoLegOdometry::add(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs) {
	Eigen::Isometry3d add;
	
	add.translation() = lhs.translation() + InertialOdometry::QuaternionLib::Cyaw_rotate(lhs.linear() ,InertialOdometry::QuaternionLib::Cyaw_rotate(rhs.linear(),rhs.translation()));
	
	//TODO - Test and confirm the correct rotation sequence has been followed here
	//add.rotate(lhs.rotation());
	//add.rotate(rhs.rotation());
	
	//std::cout << "Going to rotation matrices for adding operation:\n" << lhs.linear() << std::endl << rhs.linear() << std::endl;
	
	add.linear() = lhs.linear() * rhs.linear();
	
	//std::cout << "Product:\n" << add.linear() << std::endl;
	
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

void TwoLegOdometry::ResetInitialConditions(const Eigen::Isometry3d &left_) {
	// The left foot is used to initialise height of the pelvis.
	
	Eigen::Vector3d zero;
	zero << 0.,0.,-left_.translation().z();
	
	stepcount = 0;
	
	local_to_pelvis.translation() = zero;
	local_to_pelvis.linear().setIdentity();
	
	footsteps.reset();
}

void TwoLegOdometry::ResetWithLeftFootStates(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &right_) {
	
	ResetInitialConditions(left_);
	
	//std::cout << "Pelvis was set to: " << local_to_pelvis.translation().transpose() << std::endl;
	//std::cout << "Last step location before add: " << footsteps.getLastStep().translation().transpose() << std::endl;
	//std::cout << "PrimaryFoot to Pelvis: " << getPrimaryFootToPelvis().translation().transpose() << std::endl;
	
	footsteps.addFootstep(add(local_to_pelvis,left_),LEFTFOOT);
	//std::cout << "Last step location after add: " << footsteps.getLastStep().translation().transpose() << std::endl;
	//footsteps.addFootstep(pelvis_to_left,LEFTFOOT);
	standing_foot = LEFTFOOT; // Not sure that double states should be used, this needs to change TODO
}

int TwoLegOdometry::getStepCount() {
	return stepcount;
}

int TwoLegOdometry::getActiveFoot() {
	return standing_foot;
}

float TwoLegOdometry::leftContactStatus() {
	if (getActiveFoot() == LEFTFOOT || both_feet_in_contact) {
		return 1.0f;
	}
	return 0.0f;
}

float TwoLegOdometry::rightContactStatus() {
	if (getActiveFoot() == RIGHTFOOT || both_feet_in_contact) {
		return 1.0f;
	}
	return 0.0f;
}
