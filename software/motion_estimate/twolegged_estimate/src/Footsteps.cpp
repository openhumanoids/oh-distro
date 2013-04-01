
#include "Footsteps.h"

using namespace TwoLegs;

Footsteps::Footsteps() {
	std::cout << "New Footsteps object created" << std::endl;
}


void Footsteps::addFootstep(state RelativeFrameLocation, int foot) 
{
	//std::cout << "addFootstep function was called for Footsteps class" << std::endl;
	
	footstep new_footprint;
	
	/*
	new_footprint.footprintlocation.position = RelativeFrameLocation.translation;
	new_footprint.footprintlocation.orientation = RelativeFrameLocation.rotation;
	new_footprint.foot = foot;
	
	active_step.footprintlocation.position = RelativeFrameLocation.translation;
	active_step.footprintlocation.orientation = RelativeFrameLocation.rotation;
	*/
	
	active_step.foot = foot;
	
	footstep_hist.push_back(new_footprint);
	
}

Eigen::Isometry3d Footsteps::getLastStep() {
	//drc::transform_t trans;
	
	return active_step.footprintlocation;
		
	// TODO - decide on a final rotational representation and convert this part of the code to use drc types
	
	//return trans;
}

int Footsteps::lastFoot() {
	return active_step.foot;
}
