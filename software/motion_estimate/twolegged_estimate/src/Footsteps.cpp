
#include "Footsteps.h"

using namespace TwoLegs;

Footsteps::Footsteps() {
	std::cout << "New Footsteps object created" << std::endl;
}


void Footsteps::addFootstep(state const &RelativeFrameLocation, int foot) 
{
	std::cout << "addFootstep function was called for Footsteps class" << std::endl;
	
	footstep new_footprint;
	new_footprint.footprintlocation.position = RelativeFrameLocation.position;
	new_footprint.footprintlocation.orientation = RelativeFrameLocation.orientation;
	new_footprint.foot = foot;
	
	active_step.footprintlocation.position = RelativeFrameLocation.position;
	active_step.footprintlocation.orientation = RelativeFrameLocation.orientation;
	active_step.foot = foot;
	
	footstep_hist.push_back(new_footprint);
	
}