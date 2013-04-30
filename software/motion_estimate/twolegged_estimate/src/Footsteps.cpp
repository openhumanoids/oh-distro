
#include "Footsteps.h"
#include "QuaternionLib.h"

using namespace TwoLegs;

Footsteps::Footsteps() {
	std::cout << "New Footsteps object created" << std::endl;
}

void Footsteps::newFootstep(footstep newstep) {
	footstep_hist.push_back(newstep);
	active_step = newstep;
}

void Footsteps::addFootstep(Eigen::Isometry3d RelativeFrameLocation, int foot) 
{
	// Presently this function is only called once, to be combined -- TODO
	footstep new_footprint;
	
	new_footprint.foot = foot;
	new_footprint.footprintlocation = RelativeFrameLocation;
	
	footstep_hist.push_back(new_footprint);
	
	active_step = new_footprint;
}

Eigen::Isometry3d Footsteps::getLastStep() {
	return active_step.footprintlocation;
}

int Footsteps::lastFoot() {
	return active_step.foot;
}

void Footsteps::reset() {
	footstep_hist.clear();
}
