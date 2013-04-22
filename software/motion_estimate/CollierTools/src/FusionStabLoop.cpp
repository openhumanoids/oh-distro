
#include "FusionStabLoop.hpp"
#include <iostream>


FusionStabLoop::FusionStabLoop(int num_channels) {
	
	channels.resize(num_channels);
	
	gain = FEEDBACK_GAIN_PRESET;
	
	_feedback_integrator = new TrapezoidalIntegrator(3,1/30.);
	
	std::cout << "FusionStabLoop object created with " << channels.size() << " channels\n";
	
}

FusionStabLoop::~FusionStabLoop() {
	std::cout << "Closing out a FusionStabLoop object" << std::endl;
	
	delete _feedback_integrator;
	
}

