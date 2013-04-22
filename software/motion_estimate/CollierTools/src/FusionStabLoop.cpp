
#include "FusionStabLoop.hpp"
#include <iostream>


FusionStabLoop::FusionStabLoop(int num_channels) {
	
	channels.resize(num_channels);
	
	std::cout << "FusionStabLoop object created with " << channels.size() << " channels\n";
		
}
