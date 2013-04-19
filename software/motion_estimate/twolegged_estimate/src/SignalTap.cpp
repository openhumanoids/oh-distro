

#include "SignalTap.hpp"

Filter::Filter() {
	std::cout << "A new filtering object has been created\n";
	
}

Filter::~Filter() {
	delete(_samples);
	
}

LowPassFilter::LowPassFilter() {
	std::cout << "A new LowPassFilter object has been created\n";
		
}