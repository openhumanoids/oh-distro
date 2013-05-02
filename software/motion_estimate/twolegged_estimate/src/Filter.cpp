
#include <iostream>
#include "Filter.hpp"


Filter::Filter() {
	//std::cout << "A new filtering object has been created\n";
	//samples = Eigen::MatrixXd(10,1);
	//tap_size = size;
	//_samples_buf = new boost::circular_buffer<double>
	
	std::cout << "Filter constuctor did run\n";
	samples_buf.set_capacity(FILTER_TAP_SIZE);
	
	for (int i = 0; i<(FILTER_TAP_SIZE);i++)
	{
		samples_buf.push_back(0);
	}
}

Filter::~Filter() {
	terminate();
}

void Filter::terminate() {
	//delete _samples_buf;
	
	std::cout << "Terminating a Filter object\n";
}



LowPassFilter::LowPassFilter() {
	
	// Initialize the filter memory states
	
	std::cout << "A new LowPassFilter object has been created\n";
}

double LowPassFilter::processSample(double sample) {
	
	// put a new element in the buffer for processing
	samples_buf.push_back(sample);
	// The new sample has been added to the buffer, now we must use the values in the buffer with the coefficients to achieve the IR filtering capabibliy
	
	double accumulator = 0.;
	
	// accumulate the new composite value from all the filter coefficient and history values
	for (int i=0;i<FILTER_TAP_SIZE;i++) {
		accumulator += filter_coeffs[FILTER_TAP_SIZE-i] * samples_buf.at(i);
	}
	
	return accumulator;
}

