
#include <iostream>
#include "Filter.hpp"

/*
Filter::Filter() {
	//std::cout << "A new filtering object has been created\n";
	//samples = Eigen::MatrixXd(10,1);
	//tap_size = size;
	//_samples_buf = new boost::circular_buffer<double>
	
	std::cout << "Filter constructor did run\n";
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
*/


LowPassFilter::LowPassFilter() {
	
	// Initialize the filter memory states
	Init();


	//std::cout << "A new LowPassFilter object has been created\n";
}

LowPassFilter::LowPassFilter(const LowPassFilter &original) {
	Init(); // we can do this because we know exactly what the filter is -- it is defined in the coeff.h
}

void LowPassFilter::Init() {
	firstsample = true;
	samples_buf.set_capacity(FILTER_TAP_SIZE);

	for (int i = 0; i<(FILTER_TAP_SIZE);i++)
	{
		samples_buf.push_back(0);
	}
}


LowPassFilter& LowPassFilter::operator=(LowPassFilter org) {
	// Need this operator, to ensure that when the push_back for drc std::vector of filter is done in a local scope, objects will be maintained that can be used safely
	// This memory is cleared by the std::vector destructor and this destructor when the std::vector object moves out of scope

	// We can do this because all the filters created here are going to be the same, as defined by the coeff.h file
	Init();

	return *this;
}

double LowPassFilter::processSample(double sample) {
	if (firstsample) {
		firstsample = false;
		for (int i=0;i<FILTER_TAP_SIZE;i++) {
			samples_buf.push_back(sample); // we force the first sample into all state memory as an initial guess of there the filter should be initialized
		}
	}
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

LowPassFilter::~LowPassFilter() {
	samples_buf.clear();
	//std::cout << "Closing out LowPassFilter object\n";
}

int LowPassFilter::getTapSize() {
	return FILTER_TAP_SIZE;
}
