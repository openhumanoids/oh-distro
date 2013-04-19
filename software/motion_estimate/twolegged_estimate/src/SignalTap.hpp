#ifndef SIGNALTAP_HPP_
#define SIGNALTAP_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// objects and functions for generic operations on data

class Filter {
private:

	Eigen::MatrixXd *_samples;
public:
	Filter();
	~Filter();
	
};

class LowPassFilter : Filter { 
private:
	
	
	
public:
	LowPassFilter();
	
};

#endif /*SIGNALTAP_HPP_*/
