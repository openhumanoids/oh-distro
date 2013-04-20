#ifndef SIGNALTAP_HPP_
#define SIGNALTAP_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>

// objects and functions for generic operations on data

class Filter {
private:

	Eigen::VectorXd samples;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Filter();
	
};

class LowPassFilter : Filter { 
private:
	
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	LowPassFilter();
	
};

class DataFileLogger {
private:
	std::ofstream fs;
	
	
public:
	DataFileLogger();
	
	void Open(std::string filename);
	void Close();
	
	void log(std::string data);
};


#endif /*SIGNALTAP_HPP_*/
