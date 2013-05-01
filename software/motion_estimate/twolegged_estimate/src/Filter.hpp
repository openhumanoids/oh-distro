#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>


#include "fdacoefs_30Hz_10Hzcutoff.h"

class Filter {
protected:
	boost::circular_buffer<double> samples_buf;
	int tap_size;
	
	//Eigen::VectorXd samples;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Filter();
	~Filter();
	
	//virtual double processSample(double sample);
	
	void terminate();
};

class LowPassFilter : public Filter { 
private:
	
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	LowPassFilter();
	
	double processSample(double sample);
	
};


#endif /*FILTER_HPP_*/
