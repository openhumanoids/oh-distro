#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

//#include "fdacoefs_30Hz_10Hzcutoff.h"
#include "filter_coefficients/fdacoefs_1000_10_100_-20dB.h"

/*
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
*/

class LowPassFilter {
private:
	boost::circular_buffer<double> samples_buf;
	int tap_size;
	
	void initSamples();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	LowPassFilter();
	~LowPassFilter();
	
	LowPassFilter(const LowPassFilter &original);

	void Init();

	LowPassFilter& operator=(LowPassFilter org);

	double processSample(double sample);
	
	static int getTapSize();

};


#endif /*FILTER_HPP_*/
