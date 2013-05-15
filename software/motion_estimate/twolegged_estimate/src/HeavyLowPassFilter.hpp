/*
 * HeavyLowPassFilter.hpp
 *
 *  Created on: May 14, 2013
 *      Author: drc
 */

#ifndef HEAVYLOWPASSFILTER_HPP_
#define HEAVYLOWPASSFILTER_HPP_

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

//#include "fdacoefs_30Hz_10Hzcutoff.h"
#include "filter_coefficients/fdacoefs_1000_5_30_-40dB.h"

namespace HeavyFiltering {

class HeavyLowPassFilter {
private:
	boost::circular_buffer<double> samples_buf;
	int tap_size;
	bool firstsample;

	void initSamples();

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	HeavyLowPassFilter();
	~HeavyLowPassFilter();

	HeavyLowPassFilter(const HeavyLowPassFilter &original);

	void Init();

	HeavyLowPassFilter& operator=(HeavyLowPassFilter org);

	double processSample(double sample);

	static int getTapSize();

};
}

#endif /* HEAVYLOWPASSFILTER_HPP_ */
