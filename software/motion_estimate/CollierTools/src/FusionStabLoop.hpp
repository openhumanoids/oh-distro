#ifndef FUSIONSTABLOOP_HPP_
#define FUSIONSTABLOOP_HPP_

#include <Eigen/Dense>
#include "common_signal_objects.hpp"

#define FEEDBACK_GAIN_PRESET 0.003

class FusionStabLoop {
private:
	Eigen::VectorXd channels;
	double gain;
	
	TrapezoidalIntegrator* _feedback_integrator;
	

public:
	FusionStabLoop(int num_channels);
	~FusionStabLoop();
	
	
};

#endif /*FUSIONSTABLOOP_HPP_*/
