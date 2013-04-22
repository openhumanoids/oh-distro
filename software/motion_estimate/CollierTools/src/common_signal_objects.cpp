
#include <iostream>
#include "common_signal_objects.hpp"

TrapezoidalIntegrator::TrapezoidalIntegrator(int num_channels, double sample_time) {
	old_dx.resize(num_channels);
	state1.resize(num_channels);
	state2.resize(num_channels);
	
	reset();
	
	half_deltaT = 0.5*sample_time;
}

Eigen::VectorXd TrapezoidalIntegrator::integrate(const Eigen::VectorXd &dx) {
	
	state2 = state1 + half_deltaT * (old_dx + dx);
	state1 = state2;
	old_dx = dx;
	
	return state1;
}

void TrapezoidalIntegrator::reset() {
	old_dx.setZero();
	state1.setZero();
	state2.setZero();
	
}
