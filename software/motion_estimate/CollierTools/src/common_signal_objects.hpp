#ifndef COMMON_SIGNAL_OBJECTS_HPP_
#define COMMON_HPP_


#include <Eigen/Dense>


class TrapezoidalIntegrator {
private:
	Eigen::VectorXd old_dx;
	Eigen::VectorXd state1;// double barrel values, since Eigen does not ensure x = f(x) is stable. 
	Eigen::VectorXd state2;// Therefore we will work with x1 = f(x2); x2=x1 -- x1 and x2 should remain consistent
	double half_deltaT;
		
public:
	// Constructor
	TrapezoidalIntegrator(int num_channels, double sample_time);
	
	// Integrate samples
	Eigen::VectorXd integrate(const Eigen::VectorXd &dx);
	
	// Reset the internal state to zero, to restart the integrator
	void reset();
};

#endif /*COMMON_HPP_*/
