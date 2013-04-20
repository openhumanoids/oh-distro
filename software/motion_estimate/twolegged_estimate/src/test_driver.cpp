
#include <iostream>


#include "TwoLegOdometry.h"
#include "LegOdometry_LCM_Handler.hpp"
#include <csignal>
#include <exception>

using namespace std;

//TwoLegs::TwoLegOdometry* _leg_odo; // excessive, as this gets tested in _legs_motion_estimate
LegOdometry_Handler* _legs_motion_estimate;

void signalHandler( int signum )
{
    cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here  
    try
    {
    	_legs_motion_estimate->terminate();
    }
    catch (std::exception &e)
    {
    	std::cout << "Exception occured during close out\n";
    }
    // terminate program  

   exit(signum);  

}

int main() {
	// register signal SIGINT and signal handler  
	signal(SIGINT, signalHandler);  
	
	cout << "Test driver main function for the twoleg motion estimate pod" << endl;

	boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
	if(!lcm->good())
	    return 1;
	
	//_leg_odo = new TwoLegs::TwoLegOdometry(); // This is excessive, as the class is also invoked by LegOdometry_Handler() object.
	_legs_motion_estimate = new LegOdometry_Handler(lcm);
	
	
	// Do some stuff with the objects to test them. Preferably here you must call the internal testing functions of the different objects created..
	//_legs_motion_estimate->run(false); // true means it will operate in testing mode and not listen LCM messages
	
	while(0 == lcm->handle());
	
	//delete _leg_odo;
	delete _legs_motion_estimate;
	
	cout << "Everything ends in test_driver for legs_motion_estimate program" << endl; 
	
	return 0;
}


// here somewhere we need to listen to the LCM packets and call back the function in the TwoLegOdometry class as to update its internal states
