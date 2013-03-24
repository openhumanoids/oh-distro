
/*
 * Refer to "LegOdometry_LCM_Handler.h" for comments on the purpose of each function and member, comments here in the .cpp relate
 * to more development issues and specifics. The user only need worry about the definitions and descriptions given in the header file,
 * assuming a good software design was done.
 * d fourie
 * 3/24/2013
*/

#include <iostream>
#include <lcm/lcm.h>

#include "LegOdometry_LCM_Handler.h"

using namespace TwoLegs;
using namespace std;


LegOdometry_Handler::LegOdometry_Handler() {
	// Create the object we want to use to estimate the robot's pelvis position
	// In this case its a two legged vehicle and we use TwoLegOdometry class for this task
	_leg_odo = new TwoLegOdometry();

	
	// This class handles the interface between the LCM data broadcasts and the actual state estimation object. The LCM channels and interfaces
	// are set up with setupLCM()
	setupLCM();
	
	return;
}

LegOdometry_Handler::~LegOdometry_Handler() {
	
	
	delete _leg_odo;
	
	lcm_destroy(_lcm);
	
	
	cout << "Everything Destroyed in LegOdometry_Handler::~LegOdometry_Handler()" << endl;
	return;
}


// TODO
void LegOdometry_Handler::setupLCM() {
	
	_lcm = lcm_create(NULL);
	// TODO
	// robot_pose_channel = "ROBOT_SOMETHING_SOMETHING";
	// ?lcm_subscribe_something.h?_t_subscribe(_lcm, robot_pose_channel, TwoLegOdometry::on_?robot_state_update?_aux, this);
	
	return;
}
			

void LegOdometry_Handler::run() {
	
	cout << "LegOdometry_Handler::run() is NOT implemented yet." << endl;
	
	return;
}


/*
 * This has been copied and must be converted to the listen and respond to the correct LCM messages

void LegOdometry_Handler::on_?fnc?(const ?type?* msg) {

	cout << "LCM robot pose event caught in LegOdometry_Handler object << endl;


	//imudata.gyro_(0) = msg->gyro[0];
	// ..
	// ?Propagate?(?,leg_odo);

}
  
void LegOdometry_Handler::on_?robot_state_update?_aux(const lcm_recv_buf_t* rbuf,
                            const char* channel,
                            const ?type?* msg,
                            void* user_data) {
  (static_cast<LegOdometry_Handler *>(user_data))->?fnc?(msg);
}
*/