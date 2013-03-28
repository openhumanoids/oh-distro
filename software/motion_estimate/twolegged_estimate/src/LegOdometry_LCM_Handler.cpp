
/*
 * Refer to "LegOdometry_LCM_Handler.h" for comments on the purpose of each function and member, comments here in the .cpp relate
 * to more development issues and specifics. The user only need worry about the definitions and descriptions given in the header file,
 * assuming a good software design was done.
 * d fourie
 * 3/24/2013
*/

#include <iostream>
#include <exception>
//#include <stdio.h>
//#include <inttypes.h>


//#include "lcmtypes/drc_lcmtypes.hpp"
#include "LegOdometry_LCM_Handler.hpp"

using namespace TwoLegs;
using namespace std;


LegOdometry_Handler::LegOdometry_Handler() : _finish(false) {
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
	
//	lcm_destroy(_lcm);
	
	
	cout << "Everything Destroyed in LegOdometry_Handler::~LegOdometry_Handler()" << endl;
	return;
}

void LegOdometry_Handler::setupLCM() {
	
//	_lcm = lcm_create(NULL);
	// TODO
	// robot_pose_channel = "TRUE_ROBOT_STATE";
	// drc_robot_state_t_subscribe(_lcm, robot_pose_channel, TwoLegOdometry::on_robot_state_aux, this);
	
	if(!lcm_.good())
	  return;
	
	lcm_.subscribe("TRUE_ROBOT_STATE",&LegOdometry_Handler::robot_state_handler,this); 
	
	return;
}
			

void LegOdometry_Handler::run(bool testingmode) {
	
	// TODO
	cout << "LegOdometry_Handler::run(bool) is NOT finished yet." << endl;
	
	if (testingmode)
	{
		cout << "LegOdometry_Handler::run(bool) in tesing mode." << endl;
		
		for (int i = 0 ; i<10 ; i++)
		{
			_leg_odo->CalculateBodyStates_Testing(i);
			
		}
		
		
	}
	else
	{
		cout << "Attempting to start lcm_handle loop..." << endl;
		
		try
		{
			// This is the highest referrence point for the 
			while(0 == lcm_.handle());
		    
		}
		catch (exception& e)
		{
			cout << "LegOdometry_Handler::run() - Oops something went wrong when we tried to listen and respond to a new lcm message:" << endl;
			cout << e.what() << endl;
			
		}
	}
	
	return;
}


//void LegOdometry_Handler::robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg) {

void LegOdometry_Handler::robot_state_handler(	const lcm::ReceiveBuffer* rbuf, 
												const std::string& channel, 
												const  drc::robot_state_t* msg) {
	
	std::cout << msg->utime << ", ";
	std::cout << msg->contacts.contact_force[0].z << ", ";
	std::cout << msg->contacts.contact_force[1].z << ", ";
	
	//pass left and right leg forces and torques to TwoLegOdometry
	// TODO temporary testing interface
	
	_leg_odo->DetectFootTransistion(msg->utime, msg->contacts.contact_force[1].z , msg->contacts.contact_force[0].z);
	std::cout << _leg_odo->primary_foot() << std:: endl;
	
	
	
	
}


/*
 * This has been copied and must be converted to the listen and respond to the correct LCM messages
 

void LegOdometry_Handler::on_robot_state(const drc_robot_state_t* msg) {

	cout << "LCM robot pose event caught in LegOdometry_Handler object" << endl;


	//data.gyro_(0) = msg->gyro[0];
	// ..
	// ?Propagate?(?,leg_odo);

}

void LegOdometry_Handler::on_robot_state_aux(const lcm_recv_buf_t* rbuf,
                            const char* channel,
                            const drc_robot_state_t* msg,
                            void* user_data) {
  (static_cast<LegOdometry_Handler *>(user_data))->on_robot_state(msg);
}
*/
