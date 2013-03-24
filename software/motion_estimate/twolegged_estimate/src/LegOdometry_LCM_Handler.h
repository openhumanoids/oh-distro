/*
 * This class is the interface between the LCM communications system and the computations relating to estimation of the robots pelvis position,
 * based on leg motion and joint encoder angles of the vehicle. This software depends on the LCM events to operate and compute the new states. 
 * The computed states are then retransmitted on LCM for whoever wants to use the pelvis and head positions and orientations estimates.
 * Navigation of the pelvis and head is done relative to a reset position, from where estimates are relative to that reset position and orientation 
 * reference frame.
 * 
 * d fourie
 * 3/24/2013
*/


#ifndef LEGODOMETRY_LCM_HANDLER_H_
#define LEGODOMETRY_LCM_HANDLER_H_

#include <lcm/lcm.h>

#include "TwoLegOdometry.h"
//#include <lcmtypes/?ROBOTSOMETHING?.h>

class LegOdometry_Handler {
private:
	TwoLegs::TwoLegOdometry *_leg_odo;
	
	
	// LCM stuff
	const char* robot_pose_channel;
	lcm_t* _lcm;
	//void on_?fnc?(const ?type?* msg);
	//void LegOdometry_Handler::on_?robot_state_update?_aux(const lcm_recv_buf_t* rbuf,
	//                            const char* channel,
	//                            const ?type?* msg,
	//                            void* user_data)
	
	// Connect to the correct LCM messages and start to propagate that data into the odometry object for state estimation
	void setupLCM();
				
	
public:
	LegOdometry_Handler();
	~LegOdometry_Handler();
	
	// Run the estimator, assuming the LCM connections have been set up correctly
	void run();
	
};


#endif /*LEGODOMETRY_LCM_HANDLER_H_*/
