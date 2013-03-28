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


#ifndef LEGODOMETRY_LCM_HANDLER_HPP_
#define LEGODOMETRY_LCM_HANDLER_HPP_


#include <string>
#include <map>

#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include <model-client/model-client.hpp>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/std/vector.hpp>

//#include <lcmtypes/bot_core.hpp>

#include <lcm/lcm.h>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "TwoLegOdometry.h"

//#include "urdf/model.h"

class LegOdometry_Handler {
private:
	TwoLegs::TwoLegOdometry *_leg_odo;
	//boost::shared_ptr<lcm::LCM> lcm_;
	lcm::LCM lcm_;
	boost::shared_ptr<ModelClient> model_;
	
	
	// LCM stuff
	const char* robot_pose_channel;
	bool _finish;
	
	// Connect to the correct LCM messages and start to propagate that data into the odometry object for state estimation
	void setupLCM();
	
	//void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
	void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);	
	
public:
	LegOdometry_Handler();
	~LegOdometry_Handler();
	
	void finish() { _finish = true; }
	
	// Run the estimator, assuming the LCM connections have been set up correctly
	void run(bool testingmode);
	
};


#endif /*LEGODOMETRY_LCM_HANDLER_H_*/
