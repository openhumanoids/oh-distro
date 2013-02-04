#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

class Handler
{
public:
	~Handler() {}
	lcm::LCM lcm;

	void listenToStateAndPublishPlanMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::robot_state_t * msg)
	{
	  drc::robot_plan_t plan_msg;
	 plan_msg.utime =  msg->utime;
	 plan_msg.robot_name =  msg->robot_name;
	 plan_msg.num_states = 50;
	 drc::robot_state_t state_msg;
	 drc::position_3d_t body_origin;
	 body_origin = msg->origin_position; 
	 for (uint i=0; i<plan_msg.num_states; i++)
	 {
	   state_msg = *msg;
	   body_origin.translation.x = body_origin.translation.x + 0.5;
	   state_msg.origin_position =  body_origin; 
	  // std::cout<< state_msg.joint_name.size() <<std::endl;
	  for(std::vector<std::string>::size_type j = 0; j != state_msg.joint_name.size(); j++) 
    {
      //std::cout<< "ok" <<std::endl;
     if(state_msg.joint_name[j]=="theta")
     {
       state_msg.joint_position[j] = i*0.1;
     }
    }
	   
	   
	   plan_msg.plan.push_back(state_msg);
	 }
	
		lcm.publish("CANDIDATE_ROBOT_PLAN", &plan_msg);
	}
};

int main (int argc, char ** argv)
{
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	Handler handlerObject;
	lcm.subscribe("EST_ROBOT_STATE", &Handler::listenToStateAndPublishPlanMessage,
		&handlerObject);

	while(0 == lcm.handle());

	return 0;
}



//struct robot_plan_t
//{
//    int64_t utime;
//    string robot_name;    
//    int32_t num_states;
//    robot_state_t plan[num_states]; //each individual state is also timed.
//}



