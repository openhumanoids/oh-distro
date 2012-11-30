#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

class Handler
{
public:
	~Handler() {}
	lcm::LCM lcm;

	void handleMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const drc::robot_state_t * msg)
	{
	  //std::cout << msg->utime << std::endl;
		lcm.publish("EST_ROBOT_STATE", msg);
		
	}
};

int main (int argc, char ** argv)
{
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	Handler handlerObject;
	lcm.subscribe("TRUE_ROBOT_STATE", &Handler::handleMessage,
		&handlerObject);

	while(0 == lcm.handle());

	return 0;
}


