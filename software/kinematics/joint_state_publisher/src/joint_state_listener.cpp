// Test program for subcription of joint angles.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/sensor_msgs/joint_state_t.hpp"

namespace joint_state_listener {
class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const sensor_msgs::joint_state_t* msg)
        {

		std::cout << "Received joint_state_t message on channel " << chan << std::endl;	     	 
	    	for(std::vector<std::string>::size_type i = 0; i != msg->joint_name.size(); i++) 
	  	{
		  std::cout<< "  timestamp   = " << msg->timestamp << "  joint name  = " <<  msg->joint_name[i] << "  position    = " <<  msg->position[i] <<std::endl;
		}

        }
};

}

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    joint_state_listener::Handler handlerObject;
    lcm.subscribe("JOINT_STATES", &joint_state_listener::Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
