// Test program for subcription of joint angles.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/joint_angles_t.hpp"

namespace meas_joint_angles_listener {
class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::joint_angles_t* msg)
        {

		std::cout << "Received joint_angles_t message for robot "<<msg->robot_name<<" on channel " << chan << std::endl;	     	 
	    	for(std::vector<std::string>::size_type i = 0; i != msg->joint_name.size(); i++) 
	  	{
		  std::cout<< "  timestamp   = " << msg->utime << "  joint name  = " <<  msg->joint_name[i] << "  position    = " <<  msg->joint_position[i] <<std::endl;
		}

        }
};

}

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    meas_joint_angles_listener::Handler handlerObject;
    lcm.subscribe("MEAS_JOINT_ANGLES", &meas_joint_angles_listener::Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
