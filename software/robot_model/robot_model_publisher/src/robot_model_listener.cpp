#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::robot_urdf_t * msg)
        {
    	 std::cout << "Received urdf string on channel " << chan << std::endl;
     	 std::cout << "  timestamp   = " << msg->timestamp << std::endl;
    	 std::cout << msg->urdf_xml_string << std::endl; 
        }
};


int
main(int argc, char ** argv)
{

    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("ROBOT_MODEL", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}

