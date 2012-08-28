#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
namespace drc_control
{
class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::nav_goal_t* msg)
        {
            int i;
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  timestamp   = %lld\n", (long long)msg->utime);
	    printf(" robot name        = '%s'\n", msg->robot_name.c_str());
            printf("  position    = (%f, %f, %f)\n",
                    msg->goal_pos.translation.x, msg->goal_pos.translation.y, msg->goal_pos.translation.z);
            printf("  orientation = (%f, %f, %f, %f)\n",
                    msg->goal_pos.rotation.x, msg->goal_pos.rotation.y, 
                    msg->goal_pos.rotation.z, msg->goal_pos.rotation.w);
            printf("\n");
        }
};

}
int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    drc_control::Handler handlerObject;
    lcm.subscribe("NAV_GOAL", &drc_control::Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
 /*    drc::nav_goal_t

	int64_t utime;
	string robot_name;
	position_3d_t goal_pos;
*/
