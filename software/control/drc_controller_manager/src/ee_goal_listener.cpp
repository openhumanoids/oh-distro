#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
namespace chain_control
{
class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::ee_goal_t* msg)
        {
            int i;
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  timestamp   = %lld\n", (long long)msg->timestamp);
	    printf("  ee name        = '%s'\n", msg->ee_name.c_str());
            printf("  position    = (%f, %f, %f)\n",
                    msg->ee_goal_pos.translation.x, msg->ee_goal_pos.translation.y, msg->ee_goal_pos.translation.z);
            printf("  orientation = (%f, %f, %f, %f)\n",
                    msg->ee_goal_pos.rotation.x, msg->ee_goal_pos.rotation.y, 
                    msg->ee_goal_pos.rotation.z, msg->ee_goal_pos.rotation.w);
            printf("  use_posture_bias     = %d\n", msg->use_posture_bias);
	    printf("  halt_ee_controller     = %d\n", msg->halt_ee_controller);
            printf("\n");
           // printf("  name        = '%s'\n", msg->name.c_str());
           // printf("  enabled     = %d\n", msg->enabled);
        }
};

}
int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    chain_control::Handler handlerObject;
    lcm.subscribe("LWRISTROLL_LINK_GOAL", &chain_control::Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
/* struct ee_goal_t
{
	int64_t timestamp;
	string robot_name;
	string ee_name;
	string root_name;
	position3D_t ee_goal_pos;
	twist_t ee_goal_twist;
	int32_t num_chain_joints;
	boolean use_posture_bias;
	double joint_posture_bias [num_chain_joints];
	string chain_joint_names [num_chain_joints];
        boolean halt_ee_controller;
}
my_goal.ee_goal_pos.translation.x = 0.151708;
    my_goal.ee_goal_pos.translation.y = 0.241059;
    my_goal.ee_goal_pos.translation.z = 0.353141;

    my_goal.ee_goal_pos.rotation.x = 0;
    my_goal.ee_goal_pos.rotation.y = -0.258691;
    my_goal.ee_goal_pos.rotation.z = 0;
    my_goal.ee_goal_pos.rotation.w = 0.96596;
    
    
    my_goal.ee_goal_twist.linear_velocity.x = 0.0;
    my_goal.ee_goal_twist.linear_velocity.y = 0.0;
    my_goal.ee_goal_twist.linear_velocity.z = 0.0;
    my_goal.ee_goal_twist.angular_velocity.x = 0.0;
    my_goal.ee_goal_twist.angular_velocity.y = 0.0;
    my_goal.ee_goal_twist.angular_velocity.z = 0.0;

link_name : LWristYaw_link
translation  : 
	 .x  : 0.151708
	 .y  : 0.241059
	 .z  : 0.353141
quaternion
	 .x  : 0
	 .y  : -0.258691
	 .z  : 0
	 .w  : 0.96596*/