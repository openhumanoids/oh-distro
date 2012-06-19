// Test program for subcription of joint angles.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/sensor_msgs/transform_stamped_t.hpp"
#include "lcmtypes/sensor_msgs/tf_t.hpp"

namespace robot_state_listener {
class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const sensor_msgs::tf_t* msg)
        {
		 std::cout << "Received joint_state_t message on channel " << chan << std::endl;	     	 
		for(std::vector<sensor_msgs::transform_stamped_t>::size_type i = 0; i != msg->num_joints; i++) 
		{
		  sensor_msgs::transform_stamped_t tf_transform =  msg->tf[i];
		  std::cout << "timestamp  : " << tf_transform.timestamp << std::endl;
		  std::cout << "frame_id_  : " << tf_transform.frame_id_ << std::endl;
		  std::cout << "child_frame_id_  : " << tf_transform.child_frame_id_ << std::endl;
		  std::cout << "translation  : " << std::endl;
		  std::cout << "\t .x  : " << tf_transform.translation.x << std::endl;
		  std::cout << "\t .y  : " << tf_transform.translation.y << std::endl;
		  std::cout << "\t .z  : " << tf_transform.translation.z << std::endl;
		  std::cout << "quaternion" << std::endl;
		  std::cout << "\t .x  : " << tf_transform.rotation.x << std::endl;
		  std::cout << "\t .y  : " << tf_transform.rotation.y << std::endl;
		  std::cout << "\t .z  : " << tf_transform.rotation.z << std::endl;
		  std::cout << "\t .w  : " << tf_transform.rotation.w << std::endl;
		    
		}

        }
};

}

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    robot_state_listener::Handler handlerObject;
    lcm.subscribe("JOINT_TRANSFORMS", &robot_state_listener::Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}