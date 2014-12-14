// Test program for subcription of joint angles and subsequent conversion into ROS style tf transforms.

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/tf_t.hpp"

namespace robot_tf_listener {
class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const drc::tf_t* msg)
        {
		 std::cout << "Received tf_t message on channel " << chan << std::endl;
		 std::cout << "timestamp  : " << msg->utime << std::endl;
		 std::cout << "robot  : " << msg->robot_name << std::endl;		 
		for(std::vector<drc::transform_stamped_t>::size_type i = 0; i != msg->num_joints; i++) 
		{
		  drc::transform_stamped_t tf_transform =  msg->tf[i];
		  
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

    robot_tf_listener::Handler handlerObject;
    lcm.subscribe("JOINT_TRANSFORMS", &robot_tf_listener::Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
