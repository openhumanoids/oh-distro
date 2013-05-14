#include "robot-state-codecs.h"

RobotStateCodec::RobotStateCodec()
    : dccl_(goby::acomms::DCCLCodec::get())
{
}

bool RobotStateCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    drc::robot_state_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    // fill in analog DCCL message, encode, and send
        
    return true;
}

      
bool RobotStateCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    drc::robot_state_t lcm_object;

    // use DCCL message to reconstruct lcm message

    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());
          
    return true;
          
}      
