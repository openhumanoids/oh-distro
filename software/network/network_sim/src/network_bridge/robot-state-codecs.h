#ifndef ROBOTSTATECODECS20130514H
#define ROBOTSTATECODECS20130514H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_state_t.hpp"

#include "robot-state-analogs.pb.h"

class RobotStateCodec : public CustomChannelCodec
{
  public:
    RobotStateCodec(const std::string loopback_channel = "");
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);

    static std::map<std::string, int> joint_names_to_order_;
    static std::vector<std::string> joint_names_;
    
  private:
    goby::acomms::DCCLCodec* dccl_;
        
        
};





#endif
