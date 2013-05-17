#ifndef MANIPPLANCODECS20130517H
#define MANIPPLANCODECS20130517H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_plan_t.hpp"

#include "robot-plan-analogs.pb.h"

class ManipPlanCodec : public CustomChannelCodec
{
  public:
    ManipPlanCodec();
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);
    
  private:
    goby::acomms::DCCLCodec* dccl_;
        
        
};





#endif
