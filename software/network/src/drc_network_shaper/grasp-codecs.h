#ifndef GRASPCODECS20130531H
#define GRASPCODECS20130531H

#include "custom-codecs.h"

#include "lcmtypes/drc/desired_grasp_state_t.hpp"

#include "grasp-analogs.pb.h"

class GraspCodec : public CustomChannelCodec
{
  public:
    GraspCodec(const std::string loopback_channel = "");
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);
    
  private:
    goby::acomms::DCCLCodec* dccl_;
        
        
};





#endif
