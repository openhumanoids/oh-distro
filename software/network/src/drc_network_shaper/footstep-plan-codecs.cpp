#include "footstep-plan-codecs.h"
#include "robot-state-codecs.h"
#include "dccl/arithmetic/field_codec_arithmetic.h"
#include "lzma-string.h"

FootStepPlanCodec::FootStepPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{


    using goby::glog;
    using namespace goby::common::logger;
    dccl_->validate<drc::MinimalFootStepPlan>();
}

bool FootStepPlanCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::walking_plan_request_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalFootStepPlan dccl_plan;
    
    RobotStateCodec::to_minimal_state(lcm_object.initial_state, dccl_plan.mutable_initial_state());
    dccl_plan.set_utime(lcm_object.utime);
    dccl_plan.set_fix_right_hand(lcm_object.fix_right_hand);
    dccl_plan.set_fix_left_hand(lcm_object.fix_left_hand);
    dccl_plan.set_use_new_nominal_state(lcm_object.use_new_nominal_state);
    RobotStateCodec::to_minimal_state(lcm_object.new_nominal_state, dccl_plan.mutable_new_nominal_state());
    
    glog.is(VERBOSE) && glog << "MinimalFootStepPlan: " << pb_to_short_string(dccl_plan) << std::endl;
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_plan);


    // append LZMA compressed "footstep_plan"
    std::vector<unsigned char> footstep_plan_data(lcm_object.footstep_plan.getEncodedSize());
    lcm_object.footstep_plan.encode(&footstep_plan_data[0], 0, footstep_plan_data.size());
    encoded += CompressWithLzma(std::string(footstep_plan_data.begin(), footstep_plan_data.end()),
                                6);

    
    transmit_data->resize(encoded.size());
    std::cout << encoded.size() << "," << std::flush;
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}


bool FootStepPlanCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalFootStepPlan dccl_plan;

    dccl_->decode(encoded, &dccl_plan);

    glog.is(VERBOSE) && glog << "Decoded MinimalFootStepPlan: " << pb_to_short_string(dccl_plan) << std::endl;
    
    drc::walking_plan_request_t lcm_object;
    lcm_object.utime = dccl_plan.utime();    

    if(!RobotStateCodec::from_minimal_state(&lcm_object.initial_state, dccl_plan.initial_state()))
        return false;
    
    lcm_object.fix_right_hand = dccl_plan.fix_right_hand();
    lcm_object.fix_left_hand = dccl_plan.fix_left_hand();
    lcm_object.use_new_nominal_state = dccl_plan.use_new_nominal_state();

    if(!RobotStateCodec::from_minimal_state(&lcm_object.new_nominal_state, dccl_plan.new_nominal_state()))
        return false;
    
    std::string decoded_footstep_plan = DecompressWithLzma(std::string(transmit_data.begin() + dccl_->size(dccl_plan), transmit_data.end()));
    if(lcm_object.footstep_plan.decode(&decoded_footstep_plan[0], 0, decoded_footstep_plan.size()) == -1)
        return false;
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());
    
    return true;
          
}
