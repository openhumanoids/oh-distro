#include "manip-plan-codecs.h"
#include "robot-state-codecs.h"

ManipPlanCodec::ManipPlanCodec()
    : dccl_(goby::acomms::DCCLCodec::get())
{
    dccl_->validate<drc::MinimalRobotPlan>();
}

bool ManipPlanCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::robot_plan_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalRobotPlan dccl_plan;

    // fill in dccl_plan

    
    glog.is(VERBOSE) && glog << "MinimalRobotPlan: " << dccl_plan.ShortDebugString() << std::endl;
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_plan);
    transmit_data->resize(encoded.size());
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}

      
bool ManipPlanCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalRobotPlan dccl_plan;

    dccl_->decode(encoded, &dccl_plan);
    
    drc::robot_plan_t lcm_object;

    // fill in lcm_object
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    return true;
          
}      
