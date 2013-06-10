#include "manip-plan-codecs.h"

ManipPlanCodec::ManipPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
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
    if(!to_minimal_robot_plan(lcm_object, dccl_plan))
        return false;    

    if(!to_minimal_robot_plan_control_type_new(lcm_object, dccl_plan))
        return false;
    
    dccl_plan.set_aff_num_states(0);
    
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
    
    if(!from_minimal_robot_plan(lcm_object, dccl_plan))
        return false;

    if(!from_minimal_robot_plan_control_type_new(lcm_object, dccl_plan))
        return false;
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    return true;
          
}      


ManipMapCodec::ManipMapCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{
    dccl_->validate<drc::MinimalRobotPlan>();
}

bool ManipMapCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::aff_indexed_robot_plan_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalRobotPlan dccl_plan;
    if(!to_minimal_robot_plan(lcm_object, dccl_plan))
        return false;  

    if(!to_minimal_robot_plan_control_type_old(lcm_object, dccl_plan))
        return false;

    dccl_plan.set_aff_num_states(lcm_object.num_states);    

    for(int i = 0, n = lcm_object.aff_index.size(); i < n; ++i)
    {
        const drc::affordance_index_t& lcm_aff_index = lcm_object.aff_index[i];
        drc::AffordanceIndex* dccl_aff_index = dccl_plan.add_aff_index();        
        
        if(i == 0)
            dccl_plan.set_aff_uid(lcm_aff_index.aff_uid);

        for(int j = 0, o = lcm_aff_index.num_ees; j < o; ++j)
        {
            drc::AffordanceIndex::EEName ee_name;
            if(!drc::AffordanceIndex::EEName_Parse(lcm_aff_index.ee_name[j], &ee_name))
            {
                glog.is(WARN) && glog << "Warning, could not parse ee_name: " << lcm_aff_index.ee_name[j] <<". Make sure it is defined in robot-plan-analogs.proto" << std::endl;
                return false;
            }
            dccl_aff_index->add_ee_name(ee_name);

            drc::AffordanceIndex::DOFName dof_name;
            if(!drc::AffordanceIndex::DOFName_Parse(lcm_aff_index.dof_name[j], &dof_name))
            {
                glog.is(WARN) && glog << "Warning, could not parse dof_name: " << lcm_aff_index.dof_name[j] <<". Make sure it is defined in robot-plan-analogs.proto" << std::endl;
                return false;
            }
            dccl_aff_index->add_dof_name(dof_name);
            dccl_aff_index->add_dof_value(lcm_aff_index.dof_value[j]);
        }        
    }
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_plan);
    transmit_data->resize(encoded.size());
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}

      
bool ManipMapCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalRobotPlan dccl_plan;

    dccl_->decode(encoded, &dccl_plan);
    
    drc::aff_indexed_robot_plan_t lcm_object;
    
    if(!from_minimal_robot_plan(lcm_object, dccl_plan))
        return false;
    
    if(!from_minimal_robot_plan_control_type_old(lcm_object, dccl_plan))
        return false;
    
    for(int i = 0, n = dccl_plan.aff_num_states(); i < n; ++i)
    {
        drc::affordance_index_t lcm_aff_index;
        const drc::AffordanceIndex& dccl_aff_index = dccl_plan.aff_index(i);

        lcm_aff_index.utime = 0;
        lcm_aff_index.aff_type = "car";
        lcm_aff_index.aff_uid = dccl_plan.aff_uid();
        lcm_aff_index.num_ees = dccl_aff_index.ee_name_size();
        
        for(int j = 0, o = lcm_aff_index.num_ees; j < o; ++j)
        {
            lcm_aff_index.ee_name.push_back(
                drc::AffordanceIndex::EEName_Name(dccl_aff_index.ee_name(j)));
            lcm_aff_index.dof_name.push_back(
                drc::AffordanceIndex::DOFName_Name(dccl_aff_index.dof_name(j)));
            lcm_aff_index.dof_value.push_back(dccl_aff_index.dof_value(j));
        }
        lcm_object.aff_index.push_back(lcm_aff_index);
    }    
    
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    return true;
          
}      
