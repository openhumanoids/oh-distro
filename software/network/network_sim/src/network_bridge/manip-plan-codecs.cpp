#include "manip-plan-codecs.h"
#include "goby/acomms/dccl/dccl_field_codec_arithmetic.h"

ManipPlanCodec::ManipPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{
    using goby::glog;
    using namespace goby::common::logger;

    const float MAX = 6.284;
    const float MIN = -MAX;    

    std::string model_file_path = getenv ("HOME");
    model_file_path += "/drc/software/network/network_sim/src/network_bridge/joint_pos_frequencies.csv";
    std::ifstream model_file(model_file_path.c_str());
    if(!model_file.is_open())
        glog.is(DIE) && glog << "Could not open " << model_file_path << " for reading." << std::endl;

    std::string line;
    std::getline(model_file, line);

    std::vector<std::string> xs;
    boost::split(xs, line, boost::is_any_of(","));

    std::vector<double> x;
    for(int i = 0; i < xs.size(); ++i)
        x.push_back(goby::util::as<double>(xs[i]));
    
    for(int j = 0; j < 28; ++j)
    {
        goby::acomms::protobuf::ArithmeticModel model;

        glog.is(VERBOSE) && glog << "Making joint_pos_" << j << " model" << std::endl;

        std::getline(model_file, line);
        std::vector<std::string> ys;
        boost::split(ys, line, boost::is_any_of(","));
        
        std::vector<double> y;
        for(int i = 0; i < xs.size(); ++i)
            y.push_back(goby::util::as<double>(ys[i]));
        
        int total_frequency = 0;        
        for(int i = 0, n = x.size(); i <= n; ++i)
        {
            if(i != n)
            {
                int freq = y[i];
                model.add_value_bound(x[i]);
                model.add_frequency(freq);
                total_frequency += freq;
            }
            else
            {
                model.add_value_bound(x[i-1] + 1/pow(10.0, JOINT_POS_PRECISION));
            }            
        }
        
        const double eof_fraction = 0.1; // 10% of total frequency
        model.set_eof_frequency(total_frequency*eof_fraction/(1-eof_fraction)); 
        model.set_out_of_range_frequency(0);
        
        
        model.set_name("joint_pos_" + goby::util::as<std::string>(j));
        glog.is(VERBOSE) && glog << "Setting joint_pos_" << j << " model" << std::endl;
        goby::acomms::ModelManager::set_model(model);        
//        std::cout << pb_to_short_string(model) << std::endl;
    
    }
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

    std::cout << encoded.size() << "," << std::flush;
    
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
//    dccl_->validate<drc::MinimalRobotPlan>();
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
