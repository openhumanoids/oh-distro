
#include "manip-plan-codecs.h"
#include "dccl/arithmetic/field_codec_arithmetic.h"
#include "lzma-string.h"

//
// ManipPlanCodec
//

ManipPlanCodec::ManipPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{
    using goby::glog;
    using namespace goby::common::logger;

    if(true)
    {
        const float MAX = 6.284;
        const float MIN = -MAX;    

        std::string model_file_path = getenv ("DRC_BASE");
        model_file_path += "/software/network/src/drc_network_shaper/2015_joint_pos_plan_frequencies.csv";
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

        int j = 0;
        while(std::getline(model_file, line))
        {
            dccl::arith::protobuf::ArithmeticModel model;

            glog.is(VERBOSE) && glog << "Making joint_pos_" << j << " model" << std::endl;

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
            dccl::arith::ModelManager::set_model(model);        
//        std::cout << pb_to_short_string(model) << std::endl;
            ++j;
        }
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
    

    const int GOAL_MAX = drc::MinimalRobotStateDiff::descriptor()->FindFieldByName("utime_diff")->options().GetExtension(dccl::field).max_repeat();

    if(dccl_plan.goal_diff().utime_diff_size() > GOAL_MAX)
    {
        glog.is(WARN) && glog << "Plan exceeds maximum number of goals of " << GOAL_MAX << ", discarding."  << std::endl;
        return false;
    }    
    
    
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


//
// SupportsPlanCodec
//


SupportsPlanCodec::SupportsPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{
    using goby::glog;
    using namespace goby::common::logger;

}


bool SupportsPlanCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::robot_plan_with_supports_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalRobotPlan dccl_plan;
    if(!to_minimal_robot_plan(lcm_object.plan, dccl_plan))
        return false;    

    if(!to_minimal_robot_plan_control_type_new(lcm_object.plan, dccl_plan))
        return false;
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_plan);

    // append LZMA compressed "support_sequence"
    std::vector<unsigned char> support_sequence_data(lcm_object.support_sequence.getEncodedSize());
    lcm_object.support_sequence.encode(&support_sequence_data[0], 0, support_sequence_data.size());
    encoded += CompressWithLzma(std::string(support_sequence_data.begin(), support_sequence_data.end()), 6);

    transmit_data->resize(encoded.size());

    std::cout << encoded.size() << "," << std::flush;
    
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}

      
bool SupportsPlanCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalRobotPlan dccl_plan;

    dccl_->decode(encoded, &dccl_plan);
    
    drc::robot_plan_with_supports_t lcm_object;
    
    if(!from_minimal_robot_plan(lcm_object.plan, dccl_plan))
        return false;

    if(!from_minimal_robot_plan_control_type_new(lcm_object.plan, dccl_plan))
        return false;

    lcm_object.utime = lcm_object.plan.utime;


    std::string decoded_support_sequence = DecompressWithLzma(std::string(transmit_data.begin() + dccl_->size(dccl_plan), transmit_data.end()));
    if(lcm_object.support_sequence.decode(&decoded_support_sequence[0], 0, decoded_support_sequence.size()) == -1)
        return false;

    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    return true;
          
}
