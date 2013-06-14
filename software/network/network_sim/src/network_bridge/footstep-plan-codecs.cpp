#include "footstep-plan-codecs.h"
#include "robot-state-codecs.h"
#include "goby/acomms/dccl/dccl_field_codec_arithmetic.h"

FootStepPlanCodec::FootStepPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{

    goby_dccl_load(dccl_);    

    using goby::glog;
    using namespace goby::common::logger;

//    const float RP_MAX = 2.794;
//    const float RP_MIN = -MAX;    
    const int RP_PRECISION = drc::RotationRPYDiff::descriptor()->FindFieldByName("droll")->options().GetExtension(dccl::field).precision();
    
    goby::acomms::protobuf::ArithmeticModel rp_model;
    
    glog.is(VERBOSE) && glog << "Making rollpitch model" << std::endl;
        
    std::string rp_model_file_path = getenv ("HOME");
    rp_model_file_path += "/drc/software/network/network_sim/src/network_bridge/rollpitch_frequencies.csv";
    std::ifstream rp_model_file(rp_model_file_path.c_str());
    if(!rp_model_file.is_open())
        glog.is(DIE) && glog << "Could not open " << rp_model_file_path << " for reading." << std::endl;

    std::string line;
    std::getline(rp_model_file, line);

    std::vector<std::string> xs;
    boost::split(xs, line, boost::is_any_of(","));

    std::vector<double> x;
    for(int i = 0; i < xs.size(); ++i)
        x.push_back(goby::util::as<double>(xs[i]));
    
    
    std::getline(rp_model_file, line);
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
            rp_model.add_value_bound(x[i]);
            rp_model.add_frequency(freq);
            total_frequency += freq;
            }
        else
        {
            rp_model.add_value_bound(x[i-1] + 1/pow(10.0, RP_PRECISION));
        }            
    }
    
    const double eof_fraction = 0.1; // 10% of total frequency
    rp_model.set_eof_frequency(total_frequency*eof_fraction/(1-eof_fraction)); 
    rp_model.set_out_of_range_frequency(0);    
    
    rp_model.set_name("rollpitch");
    glog.is(VERBOSE) && glog << "Setting rollpitch model" << std::endl;
    goby::acomms::ModelManager::set_model(rp_model);        
//        std::cout << pb_to_short_string(rp_model) << std::endl;

    dccl_->validate<drc::MinimalFootStepPlan>();
}

bool FootStepPlanCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::footstep_plan_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalFootStepPlan dccl_plan;
    
    dccl_plan.set_utime(lcm_object.utime);
    dccl_plan.set_is_new_plan(lcm_object.is_new_plan);
    dccl_plan.set_opts_ignore_terrain(lcm_object.footstep_opts.ignore_terrain);
    dccl_plan.set_opts_mu(lcm_object.footstep_opts.mu);

    if(lcm_object.num_steps > 0)
    {
        drc::MinimalFootStepGoal* first_goal = dccl_plan.mutable_first_goal();
        const drc::footstep_goal_t& first_lcm_goal = lcm_object.footstep_goals[0];
        first_goal->set_utime(first_lcm_goal.utime);

        if(!RobotStateCodec::to_minimal_position3d(first_lcm_goal.pos, first_goal->mutable_pos(), true, true))
            return false;
        
        first_goal->set_step_speed(first_lcm_goal.step_speed);
        first_goal->set_step_height(first_lcm_goal.step_height);

        first_goal->set_id(first_lcm_goal.id);

        first_goal->set_bool_mask(set_bool_mask(first_lcm_goal));
        
        drc::MinimalFootStepGoalDiff* goal_diff = dccl_plan.mutable_goal_diff();
        for(int i = 1, n = lcm_object.num_steps; i < n; ++i)
        {
            const drc::footstep_goal_t& previous_lcm_goal = lcm_object.footstep_goals[i-1];
            const drc::footstep_goal_t& present_lcm_goal = lcm_object.footstep_goals[i];
            goal_diff->add_utime_diff(present_lcm_goal.utime-previous_lcm_goal.utime);

            if(!RobotStateCodec::to_position3d_diff(present_lcm_goal.pos, previous_lcm_goal.pos,
                                                    goal_diff->mutable_pos_diff(), true, true))
                return false;

            
            goal_diff->add_id_diff(present_lcm_goal.id-previous_lcm_goal.id);
            
            goal_diff->add_bool_mask(set_bool_mask(present_lcm_goal));

            goal_diff->add_step_speed(present_lcm_goal.step_speed);
            goal_diff->add_step_height(present_lcm_goal.step_height);
        }
    }
    else
    {
        glog.is(WARN) && glog << "Received plan with no steps, not sending." << std::endl;
        return false;
    }
    
    glog.is(VERBOSE) && glog << "MinimalFootStepPlan: " << pb_to_short_string(dccl_plan) << std::endl;
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_plan);
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

    glog.is(VERBOSE) && glog << "MinimalFootStepPlan: " << pb_to_short_string(dccl_plan) << std::endl;
    
    drc::footstep_plan_t lcm_object;
    lcm_object.utime = dccl_plan.utime();
    lcm_object.robot_name = "atlas";
    lcm_object.num_steps = dccl_plan.goal_diff().utime_diff_size() + 1; // +1 for first_goal    
    lcm_object.is_new_plan = dccl_plan.is_new_plan();    
    lcm_object.footstep_opts.ignore_terrain = dccl_plan.opts_ignore_terrain();
    lcm_object.footstep_opts.mu = dccl_plan.opts_mu();
    
    
    const drc::MinimalFootStepGoal& first_goal = dccl_plan.first_goal();
    
    drc::footstep_goal_t first_lcm_goal;

    first_lcm_goal.utime = first_goal.utime();
    first_lcm_goal.robot_name = "atlas";
    
    if(!RobotStateCodec::from_minimal_position3d(&first_lcm_goal.pos, first_goal.pos(), true))
        return false;
    
    first_lcm_goal.id = first_goal.id();

    read_bool_mask(first_goal.bool_mask(), &first_lcm_goal);

    first_lcm_goal.step_speed = first_goal.step_speed();
    first_lcm_goal.step_height = first_goal.step_height();

    
    lcm_object.footstep_goals.push_back(first_lcm_goal);

    const drc::MinimalFootStepGoalDiff& goal_diff = dccl_plan.goal_diff();
    drc::MinimalFootStepGoal present_goal = first_goal;    
    for(int i = 0, n = goal_diff.utime_diff_size(); i < n; ++i)
    {

        drc::footstep_goal_t lcm_goal;
        
        if(!RobotStateCodec::from_position3d_diff(present_goal.mutable_pos(), goal_diff.pos_diff(), i, true))
            return false;        
        
        if(!RobotStateCodec::from_minimal_position3d(&lcm_goal.pos, present_goal.pos()))
            return false;
        
        present_goal.set_utime(present_goal.utime() + goal_diff.utime_diff(i));
        present_goal.set_id(present_goal.id() + goal_diff.id_diff(i));
        
        lcm_goal.utime = present_goal.utime();
        lcm_goal.id = present_goal.id();
        
        read_bool_mask(goal_diff.bool_mask(i), &lcm_goal);

        lcm_goal.step_speed = goal_diff.step_speed(i);
        lcm_goal.step_height = goal_diff.step_height(i);
        
        lcm_object.footstep_goals.push_back(lcm_goal);
    }
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());
    
    return true;
          
}      

int FootStepPlanCodec::set_bool_mask(const drc::footstep_goal_t& goal)
{
    int bool_mask = 0;
    if(goal.is_right_foot) bool_mask |= 1;
    if(goal.is_in_contact) bool_mask |= (1 << 1);
    if(goal.fixed_x) bool_mask |= (1 << 2);
    if(goal.fixed_y) bool_mask |= (1 << 3);
    if(goal.fixed_z) bool_mask |= (1 << 4);
    if(goal.fixed_roll) bool_mask |= (1 << 5);
    if(goal.fixed_pitch) bool_mask |= (1 << 6);
    if(goal.fixed_yaw) bool_mask |= (1 << 7);
    return bool_mask;
}

void FootStepPlanCodec::read_bool_mask(int bool_mask, drc::footstep_goal_t * goal)
{
    goal->is_right_foot = (bool_mask & 1);
    goal->is_in_contact = (bool_mask & (1 << 1));
    goal->fixed_x = (bool_mask & (1 << 2));
    goal->fixed_y = (bool_mask & (1 << 3));
    goal->fixed_z = (bool_mask & (1 << 4));
    goal->fixed_roll = (bool_mask & (1 << 5));
    goal->fixed_pitch = (bool_mask & (1 << 6));
    goal->fixed_yaw = (bool_mask & (1 << 7));
}

