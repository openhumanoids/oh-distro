#include "manip-plan-codecs.h"
#include "robot-state-codecs.h"

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

    dccl_plan.set_utime(lcm_object.utime);

    RobotStateCodec::to_minimal_state(lcm_object.plan[0], dccl_plan.mutable_goal());    
    

    for(int i = 1, n = lcm_object.plan.size(); i < n; ++i)
    {
        const drc::robot_state_t& previous_lcm_goal = lcm_object.plan[i-1];
        const drc::robot_state_t& present_lcm_goal = lcm_object.plan[i];
        
        drc::MinimalRobotState present_goal;
        RobotStateCodec::to_minimal_state(present_lcm_goal, &present_goal);

        drc::MinimalRobotStateDiff* present_goal_diff = dccl_plan.mutable_goal_diff();
        present_goal_diff->add_utime_diff(present_lcm_goal.utime - previous_lcm_goal.utime);
        if(!RobotStateCodec::to_position3d_diff(present_lcm_goal.origin_position,
                                                previous_lcm_goal.origin_position,
                                                present_goal_diff->mutable_pos_diff()))
           return false;


        drc::MinimalRobotStateDiff::JointPositionDiff* joint_pos_diff = present_goal_diff->add_joint_pos_diff();
        
        joint_pos_diff->mutable_jp_diff_val()->CopyFrom(present_goal.joint_position());
    }
    
    // add grasp

    dccl_plan.set_arms_control_type(lcm_object.arms_control_type);
    dccl_plan.set_legs_control_type(lcm_object.legs_control_type);
    
    
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

    lcm_object.utime = dccl_plan.utime();
    lcm_object.robot_name = "atlas";
    lcm_object.num_states = dccl_plan.goal_diff().utime_diff().size() + 1;

    drc::robot_state_t first_lcm_goal;

    if(!RobotStateCodec::from_minimal_state(&first_lcm_goal, dccl_plan.goal()))
        return false;

    lcm_object.plan.push_back(first_lcm_goal);

    drc::MinimalRobotState present_goal = dccl_plan.goal();
    for(int i = 0, n = dccl_plan.goal_diff().utime_diff().size(); i < n; ++i)
    {
         drc::robot_state_t lcm_goal;

         present_goal.set_utime(present_goal.utime() + dccl_plan.goal_diff().utime_diff(i));
         lcm_goal.utime = present_goal.utime();
         present_goal.mutable_joint_position()->CopyFrom(dccl_plan.goal_diff().joint_pos_diff(i).jp_diff_val());

         if(!RobotStateCodec::from_position3d_diff(present_goal.mutable_origin_position(),
                                                   dccl_plan.goal_diff().pos_diff(), i))
            return false;

         if(!RobotStateCodec::from_minimal_state(&lcm_goal, present_goal))
             return false;
         
         lcm_object.plan.push_back(lcm_goal);
    }
    
    
    // add grasp
    lcm_object.num_grasp_transitions = 0;

    
    lcm_object.arms_control_type = dccl_plan.arms_control_type();
    lcm_object.legs_control_type = dccl_plan.legs_control_type();

    lcm_object.num_bytes = 0;
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    return true;
          
}      
