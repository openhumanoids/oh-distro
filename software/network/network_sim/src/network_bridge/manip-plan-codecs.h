#ifndef MANIPPLANCODECS20130517H
#define MANIPPLANCODECS20130517H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/aff_indexed_robot_plan_t.hpp"

#include "robot-plan-analogs.pb.h"

#include "robot-state-codecs.h"

class ManipPlanCodec : public CustomChannelCodec
{
  public:
    ManipPlanCodec(const std::string loopback_channel = "");
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);
    
  private:
    goby::acomms::DCCLCodec* dccl_;        
};

class ManipMapCodec : public CustomChannelCodec
{
  public:
    ManipMapCodec(const std::string loopback_channel = "");
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);
    
  private:
    goby::acomms::DCCLCodec* dccl_;
        
        
};


template<class LCMRobotPlan>
bool to_minimal_robot_plan(const LCMRobotPlan& lcm_object, drc::MinimalRobotPlan& dccl_plan)
{
    using goby::glog;
    using namespace goby::common::logger;
    
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

    dccl_plan.set_num_grasp_transitions(lcm_object.num_grasp_transitions);
    for(int i = 0, n = lcm_object.grasps.size(); i < n; ++i)
    {
        const drc::grasp_transition_state_t& present_grasp = lcm_object.grasps[i];
        drc::GraspTransition* dccl_grasp = dccl_plan.add_grasp();
        dccl_grasp->set_utime(present_grasp.utime);
        dccl_grasp->set_affordance_uid(present_grasp.affordance_uid);

        if(!RobotStateCodec::to_minimal_position3d(present_grasp.hand_pose, dccl_grasp->mutable_hand_pose()))
            return false;
        
        dccl_grasp->set_grasp_on(present_grasp.grasp_on);
        dccl_grasp->set_grasp_type(present_grasp.grasp_type);
        dccl_grasp->set_power_grasp(present_grasp.power_grasp);

        int offset = (present_grasp.grasp_type == drc::grasp_transition_state_t::LEFT) ?
            RobotStateCodec::joint_names_to_order_["left_f0_j0"] :
            RobotStateCodec::joint_names_to_order_["right_f0_j0"];
        
        if(!RobotStateCodec::to_minimal_joint_pos(present_grasp.joint_name,
                                                  present_grasp.joint_position,
                                                  dccl_grasp->mutable_joint_position(),
                                                  offset))
            return false;
        
    }
 
    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_plan_debug;
        printer.PrintToString(dccl_plan, &dccl_plan_debug);
        glog << "MinimalRobotPlan: " << dccl_plan_debug << std::endl;
    }

}

template<class LCMRobotPlan>
bool from_minimal_robot_plan(LCMRobotPlan& lcm_object, const drc::MinimalRobotPlan& dccl_plan)
{
    using goby::glog;
    using namespace goby::common::logger;

    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_plan_debug;
        printer.PrintToString(dccl_plan, &dccl_plan_debug);
        glog << "MinimalRobotPlan: " << dccl_plan_debug << std::endl;
    }

    
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
    
    
    lcm_object.num_grasp_transitions = dccl_plan.num_grasp_transitions();
    for(int i = 0, n = dccl_plan.num_grasp_transitions(); i < n; ++i)
    {
        drc::grasp_transition_state_t present_grasp;
        const drc::GraspTransition& dccl_grasp = dccl_plan.grasp(i);
        present_grasp.utime = dccl_grasp.utime();
        present_grasp.affordance_uid = dccl_grasp.affordance_uid();

        if(!RobotStateCodec::from_minimal_position3d(&present_grasp.hand_pose, dccl_grasp.hand_pose()))
            return false;
        
        present_grasp.grasp_on = dccl_grasp.grasp_on();
        present_grasp.grasp_type = dccl_grasp.grasp_type();
        present_grasp.power_grasp = dccl_grasp.power_grasp();

        int offset = (present_grasp.grasp_type == drc::grasp_transition_state_t::LEFT) ?
            RobotStateCodec::joint_names_to_order_["left_f0_j0"] :
            RobotStateCodec::joint_names_to_order_["right_f0_j0"];

        present_grasp.num_joints = dccl_grasp.joint_position_size();
        if(!RobotStateCodec::from_minimal_joint_pos(&present_grasp.joint_name,
                                                    &present_grasp.joint_position,
                                                    dccl_grasp.joint_position(),
                                                    offset))
            return false;

        lcm_object.grasps.push_back(present_grasp);
    }
    
    lcm_object.num_bytes = 0;    
}




#endif
