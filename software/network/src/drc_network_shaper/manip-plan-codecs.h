#ifndef MANIPPLANCODECS20130517H
#define MANIPPLANCODECS20130517H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/robot_plan_with_supports_t.hpp"

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


class SupportsPlanCodec : public CustomChannelCodec
{
  public:
    SupportsPlanCodec(const std::string loopback_channel = "");
        
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

    drc::MinimalRobotState previous_goal = dccl_plan.goal();

    for(int i = 0, n = previous_goal.full().joint_position_size(); i < n; ++i)
    {
        previous_goal.mutable_full()->set_joint_position(i, goby::util::unbiased_round(previous_goal.full().joint_position(i), JOINT_POS_PRECISION));
    }

    for(int i = 1, n = lcm_object.plan.size(); i < n; ++i)
    {
        const drc::robot_state_t& previous_lcm_goal = lcm_object.plan[i-1];
        const drc::robot_state_t& present_lcm_goal = lcm_object.plan[i];
        
        drc::MinimalRobotState present_goal;
        RobotStateCodec::to_minimal_state(present_lcm_goal, &present_goal);

        drc::MinimalRobotStateDiff* present_goal_diff = dccl_plan.mutable_goal_diff();
        present_goal_diff->add_utime_diff(present_lcm_goal.utime - previous_lcm_goal.utime);
        if(!RobotStateCodec::to_position3d_diff(present_lcm_goal.pose,
                                                previous_lcm_goal.pose,
                                                present_goal_diff->mutable_pos_diff()))
            return false;


        // pre-round everything
        for(int j = 0, m = present_goal.full().joint_position_size(); j < m; ++j)
        {
            present_goal.mutable_full()->set_joint_position(j, goby::util::unbiased_round(present_goal.full().joint_position(j), JOINT_POS_PRECISION));
        }
        
        for(int j = 0, m = present_goal.full().joint_position_size(); j < m; ++j)
        {
            const google::protobuf::Reflection* present_goal_diff_refl = present_goal_diff->GetReflection();
            const google::protobuf::Descriptor* present_goal_diff_desc = present_goal_diff->GetDescriptor();

            const std::string& joint_name = present_lcm_goal.joint_name.at(j);
            const google::protobuf::FieldDescriptor* present_goal_diff_field_desc = present_goal_diff_desc->FindFieldByName("joint_pos_diff_" + joint_name);
            if(!present_goal_diff_field_desc)
            {
                glog.is(DIE) && glog << "No hardcoded joint called " << joint_name << " present in MinimalRobotStateDiff DCCL message!" << std::endl;
            }
            
            float jp_diff = present_goal.full().joint_position(RobotStateCodec::joint_names_to_order_[joint_name]) - previous_goal.full().joint_position(RobotStateCodec::joint_names_to_order_[joint_name]);
            present_goal_diff_refl->AddFloat(present_goal_diff, present_goal_diff_field_desc, jp_diff);

            /* if(i)  */
             /*     std::cout << ",";  */
             /*  std::cout << present_goal.joint_position(i) - previous_goal.joint_position(i);  */
        }
        
//        std::cout << std::endl;        

        previous_goal = present_goal;
    }

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
                                                  dccl_grasp->mutable_joint_id(),
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

    return true;
}

template<class LCMRobotPlan>
bool to_minimal_robot_plan_control_type_new(const LCMRobotPlan& lcm_object, drc::MinimalRobotPlan& dccl_plan)
{
    dccl_plan.set_left_arm_control_type(lcm_object.left_arm_control_type);
    dccl_plan.set_right_arm_control_type(lcm_object.right_arm_control_type);
    dccl_plan.set_left_leg_control_type(lcm_object.left_leg_control_type);
    dccl_plan.set_right_leg_control_type(lcm_object.right_leg_control_type);    

    return true;
}

template<class LCMRobotPlan>
bool to_minimal_robot_plan_control_type_old(const LCMRobotPlan& lcm_object, drc::MinimalRobotPlan& dccl_plan)
{
    dccl_plan.set_left_arm_control_type(lcm_object.arms_control_type);
    dccl_plan.set_left_leg_control_type(lcm_object.legs_control_type);
    return true;
}


template<class LCMRobotPlan>
bool from_minimal_robot_plan_control_type_new(LCMRobotPlan& lcm_object, const drc::MinimalRobotPlan& dccl_plan)
{    
    lcm_object.left_arm_control_type = dccl_plan.left_arm_control_type();
    lcm_object.right_arm_control_type = dccl_plan.right_arm_control_type();
    lcm_object.left_leg_control_type = dccl_plan.left_leg_control_type();
    lcm_object.right_leg_control_type = dccl_plan.right_leg_control_type();
    return true;
}

template<class LCMRobotPlan>
bool from_minimal_robot_plan_control_type_old(LCMRobotPlan& lcm_object, const drc::MinimalRobotPlan& dccl_plan)
{
    lcm_object.arms_control_type = dccl_plan.left_arm_control_type();
    lcm_object.legs_control_type = dccl_plan.left_leg_control_type();
    return true;
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
         

         for(int j = 0, m = first_lcm_goal.joint_name.size(); j < m; ++j)
         {
             
             const google::protobuf::Reflection* present_goal_diff_refl = dccl_plan.goal_diff().GetReflection();
             const google::protobuf::Descriptor* present_goal_diff_desc = dccl_plan.goal_diff().GetDescriptor();
             
             const std::string& joint_name = first_lcm_goal.joint_name.at(j);
             const google::protobuf::FieldDescriptor* present_goal_diff_field_desc = present_goal_diff_desc->FindFieldByName("joint_pos_diff_" + joint_name);
             if(!present_goal_diff_field_desc)
             {
                 glog.is(DIE) && glog << "No hardcoded joint called " << joint_name << " present in MinimalRobotStateDiff DCCL message!" << std::endl;
             }
             present_goal.mutable_full()->set_joint_position(RobotStateCodec::joint_names_to_order_[joint_name],
                                             present_goal_diff_refl->GetRepeatedFloat(dccl_plan.goal_diff(), present_goal_diff_field_desc, i) +
                                             present_goal.full().joint_position(RobotStateCodec::joint_names_to_order_[joint_name]));
         }

         
         if(!RobotStateCodec::from_position3d_diff(present_goal.mutable_pose(),
                                                   dccl_plan.goal_diff().pos_diff(), i))
            return false;

         if(!RobotStateCodec::from_minimal_state(&lcm_goal, present_goal))
             return false;
         
         lcm_object.plan.push_back(lcm_goal);
    }
    
    
    lcm_object.num_grasp_transitions = dccl_plan.grasp_size();
    for(int i = 0, n = dccl_plan.grasp_size(); i < n; ++i)
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
                                                    dccl_grasp.joint_id(),
                                                    offset))
            return false;

        lcm_object.grasps.push_back(present_grasp);
    }
    
    lcm_object.num_bytes = 0;

    lcm_object.plan_info = std::vector<int32_t>(lcm_object.num_states, 0);
    
    return true;
}


#endif
