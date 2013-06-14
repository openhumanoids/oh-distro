#ifndef MANIPPLANCODECS20130517H
#define MANIPPLANCODECS20130517H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/aff_indexed_robot_plan_t.hpp"

#include "robot-plan-analogs.pb.h"

#include "robot-state-codecs.h"

enum { JOINT_POS_PRECISION = 3 };

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

    RobotStateCodec::to_minimal_state(lcm_object.plan[0], dccl_plan.mutable_goal(), true);

    drc::MinimalRobotState previous_goal = dccl_plan.goal();

    for(int i = 0, n = previous_goal.joint_position_size(); i < n; ++i)
    {
        previous_goal.set_joint_position(i, goby::util::unbiased_round(previous_goal.joint_position(i), JOINT_POS_PRECISION));
    }

    for(int i = 1, n = lcm_object.plan.size(); i < n; ++i)
    {
        const drc::robot_state_t& previous_lcm_goal = lcm_object.plan[i-1];
        const drc::robot_state_t& present_lcm_goal = lcm_object.plan[i];
        
        drc::MinimalRobotState present_goal;
        RobotStateCodec::to_minimal_state(present_lcm_goal, &present_goal, true);

        drc::MinimalRobotStateDiff* present_goal_diff = dccl_plan.mutable_goal_diff();
        present_goal_diff->add_utime_diff(present_lcm_goal.utime - previous_lcm_goal.utime);
        if(!RobotStateCodec::to_position3d_diff(present_lcm_goal.origin_position,
                                                previous_lcm_goal.origin_position,
                                                present_goal_diff->mutable_pos_diff(),
                                                true))
           return false;

        
        for(int i = 0, n = present_goal.joint_position_size(); i < n; ++i)
        {
            present_goal.set_joint_position(i, goby::util::unbiased_round(present_goal.joint_position(i), JOINT_POS_PRECISION));

             /* if(i)  */
             /*     std::cout << ",";  */
             /*  std::cout << present_goal.joint_position(i) - previous_goal.joint_position(i);  */
        }
        
//        std::cout << std::endl;
        
        
        present_goal_diff->add_joint_pos_diff_back_lbz(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_lbz"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_lbz"]));
        present_goal_diff->add_joint_pos_diff_back_mby(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_mby"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_mby"]));
        present_goal_diff->add_joint_pos_diff_back_ubx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_ubx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_ubx"]));

        present_goal_diff->add_joint_pos_diff_neck_ay(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["neck_ay"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["neck_ay"]));
        

        present_goal_diff->add_joint_pos_diff_l_leg_uhz(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uhz"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uhz"]));
        present_goal_diff->add_joint_pos_diff_l_leg_mhx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_mhx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_mhx"]));
        present_goal_diff->add_joint_pos_diff_l_leg_lhy(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lhy"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lhy"]));
        present_goal_diff->add_joint_pos_diff_l_leg_kny(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_kny"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_kny"]));
        present_goal_diff->add_joint_pos_diff_l_leg_uay(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uay"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uay"]));
        present_goal_diff->add_joint_pos_diff_l_leg_lax(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lax"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lax"]));


        present_goal_diff->add_joint_pos_diff_r_leg_uhz(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uhz"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uhz"]));
        present_goal_diff->add_joint_pos_diff_r_leg_mhx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_mhx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_mhx"]));
        present_goal_diff->add_joint_pos_diff_r_leg_lhy(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lhy"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lhy"]));
        present_goal_diff->add_joint_pos_diff_r_leg_kny(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_kny"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_kny"]));
        present_goal_diff->add_joint_pos_diff_r_leg_uay(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uay"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uay"]));
        present_goal_diff->add_joint_pos_diff_r_leg_lax(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lax"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lax"]));

        
        present_goal_diff->add_joint_pos_diff_l_arm_usy(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_usy"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_usy"]));
        present_goal_diff->add_joint_pos_diff_l_arm_shx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_shx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_shx"]));
        present_goal_diff->add_joint_pos_diff_l_arm_ely(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_ely"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_ely"]));
        present_goal_diff->add_joint_pos_diff_l_arm_elx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_elx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_elx"]));
        present_goal_diff->add_joint_pos_diff_l_arm_uwy(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_uwy"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_uwy"]));
        present_goal_diff->add_joint_pos_diff_l_arm_mwx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_mwx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_mwx"]));
        
        present_goal_diff->add_joint_pos_diff_r_arm_usy(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_usy"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_usy"]));
        present_goal_diff->add_joint_pos_diff_r_arm_shx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_shx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_shx"]));
        present_goal_diff->add_joint_pos_diff_r_arm_ely(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_ely"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_ely"]));
        present_goal_diff->add_joint_pos_diff_r_arm_elx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_elx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_elx"]));
        present_goal_diff->add_joint_pos_diff_r_arm_uwy(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_uwy"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_uwy"]));
        present_goal_diff->add_joint_pos_diff_r_arm_mwx(present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_mwx"]) -
                                                       previous_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_mwx"]));
        
        
//        drc::MinimalRobotStateDiff::JointPositionDiff* joint_pos_diff = present_goal_diff->add_joint_pos_diff();      
//        joint_pos_diff->mutable_jp_diff_val()->CopyFrom(present_goal.joint_position());

        previous_goal = present_goal;
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

    if(!RobotStateCodec::from_minimal_state(&first_lcm_goal, dccl_plan.goal(), true))
        return false;

    lcm_object.plan.push_back(first_lcm_goal);

    drc::MinimalRobotState present_goal = dccl_plan.goal();
    for(int i = 0, n = dccl_plan.goal_diff().utime_diff().size(); i < n; ++i)
    {
         drc::robot_state_t lcm_goal;

         present_goal.set_utime(present_goal.utime() + dccl_plan.goal_diff().utime_diff(i));
         lcm_goal.utime = present_goal.utime();
         
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["back_lbz"],
                                         dccl_plan.goal_diff().joint_pos_diff_back_lbz(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_lbz"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["back_mby"],
                                         dccl_plan.goal_diff().joint_pos_diff_back_mby(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_mby"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["back_ubx"],
                                         dccl_plan.goal_diff().joint_pos_diff_back_ubx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["back_ubx"]));
         
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["neck_ay"],
                                         dccl_plan.goal_diff().joint_pos_diff_neck_ay(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["neck_ay"]));
         
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uhz"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_leg_uhz(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uhz"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_leg_mhx"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_leg_mhx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_mhx"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lhy"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_leg_lhy(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lhy"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_leg_kny"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_leg_kny(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_kny"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uay"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_leg_uay(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_uay"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lax"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_leg_lax(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_leg_lax"]));
         
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uhz"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_leg_uhz(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uhz"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_leg_mhx"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_leg_mhx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_mhx"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lhy"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_leg_lhy(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lhy"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_leg_kny"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_leg_kny(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_kny"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uay"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_leg_uay(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_uay"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lax"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_leg_lax(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_leg_lax"]));
         
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_arm_usy"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_arm_usy(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_usy"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_arm_shx"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_arm_shx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_shx"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_arm_ely"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_arm_ely(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_ely"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_arm_elx"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_arm_elx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_elx"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_arm_uwy"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_arm_uwy(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_uwy"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["l_arm_mwx"],
                                         dccl_plan.goal_diff().joint_pos_diff_l_arm_mwx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["l_arm_mwx"]));
         
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_arm_usy"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_arm_usy(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_usy"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_arm_shx"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_arm_shx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_shx"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_arm_ely"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_arm_ely(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_ely"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_arm_elx"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_arm_elx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_elx"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_arm_uwy"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_arm_uwy(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_uwy"]));
         present_goal.set_joint_position(RobotStateCodec::joint_names_to_order_["r_arm_mwx"],
                                         dccl_plan.goal_diff().joint_pos_diff_r_arm_mwx(i) +
                                         present_goal.joint_position(RobotStateCodec::joint_names_to_order_["r_arm_mwx"]));


         
         if(!RobotStateCodec::from_position3d_diff(present_goal.mutable_origin_position(),
                                                   dccl_plan.goal_diff().pos_diff(), i, true))
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
    return true;
}


#endif
