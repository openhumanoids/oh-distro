#include "footstep-plan-codecs.h"
#include "robot-state-codecs.h"

FootStepPlanCodec::FootStepPlanCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get())
{
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
        first_goal->mutable_pos()->mutable_translation()->set_x(first_lcm_goal.pos.translation.x);
        first_goal->mutable_pos()->mutable_translation()->set_y(first_lcm_goal.pos.translation.y);
        first_goal->mutable_pos()->mutable_translation()->set_z(first_lcm_goal.pos.translation.z);
        
        first_goal->mutable_pos()->mutable_rotation()->set_x(first_lcm_goal.pos.rotation.x);
        first_goal->mutable_pos()->mutable_rotation()->set_y(first_lcm_goal.pos.rotation.y);
        first_goal->mutable_pos()->mutable_rotation()->set_z(first_lcm_goal.pos.rotation.z);
        first_goal->mutable_pos()->mutable_rotation()->set_w(first_lcm_goal.pos.rotation.w);

        first_goal->set_step_speed(first_lcm_goal.step_speed);
        first_goal->set_step_height(first_lcm_goal.step_height);

        first_goal->set_id(first_lcm_goal.id);

        first_goal->set_bool_mask(set_bool_mask(first_lcm_goal));
        
        drc::MinimalFootStepGoalDiff* goal_diff = dccl_plan.mutable_goal_diff();
        for(int i = 1, n = lcm_object.num_steps; i < n; ++i)
        {
            const drc::footstep_goal_t& previous_lcm_goal = lcm_object.footstep_goals[i-1];
            const drc::footstep_goal_t& later_lcm_goal = lcm_object.footstep_goals[i];
            goal_diff->add_utime_diff(later_lcm_goal.utime-previous_lcm_goal.utime);

            // pre-round the values to avoid cumulative rounding errors
            const int TRANSLATION_X_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dx")->options().GetExtension(dccl::field).precision();
            const int TRANSLATION_Y_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dy")->options().GetExtension(dccl::field).precision();
            const int TRANSLATION_Z_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dz")->options().GetExtension(dccl::field).precision();
           
            goal_diff->mutable_pos_diff()->mutable_translation_diff()->add_dx(
                later_lcm_goal.pos.translation.x-
                goby::util::unbiased_round(previous_lcm_goal.pos.translation.x, TRANSLATION_X_PRECISION));
            goal_diff->mutable_pos_diff()->mutable_translation_diff()->add_dy(
                later_lcm_goal.pos.translation.y-
                goby::util::unbiased_round(previous_lcm_goal.pos.translation.y, TRANSLATION_Y_PRECISION));
            goal_diff->mutable_pos_diff()->mutable_translation_diff()->add_dz(
                later_lcm_goal.pos.translation.z-
                goby::util::unbiased_round(previous_lcm_goal.pos.translation.z, TRANSLATION_Z_PRECISION));

            const int ROTATION_X_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dx")->options().GetExtension(dccl::field).precision();
            const int ROTATION_Y_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dy")->options().GetExtension(dccl::field).precision();
            const int ROTATION_Z_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dz")->options().GetExtension(dccl::field).precision();
            const int ROTATION_W_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dw")->options().GetExtension(dccl::field).precision();

            goal_diff->mutable_pos_diff()->mutable_rotation_diff()->add_dx(
                later_lcm_goal.pos.rotation.x-
                goby::util::unbiased_round(previous_lcm_goal.pos.rotation.x, ROTATION_X_PRECISION));

            goal_diff->mutable_pos_diff()->mutable_rotation_diff()->add_dy(
                later_lcm_goal.pos.rotation.y-
                goby::util::unbiased_round(previous_lcm_goal.pos.rotation.y, ROTATION_Y_PRECISION));
            
            goal_diff->mutable_pos_diff()->mutable_rotation_diff()->add_dz(
                later_lcm_goal.pos.rotation.z-
                goby::util::unbiased_round(previous_lcm_goal.pos.rotation.z, ROTATION_Z_PRECISION));

            goal_diff->mutable_pos_diff()->mutable_rotation_diff()->add_dw(
                later_lcm_goal.pos.rotation.w-
                goby::util::unbiased_round(previous_lcm_goal.pos.rotation.w, ROTATION_W_PRECISION));
            
            goal_diff->add_id_diff(later_lcm_goal.id-previous_lcm_goal.id);
            
            goal_diff->add_bool_mask(set_bool_mask(later_lcm_goal));

            goal_diff->add_step_speed(later_lcm_goal.step_speed);
            goal_diff->add_step_height(later_lcm_goal.step_height);
        }
    }
    else
    {
        glog.is(WARN) && glog << "Received plan with no steps, not sending." << std::endl;
        return false;
    }
    
    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_plan_debug;
        printer.PrintToString(dccl_plan, &dccl_plan_debug);
        glog << "MinimalFootStepPlan: " << dccl_plan_debug << std::endl;
    }
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_plan);
    transmit_data->resize(encoded.size());
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

    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_plan_debug;
        printer.PrintToString(dccl_plan, &dccl_plan_debug);
        glog << "MinimalFootStepPlan: " << dccl_plan_debug << std::endl;
    }

    
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
    
    first_lcm_goal.pos.translation.x = first_goal.pos().translation().x();
    first_lcm_goal.pos.translation.y = first_goal.pos().translation().y();
    first_lcm_goal.pos.translation.z = first_goal.pos().translation().z();

    first_lcm_goal.pos.rotation.x = first_goal.pos().rotation().x();
    first_lcm_goal.pos.rotation.y = first_goal.pos().rotation().y();
    first_lcm_goal.pos.rotation.z = first_goal.pos().rotation().z();
    first_lcm_goal.pos.rotation.w = first_goal.pos().rotation().w();
    quaternion_normalize(first_lcm_goal.pos.rotation);
    
    first_lcm_goal.id = first_goal.id();

    read_bool_mask(first_goal.bool_mask(), &first_lcm_goal);

    first_lcm_goal.step_speed = first_goal.step_speed();
    first_lcm_goal.step_height = first_goal.step_height();

    
    lcm_object.footstep_goals.push_back(first_lcm_goal);

    const drc::MinimalFootStepGoalDiff& goal_diff = dccl_plan.goal_diff();
    for(int i = 0, n = goal_diff.utime_diff_size(); i < n; ++i)
    {
        // initialize to previous goal
        drc::footstep_goal_t lcm_goal = lcm_object.footstep_goals[i];

        lcm_goal.utime += goal_diff.utime_diff(i);
        lcm_goal.pos.translation.x += goal_diff.pos_diff().translation_diff().dx(i);
        lcm_goal.pos.translation.y += goal_diff.pos_diff().translation_diff().dy(i);
        lcm_goal.pos.translation.z += goal_diff.pos_diff().translation_diff().dz(i);
        
        lcm_goal.pos.rotation.x += goal_diff.pos_diff().rotation_diff().dx(i);
        lcm_goal.pos.rotation.y += goal_diff.pos_diff().rotation_diff().dy(i);
        lcm_goal.pos.rotation.z += goal_diff.pos_diff().rotation_diff().dz(i);
        lcm_goal.pos.rotation.w += goal_diff.pos_diff().rotation_diff().dw(i);
        quaternion_normalize(lcm_goal.pos.rotation);
        
        lcm_goal.id += goal_diff.id_diff(i);
        
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

