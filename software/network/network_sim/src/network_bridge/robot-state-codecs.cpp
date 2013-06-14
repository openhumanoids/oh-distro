#include "robot-state-codecs.h"
#include "bot_core/bot_core.h"

std::map<std::string, int> RobotStateCodec::joint_names_to_order_;
std::vector<std::string> RobotStateCodec::joint_names_;

using goby::glog;
using namespace goby::common::logger;


RobotStateCodec::RobotStateCodec(const std::string loopback_channel)
    : CustomChannelCodec(loopback_channel),
     dccl_(goby::acomms::DCCLCodec::get())
{
    dccl_->validate<drc::MinimalRobotState>();

    if(joint_names_.empty())
    {
        int i = 0;
        joint_names_to_order_.insert(std::make_pair("back_lbz", i++));
        joint_names_to_order_.insert(std::make_pair("back_mby", i++));
        joint_names_to_order_.insert(std::make_pair("back_ubx", i++));
        joint_names_to_order_.insert(std::make_pair("neck_ay", i++));
        joint_names_to_order_.insert(std::make_pair("l_leg_uhz", i++));
        joint_names_to_order_.insert(std::make_pair("l_leg_mhx", i++));
        joint_names_to_order_.insert(std::make_pair("l_leg_lhy", i++));
        joint_names_to_order_.insert(std::make_pair("l_leg_kny", i++));
        joint_names_to_order_.insert(std::make_pair("l_leg_uay", i++));
        joint_names_to_order_.insert(std::make_pair("l_leg_lax", i++));
        joint_names_to_order_.insert(std::make_pair("r_leg_uhz", i++));
        joint_names_to_order_.insert(std::make_pair("r_leg_mhx", i++));
        joint_names_to_order_.insert(std::make_pair("r_leg_lhy", i++));
        joint_names_to_order_.insert(std::make_pair("r_leg_kny", i++));
        joint_names_to_order_.insert(std::make_pair("r_leg_uay", i++));
        joint_names_to_order_.insert(std::make_pair("r_leg_lax", i++));
        joint_names_to_order_.insert(std::make_pair("l_arm_usy", i++));
        joint_names_to_order_.insert(std::make_pair("l_arm_shx", i++));
        joint_names_to_order_.insert(std::make_pair("l_arm_ely", i++));
        joint_names_to_order_.insert(std::make_pair("l_arm_elx", i++));
        joint_names_to_order_.insert(std::make_pair("l_arm_uwy", i++));
        joint_names_to_order_.insert(std::make_pair("l_arm_mwx", i++));
        joint_names_to_order_.insert(std::make_pair("r_arm_usy", i++));
        joint_names_to_order_.insert(std::make_pair("r_arm_shx", i++));
        joint_names_to_order_.insert(std::make_pair("r_arm_ely", i++));
        joint_names_to_order_.insert(std::make_pair("r_arm_elx", i++));
        joint_names_to_order_.insert(std::make_pair("r_arm_uwy", i++));
        joint_names_to_order_.insert(std::make_pair("r_arm_mwx", i++));
        joint_names_to_order_.insert(std::make_pair("hokuyo_joint", i++));
        joint_names_to_order_.insert(std::make_pair("left_f0_j0", i++));
        joint_names_to_order_.insert(std::make_pair("left_f0_j1", i++));
        joint_names_to_order_.insert(std::make_pair("left_f0_j2", i++));
        joint_names_to_order_.insert(std::make_pair("left_f1_j0", i++));
        joint_names_to_order_.insert(std::make_pair("left_f1_j1", i++));
        joint_names_to_order_.insert(std::make_pair("left_f1_j2", i++));
        joint_names_to_order_.insert(std::make_pair("left_f2_j0", i++));
        joint_names_to_order_.insert(std::make_pair("left_f2_j1", i++));
        joint_names_to_order_.insert(std::make_pair("left_f2_j2", i++));
        joint_names_to_order_.insert(std::make_pair("left_f3_j0", i++));
        joint_names_to_order_.insert(std::make_pair("left_f3_j1", i++));
        joint_names_to_order_.insert(std::make_pair("left_f3_j2", i++));
        joint_names_to_order_.insert(std::make_pair("right_f0_j0", i++));
        joint_names_to_order_.insert(std::make_pair("right_f0_j1", i++));
        joint_names_to_order_.insert(std::make_pair("right_f0_j2", i++));
        joint_names_to_order_.insert(std::make_pair("right_f1_j0", i++));
        joint_names_to_order_.insert(std::make_pair("right_f1_j1", i++));
        joint_names_to_order_.insert(std::make_pair("right_f1_j2", i++));
        joint_names_to_order_.insert(std::make_pair("right_f2_j0", i++));
        joint_names_to_order_.insert(std::make_pair("right_f2_j1", i++));
        joint_names_to_order_.insert(std::make_pair("right_f2_j2", i++));
        joint_names_to_order_.insert(std::make_pair("right_f3_j0", i++));
        joint_names_to_order_.insert(std::make_pair("right_f3_j1", i++));
        joint_names_to_order_.insert(std::make_pair("right_f3_j2", i++));;

        joint_names_.resize(i);
        for(std::map<std::string, int>::const_iterator it = joint_names_to_order_.begin(),
                end = joint_names_to_order_.end(); it != end; ++it)
            joint_names_[it->second] = it->first;
    }
}

bool RobotStateCodec::encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data)
{
    
    drc::robot_state_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalRobotState dccl_state;

    if(!to_minimal_state(lcm_object, &dccl_state))
        return false;
    
    glog.is(VERBOSE) && glog << "MinimalRobotState: " << dccl_state.ShortDebugString() << std::endl;
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_state);
    transmit_data->resize(encoded.size());
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}

      
bool RobotStateCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalRobotState dccl_state;

    dccl_->decode(encoded, &dccl_state);
    
    drc::robot_state_t lcm_object;

    if(!from_minimal_state(&lcm_object, dccl_state))
        return false;
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());
    
    return true;
          
}      

bool RobotStateCodec::to_minimal_state(const drc::robot_state_t& lcm_object,
                                       drc::MinimalRobotState* dccl_state,
                                       bool use_rpy /* = false */)
{
    dccl_state->set_utime(lcm_object.utime);

    if(!to_minimal_position3d(lcm_object.origin_position, dccl_state->mutable_origin_position(), use_rpy))
        return false;


    if(!to_minimal_joint_pos(lcm_object.joint_name,
                             lcm_object.joint_position,
                             dccl_state->mutable_joint_position()))
        return false;

    return true;
}

    
bool RobotStateCodec::from_minimal_state(drc::robot_state_t* lcm_object,
                                         const drc::MinimalRobotState& dccl_state,
                                         bool use_rpy /* = false */)
{

    lcm_object->utime = dccl_state.utime();
    lcm_object->robot_name = "atlas";

    if(!from_minimal_position3d(&lcm_object->origin_position,dccl_state.origin_position(), use_rpy))
        return false;    
    
    lcm_object->num_joints = dccl_state.joint_position_size();

    
    if(!from_minimal_joint_pos(&lcm_object->joint_name,
                               &lcm_object->joint_position,
                               dccl_state.joint_position()))
        return false;
    
    std::vector<float> joint_zeros;
    joint_zeros.assign ( joint_names_.size(),0); 
    drc::joint_covariance_t j_cov;
    j_cov.variance = 0;
    std::vector<drc::joint_covariance_t> joint_cov_zeros;
    joint_cov_zeros.assign ( joint_names_.size(),j_cov);  
    lcm_object->origin_twist.linear_velocity.x =0;
    lcm_object->origin_twist.linear_velocity.y =0;
    lcm_object->origin_twist.linear_velocity.z =0;
    lcm_object->origin_twist.angular_velocity.x =0;
    lcm_object->origin_twist.angular_velocity.y =0;
    lcm_object->origin_twist.angular_velocity.z =0;  
    for(int i = 0; i < 6; i++)  {
        for(int j = 0; j < 6; j++) {
            lcm_object->origin_cov.position_cov[i][j] = 0;
            lcm_object->origin_cov.twist_cov[i][j] = 0;
        }
    }
    lcm_object->joint_velocity = joint_zeros;
    lcm_object->measured_effort = joint_zeros;
    lcm_object->joint_cov = joint_cov_zeros;

    drc::contact_state_t cs;
    cs.num_contacts = 0;
    lcm_object->contacts = cs;

    return true;
}

bool RobotStateCodec::to_minimal_position3d(const drc::position_3d_t& lcm_pos,
                                            drc::Position3D* dccl_pos, bool use_rpy /* = false */,
                                            bool omit_z /* = false */)
{
    dccl_pos->mutable_translation()->set_x(lcm_pos.translation.x);
    dccl_pos->mutable_translation()->set_y(lcm_pos.translation.y);

    if(!omit_z)
        dccl_pos->mutable_translation()->set_z(lcm_pos.translation.z);

    if(!use_rpy)
    {
        dccl_pos->mutable_rotation()->set_x(lcm_pos.rotation.x);
        dccl_pos->mutable_rotation()->set_y(lcm_pos.rotation.y);
        dccl_pos->mutable_rotation()->set_z(lcm_pos.rotation.z);
        dccl_pos->mutable_rotation()->set_w(lcm_pos.rotation.w);
    } 
    else
    {
        double q[4] = { 0 };
        double rpy[3] = { 0 };

        q[0] = lcm_pos.rotation.w;
        q[1] = lcm_pos.rotation.x;
        q[2] = lcm_pos.rotation.y;
        q[3] = lcm_pos.rotation.z;
        
        bot_quat_to_roll_pitch_yaw(q, rpy);

        dccl_pos->mutable_rpy_rotation()->set_roll(rpy[0]);
        dccl_pos->mutable_rpy_rotation()->set_pitch(rpy[1]);

        wrap_minus_pi_to_pi(rpy[2]);
        dccl_pos->mutable_rpy_rotation()->set_yaw(rpy[2]);
    }
    
    return true;
}


bool RobotStateCodec::from_minimal_position3d(drc::position_3d_t* lcm_pos,
                                              const drc::Position3D& dccl_pos,
                                              bool use_rpy /* = false */)
{
    lcm_pos->translation.x = dccl_pos.translation().x();
    lcm_pos->translation.y = dccl_pos.translation().y();
    if(dccl_pos.translation().has_z())
        lcm_pos->translation.z = dccl_pos.translation().z();
    else
        lcm_pos->translation.z = std::numeric_limits<double>::quiet_NaN();
        
    if(!use_rpy)
    {
        const drc::RotationQuaternion& rotation = dccl_pos.rotation();
        lcm_pos->rotation.x = rotation.x();
        lcm_pos->rotation.y = rotation.y();
        lcm_pos->rotation.z = rotation.z();
        lcm_pos->rotation.w = rotation.w();
    }
    else
    {
        double q[4] = { 0 };
        double rpy[3] = { 0 };

        rpy[0] = dccl_pos.rpy_rotation().roll();
        rpy[1] = dccl_pos.rpy_rotation().pitch();
        rpy[2] = dccl_pos.rpy_rotation().yaw();
        
        bot_roll_pitch_yaw_to_quat(rpy, q);
        
        lcm_pos->rotation.w = q[0];
        lcm_pos->rotation.x = q[1];
        lcm_pos->rotation.y = q[2];
        lcm_pos->rotation.z = q[3];
    }
    
    quaternion_normalize(lcm_pos->rotation);    

    return true;
}


bool RobotStateCodec::to_position3d_diff(const drc::position_3d_t& present_pos,
                                         const drc::position_3d_t& previous_pos,
                                         drc::Position3DDiff* pos_diff,
                                         bool use_rpy /* = false */,
                                         bool omit_z /* = false */)
{

    // pre-round the values to avoid cumulative rounding errors
    const int TRANSLATION_X_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dx")->options().GetExtension(dccl::field).precision();
    const int TRANSLATION_Y_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dy")->options().GetExtension(dccl::field).precision();
    const int TRANSLATION_Z_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dz")->options().GetExtension(dccl::field).precision();
           
    pos_diff->mutable_translation_diff()->add_dx(present_pos.translation.x-
        goby::util::unbiased_round(previous_pos.translation.x, TRANSLATION_X_PRECISION));
    pos_diff->mutable_translation_diff()->add_dy(present_pos.translation.y-
        goby::util::unbiased_round(previous_pos.translation.y, TRANSLATION_Y_PRECISION));

    if(!omit_z)
    {
        pos_diff->mutable_translation_diff()->add_dz(present_pos.translation.z-goby::util::unbiased_round(previous_pos.translation.z, TRANSLATION_Z_PRECISION));
    }
    
    
    if(!use_rpy)
    {
        const int ROTATION_X_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dx")->options().GetExtension(dccl::field).precision();
        const int ROTATION_Y_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dy")->options().GetExtension(dccl::field).precision();
        const int ROTATION_Z_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dz")->options().GetExtension(dccl::field).precision();
        const int ROTATION_W_PRECISION = drc::RotationQuaternionDiff::descriptor()->FindFieldByName("dw")->options().GetExtension(dccl::field).precision();
        
        pos_diff->mutable_rotation_diff()->add_dx(present_pos.rotation.x-
                                                  goby::util::unbiased_round(previous_pos.rotation.x, ROTATION_X_PRECISION));
        
        pos_diff->mutable_rotation_diff()->add_dy(present_pos.rotation.y-
                                                  goby::util::unbiased_round(previous_pos.rotation.y, ROTATION_Y_PRECISION));
        
        pos_diff->mutable_rotation_diff()->add_dz(present_pos.rotation.z-
                                                  goby::util::unbiased_round(previous_pos.rotation.z, ROTATION_Z_PRECISION));
        
        pos_diff->mutable_rotation_diff()->add_dw(present_pos.rotation.w-
                                                  goby::util::unbiased_round(previous_pos.rotation.w, ROTATION_W_PRECISION));
    }
    else
    {
        double present_q[4] = { 0 };
        double present_rpy[3] = { 0 };
        double previous_q[4] = { 0 };
        double previous_rpy[3] = { 0 };

        present_q[0] = present_pos.rotation.w;
        present_q[1] = present_pos.rotation.x;
        present_q[2] = present_pos.rotation.y;
        present_q[3] = present_pos.rotation.z;
        previous_q[0] = previous_pos.rotation.w;
        previous_q[1] = previous_pos.rotation.x;
        previous_q[2] = previous_pos.rotation.y;
        previous_q[3] = previous_pos.rotation.z;
        
        bot_quat_to_roll_pitch_yaw(present_q, present_rpy);
        bot_quat_to_roll_pitch_yaw(previous_q, previous_rpy);

        const int ROTATION_ROLL_PRECISION = drc::RotationRPYDiff::descriptor()->FindFieldByName("droll")->options().GetExtension(dccl::field).precision();
        const int ROTATION_PITCH_PRECISION = drc::RotationRPYDiff::descriptor()->FindFieldByName("dpitch")->options().GetExtension(dccl::field).precision();
        const int ROTATION_YAW_PRECISION = drc::RotationRPYDiff::descriptor()->FindFieldByName("dyaw")->options().GetExtension(dccl::field).precision();
        
        pos_diff->mutable_rpy_rotation_diff()->add_droll(present_rpy[0] - goby::util::unbiased_round(previous_rpy[0], ROTATION_ROLL_PRECISION));
        pos_diff->mutable_rpy_rotation_diff()->add_dpitch(present_rpy[1] - goby::util::unbiased_round(previous_rpy[1], ROTATION_PITCH_PRECISION));

        
        wrap_minus_pi_to_pi(present_rpy[2]);
        wrap_minus_pi_to_pi(previous_rpy[2]);
        
        pos_diff->mutable_rpy_rotation_diff()->add_dyaw(present_rpy[2] - goby::util::unbiased_round(previous_rpy[2], ROTATION_YAW_PRECISION));


//        std::cout << pos_diff->rpy_rotation_diff().droll(pos_diff->rpy_rotation_diff().droll_size()-1) << "," << pos_diff->rpy_rotation_diff().dpitch(pos_diff->rpy_rotation_diff().dpitch_size()-1) << "," <<  pos_diff->rpy_rotation_diff().dyaw(pos_diff->rpy_rotation_diff().dyaw_size()-1)  << std::endl;

    }
    
    return true;
}

bool RobotStateCodec::from_position3d_diff(drc::Position3D* present_pos,
                                           const drc::Position3DDiff& pos_diff,
                                           int i,
                                           bool use_rpy /* = false */)
{        
    present_pos->mutable_translation()->set_x(
        present_pos->translation().x() + pos_diff.translation_diff().dx(i));
    present_pos->mutable_translation()->set_y(
        present_pos->translation().y() + pos_diff.translation_diff().dy(i));

    if(i < pos_diff.translation_diff().dz_size())
    {
        present_pos->mutable_translation()->set_z(
            present_pos->translation().z() + pos_diff.translation_diff().dz(i));
    }
    else
    {
        present_pos->mutable_translation()->set_z(std::numeric_limits<double>::quiet_NaN());
    }
    
    
    if(!use_rpy)
    {
        present_pos->mutable_rotation()->set_x(
            present_pos->rotation().x() + pos_diff.rotation_diff().dx(i));
        present_pos->mutable_rotation()->set_y(
            present_pos->rotation().y() + pos_diff.rotation_diff().dy(i));
        present_pos->mutable_rotation()->set_z(
            present_pos->rotation().z() + pos_diff.rotation_diff().dz(i));
        present_pos->mutable_rotation()->set_w(
            present_pos->rotation().w() + pos_diff.rotation_diff().dw(i));
    }
    else
    {
        present_pos->mutable_rpy_rotation()->set_roll(present_pos->rpy_rotation().roll() + pos_diff.rpy_rotation_diff().droll(i));
        present_pos->mutable_rpy_rotation()->set_pitch(present_pos->rpy_rotation().pitch() + pos_diff.rpy_rotation_diff().dpitch(i));
        present_pos->mutable_rpy_rotation()->set_yaw(present_pos->rpy_rotation().yaw() + pos_diff.rpy_rotation_diff().dyaw(i));
        
        double q[4] = { 0 };
        double rpy[3] = { 0 };

        rpy[0] = present_pos->rpy_rotation().roll();
        rpy[1] = present_pos->rpy_rotation().pitch();
        rpy[2] = present_pos->rpy_rotation().yaw();
        
        bot_roll_pitch_yaw_to_quat(rpy, q);

        present_pos->mutable_rotation()->set_w(q[0]);
        present_pos->mutable_rotation()->set_x(q[1]);
        present_pos->mutable_rotation()->set_y(q[2]);
        present_pos->mutable_rotation()->set_z(q[3]);
    }
    
    
    return true;
}


    
