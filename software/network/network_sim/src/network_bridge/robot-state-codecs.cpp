#include "robot-state-codecs.h"

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
                                              drc::MinimalRobotState* dccl_state)
{
    dccl_state->set_utime(lcm_object.utime);

    if(!to_minimal_position3d(lcm_object.origin_position, dccl_state->mutable_origin_position()))
        return false;

    for(int i = 0, n = lcm_object.joint_position.size(); i < n; ++i)
        dccl_state->add_joint_position(std::numeric_limits<float>::quiet_NaN());
    
    for(int i = 0, n = lcm_object.joint_position.size(); i < n; ++i)
    {
        std::map<std::string, int>::const_iterator order =
            joint_names_to_order_.find(lcm_object.joint_name[i]);

        if(order == joint_names_to_order_.end())
        {
            glog.is(WARN) && glog << "No joint called [" << lcm_object.joint_name[i] << "] found in hard-coded map." << std::endl;
            return false;
        }

        double position = lcm_object.joint_position[i];
        const double pi = 3.14159; 
        while(position >= pi) position -= 2*pi;
        while(position < -pi) position += 2*pi;
        

        dccl_state->set_joint_position(order->second, position);
    }
    return true;
}

    
bool RobotStateCodec::from_minimal_state(drc::robot_state_t* lcm_object,
                                         const drc::MinimalRobotState& dccl_state)
{

    lcm_object->utime = dccl_state.utime();
    lcm_object->robot_name = "atlas";

    if(!from_minimal_position3d(&lcm_object->origin_position,dccl_state.origin_position()))
        return false;    
    
    lcm_object->num_joints = dccl_state.joint_position_size();
    lcm_object->joint_name.resize(dccl_state.joint_position_size());
    std::copy(joint_names_.begin(), joint_names_.begin() + dccl_state.joint_position_size(), lcm_object->joint_name.begin());
    
    for(int i = 0, n = dccl_state.joint_position_size(); i < n; ++i)
        lcm_object->joint_position.push_back(dccl_state.joint_position(i));

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
                           drc::Position3D* dccl_pos)
{
    dccl_pos->mutable_translation()->set_x(lcm_pos.translation.x);
    dccl_pos->mutable_translation()->set_y(lcm_pos.translation.y);
    dccl_pos->mutable_translation()->set_z(lcm_pos.translation.z);
    
    dccl_pos->mutable_rotation()->set_x(lcm_pos.rotation.x);
    dccl_pos->mutable_rotation()->set_y(lcm_pos.rotation.y);
    dccl_pos->mutable_rotation()->set_z(lcm_pos.rotation.z);
    dccl_pos->mutable_rotation()->set_w(lcm_pos.rotation.w);

    return true;
}


bool RobotStateCodec::from_minimal_position3d(drc::position_3d_t* lcm_pos,
                             const drc::Position3D& dccl_pos)
{
    lcm_pos->translation.x = dccl_pos.translation().x();
    lcm_pos->translation.y = dccl_pos.translation().y();
    lcm_pos->translation.z = dccl_pos.translation().z();

    // renormalize rotation quat
    const drc::RotationQuaternion& rotation = dccl_pos.rotation();
    lcm_pos->rotation.x = rotation.x();
    lcm_pos->rotation.y = rotation.y();
    lcm_pos->rotation.z = rotation.z();
    lcm_pos->rotation.w = rotation.w();

    quaternion_normalize(lcm_pos->rotation);    

    return true;
}


bool RobotStateCodec::to_position3d_diff(const drc::position_3d_t& present_pos,
                                         const drc::position_3d_t& previous_pos,
                                         drc::Position3DDiff* pos_diff)
{

    // pre-round the values to avoid cumulative rounding errors
    const int TRANSLATION_X_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dx")->options().GetExtension(dccl::field).precision();
    const int TRANSLATION_Y_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dy")->options().GetExtension(dccl::field).precision();
    const int TRANSLATION_Z_PRECISION = drc::TranslationVectorDiff::descriptor()->FindFieldByName("dz")->options().GetExtension(dccl::field).precision();
           
    pos_diff->mutable_translation_diff()->add_dx(present_pos.translation.x-
        goby::util::unbiased_round(previous_pos.translation.x, TRANSLATION_X_PRECISION));
    pos_diff->mutable_translation_diff()->add_dy(present_pos.translation.y-
        goby::util::unbiased_round(previous_pos.translation.y, TRANSLATION_Y_PRECISION));
    pos_diff->mutable_translation_diff()->add_dz(present_pos.translation.z-
        goby::util::unbiased_round(previous_pos.translation.z, TRANSLATION_Z_PRECISION));

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
    
    return true;
}

bool RobotStateCodec::from_position3d_diff(drc::Position3D* present_pos,
                                           const drc::Position3DDiff& pos_diff,
                                           int i)
{        
    present_pos->mutable_translation()->set_x(
        present_pos->translation().x() + pos_diff.translation_diff().dx(i));
    present_pos->mutable_translation()->set_y(
        present_pos->translation().y() + pos_diff.translation_diff().dy(i));
    present_pos->mutable_translation()->set_z(
        present_pos->translation().z() + pos_diff.translation_diff().dz(i));
        
    present_pos->mutable_rotation()->set_x(
        present_pos->rotation().x() + pos_diff.rotation_diff().dx(i));
    present_pos->mutable_rotation()->set_y(
        present_pos->rotation().y() + pos_diff.rotation_diff().dy(i));
    present_pos->mutable_rotation()->set_z(
        present_pos->rotation().z() + pos_diff.rotation_diff().dz(i));
    present_pos->mutable_rotation()->set_w(
        present_pos->rotation().w() + pos_diff.rotation_diff().dw(i));

    
    return true;
}

