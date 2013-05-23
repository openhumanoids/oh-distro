#include "robot-state-codecs.h"

std::map<std::string, int> RobotStateCodec::joint_names_to_order_;
std::vector<std::string> RobotStateCodec::joint_names_;

RobotStateCodec::RobotStateCodec()
    : dccl_(goby::acomms::DCCLCodec::get())
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
    using goby::glog;
    using namespace goby::common::logger;
    
    drc::robot_state_t lcm_object;
    if(lcm_object.decode(&lcm_data[0], 0, lcm_data.size()) == -1)
        return false;

    drc::MinimalRobotState dccl_state;

    dccl_state.set_utime(lcm_object.utime);
    dccl_state.mutable_origin_position()->mutable_translation()->set_x(
        lcm_object.origin_position.translation.x);
    dccl_state.mutable_origin_position()->mutable_translation()->set_y(
        lcm_object.origin_position.translation.y);
    dccl_state.mutable_origin_position()->mutable_translation()->set_z(
        lcm_object.origin_position.translation.z);

    
    dccl_state.mutable_origin_position()->mutable_rotation()->set_x(
        lcm_object.origin_position.rotation.x);
    dccl_state.mutable_origin_position()->mutable_rotation()->set_y(
        lcm_object.origin_position.rotation.y);
    dccl_state.mutable_origin_position()->mutable_rotation()->set_z(
        lcm_object.origin_position.rotation.z);
    dccl_state.mutable_origin_position()->mutable_rotation()->set_w(
        lcm_object.origin_position.rotation.w);

    for(int i = 0, n = lcm_object.joint_position.size(); i < n; ++i)
        dccl_state.add_joint_position(std::numeric_limits<float>::quiet_NaN());
    
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
        

        dccl_state.set_joint_position(order->second, position);
    }

    glog.is(VERBOSE) && glog << "MinimalRobotState: " << dccl_state.ShortDebugString() << std::endl;
    
    std::string encoded;
    dccl_->encode(&encoded, dccl_state);
    transmit_data->resize(encoded.size());
    std::copy(encoded.begin(), encoded.end(), transmit_data->begin());
    return true;
}

      
bool RobotStateCodec::decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data)
{
    using goby::glog;
    using namespace goby::common::logger;

    std::string encoded(transmit_data.begin(), transmit_data.end());
    drc::MinimalRobotState dccl_state;

    dccl_->decode(encoded, &dccl_state);
    
    drc::robot_state_t lcm_object;
    lcm_object.utime = dccl_state.utime();
    lcm_object.robot_name = "atlas";

    lcm_object.origin_position.translation.x = dccl_state.origin_position().translation().x();
    lcm_object.origin_position.translation.y = dccl_state.origin_position().translation().y();
    lcm_object.origin_position.translation.z = dccl_state.origin_position().translation().z();

    // renormalize rotation quat
    const drc::RotationQuaternion& rotation = dccl_state.origin_position().rotation();
    double length = std::sqrt(rotation.x()*rotation.x() +
                       rotation.y()*rotation.y() +
                       rotation.z()*rotation.z() +
                       rotation.w()*rotation.w());

    lcm_object.origin_position.rotation.x = rotation.x() / length;
    lcm_object.origin_position.rotation.y = rotation.y() / length;
    lcm_object.origin_position.rotation.z = rotation.z() / length;
    lcm_object.origin_position.rotation.w = rotation.w() / length;

    lcm_object.num_joints = dccl_state.joint_position_size();
    lcm_object.joint_name.resize(dccl_state.joint_position_size());
    std::copy(joint_names_.begin(), joint_names_.begin() + dccl_state.joint_position_size(), lcm_object.joint_name.begin());
    
    for(int i = 0, n = dccl_state.joint_position_size(); i < n; ++i)
        lcm_object.joint_position.push_back(dccl_state.joint_position(i));

    std::vector<float> joint_zeros;
    joint_zeros.assign ( joint_names_.size(),0); 
    drc::joint_covariance_t j_cov;
    j_cov.variance = 0;
    std::vector<drc::joint_covariance_t> joint_cov_zeros;
    joint_cov_zeros.assign ( joint_names_.size(),j_cov);  
    lcm_object.origin_twist.linear_velocity.x =0;
    lcm_object.origin_twist.linear_velocity.y =0;
    lcm_object.origin_twist.linear_velocity.z =0;
    lcm_object.origin_twist.angular_velocity.x =0;
    lcm_object.origin_twist.angular_velocity.y =0;
    lcm_object.origin_twist.angular_velocity.z =0;  
    for(int i = 0; i < 6; i++)  {
        for(int j = 0; j < 6; j++) {
            lcm_object.origin_cov.position_cov[i][j] = 0;
            lcm_object.origin_cov.twist_cov[i][j] = 0;
        }
    }
    lcm_object.joint_velocity = joint_zeros;
    lcm_object.measured_effort = joint_zeros;
    lcm_object.joint_cov = joint_cov_zeros;

    drc::contact_state_t cs;
    cs.num_contacts = 0;
    lcm_object.contacts = cs;

    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());
    
    return true;
          
}      
