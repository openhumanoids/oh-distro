#include "robot-state-codecs.h"
#include "bot_core/bot_core.h"
#include "drc_utils/joint_utils.hpp"
#include "dccl/arithmetic/field_codec_arithmetic.h"

std::map<std::string, int> RobotStateCodec::joint_names_to_order_;
std::vector<std::string> RobotStateCodec::joint_names_;

using goby::glog;
using namespace goby::common::logger;




RobotStateCodec::RobotStateCodec(const std::string loopback_channel, int frequency, bool add_joint_effort, double key_frame_period)
    : CustomChannelCodec(loopback_channel),
      dccl_(goby::acomms::DCCLCodec::get()),
      last_key_time_(0),
      frequency_(frequency),
      add_joint_effort_(add_joint_effort),
      key_frame_period_(key_frame_period)
{
    if(true)
    {
        void* dl_handle = dlopen("libdccl_arithmetic.so", RTLD_LAZY);
        if(!dl_handle)
        {
            std::cerr << "Failed to open libdccl_arithmetic.so" << std::endl;
            exit(1);
        }
        dccl_->load_shared_library_codecs(dl_handle);

        const int JOINT_POS_PRECISION = drc::MinimalRobotState::FullState::descriptor()->FindFieldByName("joint_position")->options().GetExtension(dccl::field).precision();

        const float MAX = 6.284;
        const float MIN = -MAX;    

        std::string model_file_path = getenv ("DRC_BASE");
        model_file_path += "/software/network/src/drc_network_shaper/2015_aggregate_joint_pos_frequencies.csv";
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
    
        dccl::arith::protobuf::ArithmeticModel model;
        
        glog.is(VERBOSE) && glog << "Making joint_pos_ers model" << std::endl;

        std::getline(model_file, line);
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
        
        
        model.set_name("joint_pos_ers");
        glog.is(VERBOSE) && glog << "Setting joint_pos_ers model" << std::endl;
        dccl::arith::ModelManager::set_model(model);        
//        std::cout << pb_to_short_string(model) << std::endl;
    
    }


    dccl_->validate<drc::MinimalRobotState>();

    if(joint_names_.empty())
    {
        const int atlas_version = 5;
        JointUtils utils(atlas_version);

        int i = 0;        
        for(std::vector<std::string>::const_iterator it = utils.all_joint_names.begin(), end = utils.all_joint_names.end(); it != end; ++it)
        {
            joint_names_to_order_.insert(std::make_pair(*it, i++));
        }

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

    if(!to_minimal_state(lcm_object, &dccl_state, false, add_joint_effort_))
        return false;
    

    for(int j = 0, m = dccl_state.full().joint_position_size(); j<m; ++j)
    {
        dccl_state.mutable_full()->set_joint_position(j, goby::util::unbiased_round(dccl_state.full().joint_position(j), JOINT_POS_PRECISION));
    }

    if(dccl_state.utime() > last_key_time_ &&
       tx_key_state_.full().joint_position_size() == dccl_state.full().joint_position_size() &&
       dccl_state.utime() < (last_key_time_ + key_frame_period_*1.0e6))
    {
        // send difference instead
        for(int j = 0, m = dccl_state.full().joint_position_size(); j<m; ++j)
            dccl_state.mutable_diff()->add_joint_position_diff(dccl_state.full().joint_position(j)-tx_key_state_.full().joint_position(j));
        
        tx_key_state_ = dccl_state;
        dccl_state.clear_full();
    }
    else
    {
        last_key_time_ = dccl_state.utime();
        tx_key_state_ = dccl_state;
        glog.is(VERBOSE) && glog << "Last key time: " << last_key_time_ << std::endl;
    }
    
    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_state_debug;
        printer.PrintToString(dccl_state, &dccl_state_debug);
        glog << "MinimalRobotState: " << dccl_state_debug << std::endl;
    }    

    
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

    if(glog.is(VERBOSE))
    {
        google::protobuf::TextFormat::Printer printer;
        printer.SetUseShortRepeatedPrimitives(true);
        std::string dccl_state_debug;
        printer.PrintToString(dccl_state, &dccl_state_debug);
        glog << "MinimalRobotState: " << dccl_state_debug << std::endl;
    } 
    
    if(dccl_state.has_diff()) // diff frame
    {
        goby::uint64 period = (1.0/frequency_)*1e6;
        if(rx_key_state_.utime() + 1.25*period < dccl_state.utime()) // allow 20% of period for slop in sending
        {
            glog.is(WARN) && glog << "Missed EST_ROBOT_STATE frame...waiting for next key!" << std::endl;
            return false;
        }

        for(int j = 0, m = rx_key_state_.full().joint_position_size(); j<m; ++j)
        {
            dccl_state.mutable_full()->add_joint_position(rx_key_state_.full().joint_position(j) + dccl_state.diff().joint_position_diff(j));
        }
        dccl_state.clear_diff();
    }
    rx_key_state_ = dccl_state;
    
    drc::robot_state_t lcm_object;

    if(!from_minimal_state(&lcm_object, dccl_state))
        return false;
    
    lcm_data->resize(lcm_object.getEncodedSize());
    lcm_object.encode(&(*lcm_data)[0], 0, lcm_data->size());

    
    
    return true;
          
}      

bool RobotStateCodec::to_minimal_state(const drc::robot_state_t& lcm_object,
                                       drc::MinimalRobotState* dccl_state,
                                       bool use_rpy /* = false */,
                                       bool add_joint_effort /* = false */)
{
    if(lcm_object.utime != 0)
        dccl_state->set_utime(lcm_object.utime);

    if(!to_minimal_position3d(lcm_object.pose, dccl_state->mutable_pose(), use_rpy))
        return false;


    if(!to_minimal_joint_pos(lcm_object.joint_name,
                             lcm_object.joint_position,
                             dccl_state->mutable_full()->mutable_joint_position(),
                             dccl_state->mutable_full()->mutable_joint_id()))
        return false;



    
    if(add_joint_effort)
    {
        for(int i = 0, n = lcm_object.joint_effort.size(); i < n; ++i)
        {
            // only the arms and back
            if(lcm_object.joint_name[i].find("_arm_") != std::string::npos || lcm_object.joint_name[i].find("back_") != std::string::npos)
            {
                static const double max = drc::MinimalRobotState::descriptor()->FindFieldByName("twice_joint_effort")->options().GetExtension(dccl::field).max();
                static const double min = drc::MinimalRobotState::descriptor()->FindFieldByName("twice_joint_effort")->options().GetExtension(dccl::field).min();

                float twice_joint_effort = lcm_object.joint_effort[i]*2;

                if(twice_joint_effort > max)
                    twice_joint_effort = max;
                if(twice_joint_effort < min)
                    twice_joint_effort = min;
                
                dccl_state->add_twice_joint_effort(twice_joint_effort);
            }
        } 
    }
    

    
    dccl_state->set_l_foot_force_z(lcm_object.force_torque.l_foot_force_z);
    dccl_state->set_l_foot_torque_x(lcm_object.force_torque.l_foot_torque_x);
    dccl_state->set_l_foot_torque_y(lcm_object.force_torque.l_foot_torque_y);

    dccl_state->set_r_foot_force_z(lcm_object.force_torque.r_foot_force_z);
    dccl_state->set_r_foot_torque_x(lcm_object.force_torque.r_foot_torque_x);
    dccl_state->set_r_foot_torque_y(lcm_object.force_torque.r_foot_torque_y);

    dccl_state->add_l_hand_force(lcm_object.force_torque.l_hand_force[0]);
    dccl_state->add_l_hand_force(lcm_object.force_torque.l_hand_force[1]);
    dccl_state->add_l_hand_force(lcm_object.force_torque.l_hand_force[2]);

    dccl_state->add_l_hand_torque(lcm_object.force_torque.l_hand_torque[0]);
    dccl_state->add_l_hand_torque(lcm_object.force_torque.l_hand_torque[1]);
    dccl_state->add_l_hand_torque(lcm_object.force_torque.l_hand_torque[2]);

    dccl_state->add_r_hand_force(lcm_object.force_torque.r_hand_force[0]);
    dccl_state->add_r_hand_force(lcm_object.force_torque.r_hand_force[1]);
    dccl_state->add_r_hand_force(lcm_object.force_torque.r_hand_force[2]);

    dccl_state->add_r_hand_torque(lcm_object.force_torque.r_hand_torque[0]);
    dccl_state->add_r_hand_torque(lcm_object.force_torque.r_hand_torque[1]);
    dccl_state->add_r_hand_torque(lcm_object.force_torque.r_hand_torque[2]);
    
    return true;
}

    
bool RobotStateCodec::from_minimal_state(drc::robot_state_t* lcm_object,
                                         const drc::MinimalRobotState& dccl_state,
                                         bool use_rpy /* = false */)
{

    lcm_object->utime = dccl_state.has_utime() ? dccl_state.utime() : 0;

    if(!from_minimal_position3d(&lcm_object->pose,dccl_state.pose(), use_rpy))
        return false;    
    
    lcm_object->num_joints = dccl_state.full().joint_position_size();

    
    if(!from_minimal_joint_pos(&lcm_object->joint_name,
                               &lcm_object->joint_position,
                               dccl_state.full().joint_position(),
                               dccl_state.full().joint_id()))
        return false;
    
    std::vector<float> joint_zeros;
    joint_zeros.assign ( lcm_object->num_joints,0); 
    lcm_object->twist.linear_velocity.x =0;
    lcm_object->twist.linear_velocity.y =0;
    lcm_object->twist.linear_velocity.z =0;
    lcm_object->twist.angular_velocity.x =0;
    lcm_object->twist.angular_velocity.y =0;
    lcm_object->twist.angular_velocity.z =0;  
    lcm_object->joint_velocity = joint_zeros;

    lcm_object->joint_effort = joint_zeros;

    if(dccl_state.twice_joint_effort_size() != 0)
    {
        int j = 0;
        for(int i = 0, n = lcm_object->joint_name.size(); i < n; ++i)
        { 
            if(lcm_object->joint_name[i].find("_arm_") != std::string::npos || lcm_object->joint_name[i].find("back_") != std::string::npos)
            {
                lcm_object->joint_effort[i] = (float)dccl_state.twice_joint_effort(j++)/2;
                if(j >= dccl_state.twice_joint_effort_size())
                    break;
            }
        }
        
    }
    
    lcm_object->force_torque.l_foot_force_z = dccl_state.l_foot_force_z();
    lcm_object->force_torque.l_foot_torque_x = dccl_state.l_foot_torque_x();
    lcm_object->force_torque.l_foot_torque_y = dccl_state.l_foot_torque_y();

    if(dccl_state.l_hand_torque_size() == 3)
    {
        lcm_object->force_torque.l_hand_torque[0] = dccl_state.l_hand_torque(0);
        lcm_object->force_torque.l_hand_torque[1] = dccl_state.l_hand_torque(1);
        lcm_object->force_torque.l_hand_torque[2] = dccl_state.l_hand_torque(2);
    }
    else
    {
        lcm_object->force_torque.l_hand_torque[0] = 0;
        lcm_object->force_torque.l_hand_torque[1] = 0;
        lcm_object->force_torque.l_hand_torque[2] = 0;    
    }
    
    if(dccl_state.l_hand_force_size() == 3)
    {
        lcm_object->force_torque.l_hand_force[0] = dccl_state.l_hand_force(0);
        lcm_object->force_torque.l_hand_force[1] = dccl_state.l_hand_force(1);
        lcm_object->force_torque.l_hand_force[2] = dccl_state.l_hand_force(2);
    }
    else
    {
        lcm_object->force_torque.l_hand_force[0] = 0;
        lcm_object->force_torque.l_hand_force[1] = 0;
        lcm_object->force_torque.l_hand_force[2] = 0;    
    }

    lcm_object->force_torque.r_foot_force_z = dccl_state.r_foot_force_z();
    lcm_object->force_torque.r_foot_torque_x = dccl_state.r_foot_torque_x();
    lcm_object->force_torque.r_foot_torque_y = dccl_state.r_foot_torque_y();

    if(dccl_state.r_hand_torque_size() == 3)
    {
        lcm_object->force_torque.r_hand_torque[0] = dccl_state.r_hand_torque(0);
        lcm_object->force_torque.r_hand_torque[1] = dccl_state.r_hand_torque(1);
        lcm_object->force_torque.r_hand_torque[2] = dccl_state.r_hand_torque(2);
    }
    else
    {
        lcm_object->force_torque.r_hand_torque[0] = 0;
        lcm_object->force_torque.r_hand_torque[1] = 0;
        lcm_object->force_torque.r_hand_torque[2] = 0;    
    }
    
    if(dccl_state.r_hand_force_size() == 3)
    {
        lcm_object->force_torque.r_hand_force[0] = dccl_state.r_hand_force(0);
        lcm_object->force_torque.r_hand_force[1] = dccl_state.r_hand_force(1);
        lcm_object->force_torque.r_hand_force[2] = dccl_state.r_hand_force(2);
    }
    else
    {
        lcm_object->force_torque.r_hand_force[0] = 0;
        lcm_object->force_torque.r_hand_force[1] = 0;
        lcm_object->force_torque.r_hand_force[2] = 0;    
    }
    
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


    
