#ifndef ROBOTSTATECODECS20130514H
#define ROBOTSTATECODECS20130514H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_state_t.hpp"

#include "robot-state-analogs.pb.h"

enum { JOINT_POS_PRECISION = 3 };

inline void quaternion_normalize(drc::quaternion_t& q)
{
    double length = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);

    q.x = q.x / length;
    q.y = q.y / length;
    q.z = q.z / length;
    q.w = q.w / length;    
}


class RobotStateCodec : public CustomChannelCodec
{
  public:
    RobotStateCodec(const std::string loopback_channel, int frequency, bool add_joint_effort);
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);

    static bool to_minimal_state(const drc::robot_state_t& lcm_object,
                                 drc::MinimalRobotState* dccl_state,
                                 bool use_rpy = false,
                                 bool add_joint_effort = false);

    static bool from_minimal_state(drc::robot_state_t* lcm_object,
                                   const drc::MinimalRobotState& dccl_state,
                                   bool use_rpy = false);

    static bool to_minimal_position3d(const drc::position_3d_t& lcm_pos,
                                      drc::Position3D* dccl_pos, bool use_rpy = false,
                                      bool omit_z = false);
    static bool from_minimal_position3d(drc::position_3d_t* lcm_pos,
                                        const drc::Position3D& dccl_pos, bool use_rpy = false);    


    static bool to_position3d_diff(const drc::position_3d_t& present_pos,
                                   const drc::position_3d_t& previous_pos,
                                   drc::Position3DDiff* pos_diff,
                                   bool use_rpy = false,
                                   bool omit_z = false);
    static bool from_position3d_diff(drc::Position3D* pos,
                                     const drc::Position3DDiff& pos_diff,
                                     int index,
                                     bool use_rpy = false);

    // offset is the starting point of joint names in the master list of all 53
    template<class D1, class D2>
        static bool to_minimal_joint_pos(const std::vector<std::string>& joint_names,
                                         const std::vector<D1>& joint_pos,
                                         google::protobuf::RepeatedField<D2>* dccl_joint_pos,
                                         google::protobuf::RepeatedField<int>* dccl_joint_id,
                                         int offset = 0)
    {
        
        using goby::glog;
        using namespace goby::common::logger;

        bool joints_in_order = true;
        for(int i = 0, n = joint_pos.size(); i < n; ++i)
        {
            
            std::map<std::string, int>::const_iterator order =
                joint_names_to_order_.find(joint_names[i]);

            if(order->second != i)
            {
                if(joints_in_order)
                    glog.is(VERBOSE) && glog << "Joint " << joint_names[i] << " out of order in joint_utils.hpp. Expecting: " << i << ", got " << order->second << std::endl;
                joints_in_order = false;
            }

            double position = joint_pos[i];

//            std::cout << "joint: " << joint_names[i] <<  " pos: " << position << " index: " << order->second - offset << std::endl;
            
            dccl_joint_pos->Add(position);
            dccl_joint_id->Add(order->second);
        }

        if(joints_in_order)
            dccl_joint_id->Clear(); // no need to waste space
        
        return true;
    }
    
    template<class D1, class D2>
        static bool from_minimal_joint_pos(std::vector<std::string>* joint_names,
                                           std::vector<D1>* joint_pos,
                                           const google::protobuf::RepeatedField<D2>& dccl_joint_pos,
                                           const google::protobuf::RepeatedField<int>& dccl_joint_id,
                                           int offset = 0)
    {

        for(int i = 0, n = dccl_joint_pos.size(); i < n; ++i)
        {
            
            if(dccl_joint_id.size()) // out of order
                joint_names->push_back(joint_names_.at(dccl_joint_id.Get(i)));
            else // in order
                joint_names->push_back(joint_names_.at(i));
            
            joint_pos->push_back(dccl_joint_pos.Get(i));
        }

        return true;

    }

    static void wrap_minus_pi_to_pi(double& angle)
    {
        const double pi = 3.142;
        while(angle >= pi) angle -= 2*pi;
        while(angle < -pi) angle += 2*pi;
    }
    
    
    static std::map<std::string, int> joint_names_to_order_;
    static std::vector<std::string> joint_names_;
    
  private:
    goby::acomms::DCCLCodec* dccl_;
    drc::MinimalRobotState tx_key_state_, rx_key_state_;
    goby::uint64 last_key_time_;
    int frequency_;
    bool add_joint_effort_;
    
    enum { KEY_FRAME_PERIOD = 5 };
};





#endif
