#ifndef ROBOTSTATECODECS20130514H
#define ROBOTSTATECODECS20130514H

#include "custom-codecs.h"

#include "lcmtypes/drc/robot_state_t.hpp"

#include "robot-state-analogs.pb.h"

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
    RobotStateCodec(const std::string loopback_channel = "");
        
    bool encode(const std::vector<unsigned char>& lcm_data, std::vector<unsigned char>* transmit_data);
      
    bool decode(std::vector<unsigned char>* lcm_data, const std::vector<unsigned char>& transmit_data);

    static bool to_minimal_state(const drc::robot_state_t& lcm_object,
                                 drc::MinimalRobotState* dccl_state);
    static bool from_minimal_state(drc::robot_state_t* lcm_object,
                                   const drc::MinimalRobotState& dccl_state);

    static bool to_minimal_position3d(const drc::position_3d_t& lcm_pos,
                                      drc::Position3D* dccl_pos);
    static bool from_minimal_position3d(drc::position_3d_t* lcm_pos,
                                        const drc::Position3D& dccl_pos);    


    static bool to_position3d_diff(const drc::position_3d_t& present_pos,
                                   const drc::position_3d_t& previous_pos,
                                   drc::Position3DDiff* pos_diff);
    static bool from_position3d_diff(drc::Position3D* pos,
                                     const drc::Position3DDiff& pos_diff,
                                     int index);

    // offset is the starting point of joint names in the master list of all 53
    template<class D1, class D2>
        static bool to_minimal_joint_pos(const std::vector<std::string>& joint_names,
                                         const std::vector<D1>& joint_pos,
                                         google::protobuf::RepeatedField<D2>* dccl_joint_pos,
                                         int offset = 0)
    {
        
        using goby::glog;
        using namespace goby::common::logger;

        for(int i = 0, n = joint_pos.size(); i < n; ++i)
            dccl_joint_pos->Add(std::numeric_limits<float>::quiet_NaN());
    
        for(int i = 0, n = joint_pos.size(); i < n; ++i)
        {
            std::map<std::string, int>::const_iterator order =
                joint_names_to_order_.find(joint_names[i]);

            if(order == joint_names_to_order_.end())
            {
                glog.is(WARN) && glog << "No joint called [" << joint_names[i] << "] found in hard-coded map." << std::endl;
                return false;
            }

            double position = joint_pos[i];
            const double pi = 3.14159; 
            while(position >= pi) position -= 2*pi;
            while(position < -pi) position += 2*pi;
        

            dccl_joint_pos->Set(order->second - offset, position);
        }
    
        return true;
    }
    
    template<class D1, class D2>
        static bool from_minimal_joint_pos(std::vector<std::string>* joint_names,
                                           std::vector<D1>* joint_pos,
                                           const google::protobuf::RepeatedField<D2>& dccl_joint_pos,
                                           int offset = 0)
    {

        joint_names->resize(dccl_joint_pos.size());
        std::copy(joint_names_.begin() + offset, joint_names_.begin() + dccl_joint_pos.size() + offset, joint_names->begin());
    
        for(int i = 0, n = dccl_joint_pos.size(); i < n; ++i)
            joint_pos->push_back(dccl_joint_pos.Get(i));

        return true;

    }
    
    
    static std::map<std::string, int> joint_names_to_order_;
    static std::vector<std::string> joint_names_;
    
  private:
    goby::acomms::DCCLCodec* dccl_;
        
        
};





#endif
