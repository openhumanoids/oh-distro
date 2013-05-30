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
    
    
    static std::map<std::string, int> joint_names_to_order_;
    static std::vector<std::string> joint_names_;
    
  private:
    goby::acomms::DCCLCodec* dccl_;
        
        
};





#endif
