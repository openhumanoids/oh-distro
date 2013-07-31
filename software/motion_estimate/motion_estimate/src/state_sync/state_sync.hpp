#ifndef STATE_SYNC_HPP_
#define STATE_SYNC_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>

#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/multisense.hpp"

struct Joints { 
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> effort;
  std::vector<std::string> name;
};


///////////////////////////////////////////////////////////////
class state_sync{
  public:
    state_sync(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~state_sync(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg);
    void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void sandiaLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sandia_state_t* msg);
    void sandiaRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::sandia_state_t* msg);

    void publishRobotState(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints(drc::state_t& msg_out, Joints joints);
    
    
    void publishRobotState_VRC(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints_VRC(drc::robot_state_t& msg_out, Joints joints);
    drc::contact_state_t setContacts_VRC(const  drc::force_torque_t& msg);

    Joints head_joints_;
    Joints atlas_joints_;
    Joints sandia_left_joints_;
    Joints sandia_right_joints_;
    
};    

#endif
