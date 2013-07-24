#ifndef STATE_SYNC_HPP_
#define STATE_SYNC_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>

#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/multisense.hpp"

struct Joints { 
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<std::string> name; // temporary - remove name from message eventually
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

    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg);

    void publishRobotState(int64_t utime_in);
    void appendJoints(drc::robot_state_t& msg_out, Joints joints);

    Joints head_joints_;
    std::vector<std::string> head_names_;
};    

#endif
