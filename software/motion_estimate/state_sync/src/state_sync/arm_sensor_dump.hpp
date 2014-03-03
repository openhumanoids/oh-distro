#ifndef STATE_SYNC_HPP_
#define STATE_SYNC_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/multisense.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>

struct Joints { 
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> effort;
  std::vector<std::string> name;
};

// Equivalent to bot_core_pose contents
struct PoseT { 
  int64_t utime;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector4d orientation;
  Eigen::Vector3d rotation_rate;
  Eigen::Vector3d accel;
};



///////////////////////////////////////////////////////////////
class state_sync{
  public:
    state_sync(boost::shared_ptr<lcm::LCM> &lcm_, bool standalone_head_,
      bool spoof_motion_estimation, bool simulation_mode_,
      bool use_transmission_joint_sensors_);
    
    ~state_sync(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool standalone_head_;
    bool bdi_motion_estimate_;
    bool simulation_mode_;
    bool use_transmission_joint_sensors_;

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg);
    void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void sandiaLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void sandiaRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void irobotLeftHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void irobotRightHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg);
    void offsetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    
    Joints head_joints_;
    Joints atlas_joints_;
    Joints sandia_left_joints_;
    Joints sandia_right_joints_;
    Joints irobot_left_joints_;
    Joints irobot_right_joints_;
    PoseT pose_BDI_;
    Joints atlas_joints_out_;
    std::vector<float> manual_joint_offsets_;

    // Returns false if Pose BDI is old or hasn't appeared yet
    bool insertPoseBDI( drc::robot_state_t& msg);    
    void publishRobotState(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints(drc::robot_state_t& msg_out, Joints joints);

    bool is_sandia_left_;
    bool is_sandia_right_;    
};    

#endif
