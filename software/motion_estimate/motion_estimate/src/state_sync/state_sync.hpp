#ifndef STATE_SYNC_HPP_
#define STATE_SYNC_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>
#include <model-client/model-client.hpp>
#include <drc_utils/joint_utils.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/multisense.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "atlas/AtlasControlTypes.h"
#include "atlas/AtlasJointNames.h"

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
    state_sync(boost::shared_ptr<lcm::LCM> &lcm_, 
      bool standalone_head_, bool standalone_hand_,
      bool spoof_motion_estimation, bool simulation_mode_,
      bool use_encoder_joint_sensors_);
    
    ~state_sync(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    JointUtils joint_utils_;
    
    bool standalone_head_, standalone_hand_;
    bool bdi_motion_estimate_;
    bool simulation_mode_;
    bool use_encoder_joint_sensors_;

    long utime_prev_;

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg);
    void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void leftHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void rightHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg);
    void potOffsetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void refreshEncoderCalibrationHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg);
    void loadEncoderOffsetsFromFile();
    
    Joints head_joints_;
    Joints atlas_joints_;
    Joints left_hand_joints_;
    Joints right_hand_joints_;
    PoseT pose_BDI_;
    Joints atlas_joints_out_;

    // Keep two different offset vectors, for clarity:
    std::vector<float> pot_joint_offsets_;
    std::vector<float> encoder_joint_offsets_;
    std::vector<float> max_encoder_wrap_angle_;
    std::vector<bool> use_encoder_;

    // Returns false if Pose BDI is old or hasn't appeared yet
    bool insertPoseBDI( drc::robot_state_t& msg);    
    void publishRobotState(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints(drc::robot_state_t& msg_out, Joints joints);
};    

#endif
