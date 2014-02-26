#ifndef STATE_SYNC_HPP_
#define STATE_SYNC_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>
#include <model-client/model-client.hpp>
#include <drc_utils/joint_utils.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/multisense.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_state_extra_t.hpp"
#include "lcmtypes/drc/hand_state_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/system_status_t.hpp"
#include "lcmtypes/drc/utime_t.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pointcloud_tools/pointcloud_math.hpp>
#include "atlas/AtlasControlTypes.h"
#include "atlas/AtlasJointNames.h"

struct Joints { 
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> effort;
  std::vector<std::string> name;
};

///////////////////////////////////////////////////////////////
class state_sync{
  public:
    state_sync(boost::shared_ptr<lcm::LCM> &lcm_, 
      bool standalone_head_, bool standalone_hand_,
      bool spoof_motion_estimation, bool simulation_mode_,
      bool use_encoder_joint_sensors_, std::string output_channel_);
    
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
    std::string output_channel_;

    long utime_prev_;

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::state_t* msg);
    void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void leftHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void rightHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::hand_state_t* msg);
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseMITHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg);
    void potOffsetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void refreshEncoderCalibrationHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg);
    void loadEncoderOffsetsFromFile();
    void enableEncoderHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg);
    void enableEncoders(bool enable);
    
    Joints head_joints_;
    Joints atlas_joints_;
    Joints left_hand_joints_;
    Joints right_hand_joints_;
    Joints atlas_joints_out_;

    PoseT pose_BDI_;
    PoseT pose_MIT_;
    
    
    // Keep two different offset vectors, for clarity:
    std::vector<float> pot_joint_offsets_;
    std::vector<float> encoder_joint_offsets_;
    std::vector<float> max_encoder_wrap_angle_;
    std::vector<bool> use_encoder_;

    void publishRobotState(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints(drc::robot_state_t& msg_out, Joints joints);
};    

#endif
