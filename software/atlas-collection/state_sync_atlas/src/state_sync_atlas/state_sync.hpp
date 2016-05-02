#ifndef STATE_SYNC_HPP_
#define STATE_SYNC_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>
#include <model-client/model-client.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/atlas/state_extra_t.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>


#include <pronto_utils/pronto_math.hpp>
#include "atlas/AtlasControlTypes.h"
#include "atlas/AtlasJointNames.h"
#include <estimate_tools/simple_kalman_filter.hpp>
#include <estimate_tools/backlash_filter.hpp>
#include <estimate_tools/alpha_filter.hpp>
#include <estimate_tools/torque_adjustment.hpp>

struct Joints { 
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> effort;
  std::vector<std::string> name;
};


/////////////////////////////////////
class CommandLineConfig{
  public:
    CommandLineConfig(){
      // Read from command line:
      standalone_head = false;
      standalone_hand = false;
      bdi_motion_estimate = false;
      simulation_mode = false;
      publish_pose_body = true;
      output_channel = "EST_ROBOT_STATE";
      atlas_version = 5;

      // Defaults - not read from command line:
      use_encoder_joint_sensors = false;
      use_joint_kalman_filter = false;
      use_joint_backlash_filter = false;
      use_rotation_rate_alpha_filter = false;
      use_torque_adjustment = false;
    }
    ~CommandLineConfig(){};

    bool standalone_head, standalone_hand;
    bool bdi_motion_estimate;
    bool simulation_mode;
    bool publish_pose_body;
    std::string output_channel;
    int atlas_version;

    bool use_encoder_joint_sensors;
    bool use_joint_kalman_filter;
    bool use_joint_backlash_filter;
    bool use_rotation_rate_alpha_filter;
    bool use_torque_adjustment;
};

///////////////////////////////////////////////////////////////
class state_sync{
  public:
    state_sync(boost::shared_ptr<lcm::LCM> &lcm_, boost::shared_ptr<CommandLineConfig> &cl_cfg_);
    
    ~state_sync(){
    }
    void Identity();
    
    void setBotParam(BotParam* new_botparam){
      botparam_ = new_botparam;
    }
    void setEncodersFromParam();
    
  private:
    boost::shared_ptr<CommandLineConfig> cl_cfg_;
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<ModelClient> model_;
    BotParam* botparam_;
    
    long utime_prev_;
    
    void coreRobotHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg);
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void leftHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void rightHandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void poseBDIHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseMITHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  atlas::state_extra_t* msg);
    
    void enableEncoderHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::utime_t* msg);
    void enableEncoders(bool enable);
    
    Joints head_joints_;
    Joints core_robot_joints_;
    Joints left_hand_joints_;
    Joints right_hand_joints_;
    Joints core_robot_joints_out_;

    PoseT pose_BDI_;
    PoseT pose_MIT_;
    void setPoseToZero(PoseT &pose);
    
    // Kalman/Backlash Filters for joint angles:
    void filterJoints(int64_t utime, std::vector<float> &joint_position, std::vector<float> &joint_velocity);
    std::vector<int> filter_idx_;
    std::vector<EstimateTools::SimpleKalmanFilter*> joint_kf_;
    std::vector<EstimateTools::BacklashFilter*> joint_backlashfilter_;
    
    // Alpha Filters:
    EstimateTools::AlphaFilter* rotation_rate_alpha_filter_;
    EstimateTools::AlphaFilter* neck_alpha_filter_;
    
    // Torque Adjustment:
    EstimateTools::TorqueAdjustment* torque_adjustment_;
    
    // Upper Body Encoder Calibrations:
    std::vector<float> encoder_joint_offsets_;
    std::vector<float> max_encoder_wrap_angle_;
    std::vector<bool> use_encoder_;
    std::vector<int> encoder_joint_indices_;
    std::vector<int> extra_offsettable_joint_indices_;

    void publishRobotState(int64_t utime_in, const  bot_core::six_axis_force_torque_array_t& msg);
    void appendJoints(bot_core::robot_state_t& msg_out, Joints joints);
    
    bool insertPoseInRobotState(bot_core::robot_state_t& msg, PoseT pose);


    bot_core::six_axis_force_torque_array_t force_torque_; // More recent force torque messurement
    bool force_torque_init_; // Have we received a force torque message?

};    

#endif
