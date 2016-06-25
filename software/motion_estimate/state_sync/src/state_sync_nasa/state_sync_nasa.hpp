#ifndef STATE_SYNC_NASA_HPP_
#define STATE_SYNC_NASA_HPP_

#include <lcm/lcm-cpp.hpp>

#include <map>
#include <memory>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/system_status_t.hpp"
#include "lcmtypes/bot_core/utime_t.hpp"
#include "lcmtypes/bot_core/six_axis_force_torque_array_t.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <model-client/model-client.hpp>
#include <drake/systems/plants/RigidBodyTree.h>

#include <pronto_utils/pronto_math.hpp>
#include <estimate_tools/torque_adjustment.hpp>
#include <estimate_tools/alpha_filter.hpp>

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
      output_channel = "EST_ROBOT_STATE";
      use_ihmc = false;
      pin_floating_base = false;

      // Defaults - not read from command line:
      use_torque_adjustment = false;
      use_joint_velocity_low_pass = false;
    }
    ~CommandLineConfig(){};

    std::string output_channel;
    bool use_ihmc;
    bool use_torque_adjustment;
    bool use_joint_velocity_low_pass;
    bool pin_floating_base;

};

///////////////////////////////////////////////////////////////
class state_sync_nasa{
  public:
    state_sync_nasa(std::shared_ptr<lcm::LCM> &lcm_, std::shared_ptr<CommandLineConfig> &cl_cfg_);
    
    ~state_sync_nasa(){
    }
    void Identity();
    
    void setBotParam(BotParam* new_botparam){
      botparam_ = new_botparam;
    }

    std::vector<std::string> joints_to_be_clamped_to_joint_limits_;
    double clamping_tolerance_in_degrees_;
    
  private:
    std::shared_ptr<CommandLineConfig> cl_cfg_;
    std::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    std::map<std::string, double> joint_limit_min_;
    std::map<std::string, double> joint_limit_max_;
        
    void coreRobotHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg);
    void poseIHMCHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void neckStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);

    Joints core_robot_joints_;

    PoseT pose_ihmc_;
    PoseT pose_pronto_;
    void setPoseToZero(PoseT &pose);
           
    // Torque Adjustment:
    EstimateTools::TorqueAdjustment* torque_adjustment_;
    // joint velocity filter
    std::shared_ptr<EstimateTools::AlphaFilter> joint_vel_filter_;
    Eigen::VectorXd raw_vel_, filtered_vel_;

    void publishRobotState(int64_t utime_in, const  bot_core::six_axis_force_torque_array_t& msg);
    void appendJoints(bot_core::robot_state_t& msg_out, Joints joints);
    float clampJointToJointLimits(std::string joint_name, float joint_position);
    
    bool insertPoseInRobotState(bot_core::robot_state_t& msg, PoseT pose);


    bot_core::six_axis_force_torque_array_t force_torque_; // More recent force torque messurement
    bool force_torque_init_; // Have we received a force torque message?

};    

#endif
