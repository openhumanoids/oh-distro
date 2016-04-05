#ifndef STATE_SYNC_NASA_HPP_
#define STATE_SYNC_NASA_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <map>
#include <model-client/model-client.hpp>

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


#include <pronto_utils/pronto_math.hpp>
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
      output_channel = "EST_ROBOT_STATE";

      // Defaults - not read from command line:
      use_torque_adjustment = false;

      // Mode - switches between CORE_ROBOT_STATE (NASA) and VAL_CORE_ROBOT_STATE (IHMC) as source
      mode = "ihmc";
    }
    ~CommandLineConfig(){};

    std::string output_channel;

    bool use_torque_adjustment;

    std::string mode;
};

///////////////////////////////////////////////////////////////
class state_sync_nasa{
  public:
    state_sync_nasa(boost::shared_ptr<lcm::LCM> &lcm_, boost::shared_ptr<CommandLineConfig> &cl_cfg_);
    
    ~state_sync_nasa(){
    }
    void Identity();
    
    void setBotParam(BotParam* new_botparam){
      botparam_ = new_botparam;
    }
    
  private:
    boost::shared_ptr<CommandLineConfig> cl_cfg_;
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;
        
    void coreRobotHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);
    void forceTorqueHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::six_axis_force_torque_array_t* msg);
    void poseIHMCHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void poseProntoHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    void neckStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::joint_state_t* msg);

    Joints core_robot_joints_;

    PoseT pose_IHMC_;
    PoseT pose_Pronto_;
    void setPoseToZero(PoseT &pose);
           
    // Torque Adjustment:
    EstimateTools::TorqueAdjustment* torque_adjustment_;

    void publishRobotState(int64_t utime_in, const  bot_core::six_axis_force_torque_array_t& msg);
    void appendJoints(bot_core::robot_state_t& msg_out, Joints joints);
    
    bool insertPoseInRobotState(bot_core::robot_state_t& msg, PoseT pose);


    bot_core::six_axis_force_torque_array_t force_torque_; // More recent force torque messurement
    bool force_torque_init_; // Have we received a force torque message?

};    

#endif
