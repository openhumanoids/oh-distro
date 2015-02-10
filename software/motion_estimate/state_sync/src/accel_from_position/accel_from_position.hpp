#ifndef accel_from_position_HPP_
#define accel_from_position_HPP_

#include <lcm/lcm-cpp.hpp>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/std/vector.hpp>

#include <map>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pronto_utils/pronto_math.hpp>
#include <pronto_utils/pronto_lcm.hpp>
#include <pronto_utils/pronto_vis.hpp>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc/atlas_foot_pos_est_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"
#include "lcmtypes/microstrain/ins_t.hpp"

///////////////////////////////////////////////////////////////
class accel_from_position{
  public:
    accel_from_position(boost::shared_ptr<lcm::LCM> &lcm_, bool show_labels_, 
                  bool show_triads_, bool standalone_head_, bool show_ground_image_,
                  bool bdi_motion_estimate_, bool multisense_sim_);
    
    ~accel_from_position(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;


    BotFrames* botframes_;
    bot::frames* botframes_cpp_;   

    pronto_vis* pc_vis_;
    
    bool show_labels_, show_triads_, ground_height_;
    bool standalone_head_, bdi_motion_estimate_;
    bool multisense_sim_;

    void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);
    void publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel);

    Eigen::Isometry3d imu_to_local_prev_;
    Eigen::Vector3d imu_to_local_vel_prev_;
    int64_t utime_prev_;
};    

#endif
