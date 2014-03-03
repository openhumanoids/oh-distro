#ifndef JOINTS2FRAMES_HPP_
#define JOINTS2FRAMES_HPP_

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

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <pointcloud_tools/pointcloud_math.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc/atlas_foot_pos_est_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"

struct FrequencyLimit {
  FrequencyLimit(){}
  FrequencyLimit(int64_t last_utime , int64_t min_period):
      last_utime(last_utime),min_period(min_period){}
  
  int64_t last_utime; // utime of last published message
  int64_t min_period; // minimum time between publishes
};

///////////////////////////////////////////////////////////////
class joints2frames{
  public:
    joints2frames(boost::shared_ptr<lcm::LCM> &lcm_, bool show_labels_, 
                  bool show_triads_, bool standalone_head_, bool show_ground_image_,
                  bool bdi_motion_estimate_);
    
    ~joints2frames(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    pointcloud_vis* pc_vis_;
    
    map<string, FrequencyLimit > pub_frequency_;
    bool show_labels_, show_triads_, ground_height_;
    bool standalone_head_, bdi_motion_estimate_;

    void urdf_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void foot_pos_est_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_foot_pos_est_t* msg);
    
    void publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel);
    void publishRigidTransform(Eigen::Isometry3d pose, int64_t utime, std::string channel);
    bool shouldPublish(int64_t utime, std::string channel); 
    
    double getMaxFrequency(std::string query_root);
};    

#endif
