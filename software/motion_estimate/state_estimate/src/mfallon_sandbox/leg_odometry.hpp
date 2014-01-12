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

//#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"


#include "foot_contact.hpp"

///////////////////////////////////////////////////////////////
class leg_odometry{
  public:
    leg_odometry(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~leg_odometry(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    pointcloud_vis* pc_vis_;
    
    void foot_contact_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::foot_contact_estimate_t* msg);
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void foot_pos_est_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_foot_pos_est_t* msg);
    void publishPose(Eigen::Isometry3d pose, int64_t utime, std::string channel);
    
    foot_contact* foot_contact_;
    bool last_left_contact_, last_right_contact_;
    // has the leg odometry been initialized
    bool leg_odo_init_;
    Eigen::Isometry3d world_to_body_;
    int primary_foot_;
    Eigen::Isometry3d world_to_fixed_primary_foot_;
    Eigen::Isometry3d world_to_secondary_foot_;
};    

#endif
