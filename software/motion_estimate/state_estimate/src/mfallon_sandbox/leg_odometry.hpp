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

#include <leg-odometry/FootContact.h>

struct CommandLineConfig
{
    std::string config_filename;
    std::string urdf_filename;
    std::string lcmlog_filename;
    bool read_lcmlog;
    int64_t begin_timestamp;
    int64_t end_timestamp;
};


///////////////////////////////////////////////////////////////
class leg_odometry{
  public:
    leg_odometry(boost::shared_ptr<lcm::LCM> &lcm_subscribe_, boost::shared_ptr<lcm::LCM> &lcm_publish_, const CommandLineConfig& cl_cfg_);
    
    ~leg_odometry(){
    }
    
    void Update(const  drc::robot_state_t* msg);
    void terminate();    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_subscribe_, lcm_publish_;
    const CommandLineConfig cl_cfg_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    pointcloud_vis* pc_vis_;
    
    // params:
    std::string leg_odometry_mode_;
    
   
    TwoLegs::FootContact* foot_contact_logic_;
    
    void initializePose(int mode,Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot);
    // Pure Leg Odometry, no IMU
    void leg_odometry_basic(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status);
    // At the moment a foot transition occurs: slave the pelvis pitch and roll and then fix foot using fk.
    // Dont move or rotate foot after that.
    void leg_odometry_gravity_slaved_once(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status);
    // Foot position, as with above. For subsequent ticks, foot quaternion is updated using the pelvis quaternion
    // The pelvis position is then backed out using this new foot positon and fk.
    void leg_odometry_gravity_slaved_always(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, int contact_status);
    
    // Position Estimate produced by BDI:
    Eigen::Isometry3d world_to_body_bdi_;
    // has the leg odometry been initialized
    bool leg_odo_init_;
    Eigen::Isometry3d world_to_body_;
    int primary_foot_;
    Eigen::Isometry3d world_to_fixed_primary_foot_;
    Eigen::Isometry3d world_to_secondary_foot_;
    
    Eigen::Isometry3d previous_body_to_l_foot_;
    Eigen::Isometry3d previous_body_to_r_foot_;
    
    
    // Utilities and Logging
    int verbose_;
    void openLogFile();
    ofstream logfile_;
};    

#endif
