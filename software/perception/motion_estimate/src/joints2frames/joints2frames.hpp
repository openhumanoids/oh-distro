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

#include <pointcloud_tools/pointcloud_math.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

//#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"


///////////////////////////////////////////////////////////////
class joints2frames{
  public:
    joints2frames(boost::shared_ptr<lcm::LCM> &publish_lcm, bool show_labels_, bool show_triads_);
    
    ~joints2frames(){
    }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    pointcloud_vis* pc_vis_;

    Isometry3dTime world_to_bodyT_;
    Isometry3dTime body_to_headT_;

    void urdf_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    bool show_labels_, show_triads_;

    // Solver: [from sisir]
    std::string _robot_name;
    std::string _urdf_xml_string;
    std::string _head_link_name;
    std::vector<std::string> _joint_names_;    
    std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
    
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    bool _urdf_parsed;    
    std::vector<drc::link_transform_t> _link_tfs;
    
    Eigen::Quaterniond  euler_to_quat(double yaw, double pitch, double roll);
};    

#endif
