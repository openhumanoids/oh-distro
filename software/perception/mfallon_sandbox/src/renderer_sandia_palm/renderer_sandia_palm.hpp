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

#include <pointcloud_tools/pointcloud_math.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

//#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>

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
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;

    pointcloud_vis* pc_vis_;

    void urdf_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
    void foot_pos_est_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_foot_pos_est_t* msg);
    
    
    bool show_labels_, show_triads_, ground_height_;

    bool standalone_head_, bdi_motion_estimate_;
    
    double last_ground_height_;
    int64_t last_ground_publish_utime_;
    int64_t last_joint_publish_utime_;
    
    Eigen::Quaterniond  euler_to_quat(double yaw, double pitch, double roll);

    pcl::PolygonMesh::Ptr combined_mesh_ptr_;

};    

#endif
