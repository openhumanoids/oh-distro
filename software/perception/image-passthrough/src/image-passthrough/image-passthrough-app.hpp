#include <iostream>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>

#include "image-passthrough.hpp"

#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/GlKinematicBody.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <path_util/path_util.h>
#include <affordance/AffordanceUtils.hpp>



#include <ConciseArgs>

#define DO_TIMING_PROFILE FALSE
#define PCL_VERBOSITY_LEVEL L_ERROR
// offset of affordance in mask labelling:
#define AFFORDANCE_OFFSET 64


class Pass{
  public:
    typedef boost::shared_ptr<Pass> Ptr;
    typedef boost::shared_ptr<const Pass> ConstPtr;
    
    Pass(int argc, char** argv, boost::shared_ptr<lcm::LCM> &publish_lcm, 
         std::string camera_channel_, int output_color_mode_, bool use_convex_hulls_, string camera_frame_);
    
    ~Pass(){
    }
    
    bool createMask(int64_t msg_time);
    void sendOutput(int64_t utime);
    
    // Get the GL depth buffer, flip up/down it, color mask it. always 3 colors
    uint8_t* getDepthBufferAsColor(){
      return simexample->getDepthBufferAsColor();
    }
    // Get the GL color buffer, flip up/down and output either gray (red channel) or rgb
    uint8_t* getColorBuffer(int n_colors){
      return simexample->getColorBuffer(n_colors);
    }
    
    // Get the raw GL depth buffer.  Direct pointer to GL buffer:
    const float* getDepthBuffer(){
      return simexample->getDepthBuffer();
    }
    // Get the GL depth buffer as floats - ranges I think
    const uint8_t* getColorBuffer(){
      return simexample->getColorBuffer();
    }    
    
  private:
    // LCM:
    boost::shared_ptr<lcm::LCM> lcm_;
    void urdfHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_urdf_t* msg);
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);
    bool urdf_parsed_;
    bool urdf_subscription_on_;
    lcm::Subscription *urdf_subscription_; //valid as long as urdf_parsed_ == false
    
    // External Objects:
    BotParam* botparam_;
    pointcloud_vis* pc_vis_;
    boost::shared_ptr<visualization_utils::GlKinematicBody> gl_robot_;
    image_io_utils*  imgutils_; 
    SimExample::Ptr simexample;
    boost::shared_ptr<rgbd_primitives>  prim_; // this should be moved into the library


    // Config:
    int width_, height_;
    std::string camera_channel_, camera_frame_; // what channel and what frame
    
    // State:
    pcl::PolygonMesh::Ptr combined_aff_mesh_ ;
    bool aff_mesh_filled_;
    drc::robot_state_t last_rstate_; // Last robot state: this is used to extract link positions:
    bool init_rstate_;    
    std::map<std::string, boost::shared_ptr<urdf::Link> > links_map_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    std::string robot_name_;
    std::string urdf_xml_string_; 
    
    // Settings:
    bool verbose_;
    int output_color_mode_;
    bool use_convex_hulls_;
    
    AffordanceUtils affutils;
    bool affordancePlusInterpret(drc::affordance_plus_t affplus, int aff_uid, pcl::PolygonMesh::Ptr &mesh_out);
};

