#include <iostream>
#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>

#include "image-passthrough.hpp"

#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/GlKinematicBody.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <rgbd_simulation/rgbd_primitives.hpp> // to create basic meshes
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <camera_params/camera_params.hpp>     // Camera Parameters
#include <model-client/model-client.hpp> // Receive the robot model

#include <path_util/path_util.h>
#include <affordance/AffordanceUtils.hpp>

#include "lcmtypes/drc/affordance_plus_t.hpp"
#include "lcmtypes/drc/affordance_plus_collection_t.hpp"


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
         std::string camera_channel_, int output_color_mode_, 
         bool use_convex_hulls_, std::string camera_frame_,
         CameraParams camera_params_, bool verbose_);
    
    ~Pass(){
    }
    
    bool createMask(int64_t msg_time);
    void sendOutput(int64_t utime);
    
    void sendOutputOverlay(int64_t utime, uint8_t* img_buf);
    
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
    
    void setUpdateRobotState(bool update_robot_state_in){
      update_robot_state_ = update_robot_state_in; 
    };
    void setRobotState(Eigen::Isometry3d & world_to_body_in, std::map<std::string, double> & jointpos_in){
      world_to_body_ = world_to_body_in;
      jointpos_ = jointpos_in;
      init_rstate_ = true;
    }
    void setAffordanceMesh(pcl::PolygonMesh::Ptr &aff_mesh_in){ 
      combined_aff_mesh_ = aff_mesh_in;
      aff_mesh_filled_ = true;
    }
    void setRendererRobot(bool renderer_robot_in){
      renderer_robot_ = renderer_robot_in;
    }
    
    
  private:
    boost::shared_ptr<ModelClient> model_;
    
    // settings:
    bool update_robot_state_; // update the robot state using EST_ROBOT_STATE continously?
    bool verbose_;
    int output_color_mode_;
    bool use_convex_hulls_;
    bool renderer_robot_;
    
    // LCM:
    boost::shared_ptr<lcm::LCM> lcm_;
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   
    void affordancePlusHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::affordance_plus_collection_t* msg);
    
    // External Objects:
    BotParam* botparam_;
    pronto_vis* pc_vis_;
    boost::shared_ptr<visualization_utils::GlKinematicBody> gl_robot_;
    image_io_utils*  imgutils_; 
    SimExample::Ptr simexample;
    boost::shared_ptr<rgbd_primitives>  prim_; // this should be moved into the library


    // Config:
    CameraParams camera_params_;   
    std::string camera_channel_, camera_frame_; // what channel and what frame
    
    // State:
    std::string urdf_xml_string_; 
    bool init_rstate_; // have we received a robot state?    
    Eigen::Isometry3d world_to_body_; // Current Position of the Robot:
    std::map<std::string, double> jointpos_;
    
    
    std::map<std::string, boost::shared_ptr<urdf::Link> > links_map_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    pcl::PolygonMesh::Ptr combined_aff_mesh_ ;
    bool aff_mesh_filled_;
    
    AffordanceUtils affutils;
    
    void prepareModel();
    
    void affordancePlusInterpret(drc::affordance_plus_t affplus, int aff_uid, pcl::PolygonMesh::Ptr &mesh_out);
    
};

