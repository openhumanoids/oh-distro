#ifndef PCL_SIMULATION_IO_
#define PCL_SIMULATION_IO_

#include <boost/shared_ptr.hpp>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>


#include "rgbd_simulation/camera.h"
#include "rgbd_simulation/scene.h"
#include "rgbd_simulation/range_likelihood.h"

// Writing PNG files:
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>

#include <lcmtypes/bot_core_image_t.h>

using namespace pcl::simulation;


struct PolygonMeshStruct
{
  // Name of the link
  std::string link_name;
  // Location of obj file (or other)
  std::string file_path;
  // Contents of the file:
  pcl::PolygonMesh::Ptr polygon_mesh;
};


class SimExample
{
  public:
    typedef boost::shared_ptr<SimExample> Ptr;
    typedef boost::shared_ptr<const SimExample> ConstPtr;
	
    SimExample (int argc, char** argv,
		int height_,int width_, boost::shared_ptr<lcm::LCM> &lcm_, int output_color_mode_,
		std::string path_to_shaders ="");
    void initializeGL (int argc, char** argv);
    
    Scene::Ptr scene_;
    Camera::Ptr camera_;
    RangeLikelihood::Ptr rl_;  

    std::map<std::string, PolygonMeshStruct > polymesh_map_; // associates link name with pcl::PolyMesh struct

    // Create a mapping between link names and meshes 
    // and then read in the meshes from file
    void setPolygonMeshs (std::vector< std::string > link_names_in,
                          std::vector< std::string > file_paths_in);
    
    bool mergePolygonMesh(pcl::PolygonMesh::Ptr &meshA, pcl::PolygonMesh::Ptr meshB);
    
    // generic double to float converter: [remove if something in eigen works instead]
    Eigen::Isometry3f isometryDoubleToFloat(Eigen::Isometry3d );    
    
    // Create a scene using a robot's joint positions
    // NOTE: generalised to allow rendering of any set of objects. link = object
    void createScene (std::vector<std::string> object_names,
                      std::vector<Eigen::Isometry3d> object_tfs);

    void mergePolygonMeshToCombinedMesh( pcl::PolygonMesh::Ptr meshB);
    // Actually upload to GPU
    void addScene ();
    void doSim (Eigen::Isometry3d pose_in);

    // these ARE DEPRECIATED:
    void write_score_image(const float* score_buffer,std::string fname, int64_t utime);
    void write_depth_image_uint(const float* depth_buffer,std::string fname, int64_t utime);

    // Get the GL depth buffer, invert it, color mask it. always 3 colors
    uint8_t* getDepthBuffer();
    // Get the GL color buffer, invert it and output either gray (red channel) or rgb
    uint8_t* getColorBuffer(int n_colors_);

    void setCameraIntrinsicsParameters (int camera_width_in,
                                      int camera_height_in,
                                      float camera_fx_in,
                                      float camera_fy_in,
                                      float camera_cx_in,
                                      float camera_cy_in);

    pcl::PolygonMesh::Ptr getCombinedMesh(){ return combined_mesh_ptr_; }

    // r,g,b are assumed to be in the range 0->255 where 0,0,0 is white and 1,1,1 black
    void setPolygonMeshColor( pcl::PolygonMesh::Ptr &mesh, int r,int g, int b );

    
    // Duplicates the list in collections renderer:
    // assumed to be 3xN colors
    std::vector < int > colors_;       
    
  private:
    uint16_t t_gamma[2048];  

    // either output a (0) color mash or (1) grey mask or (2) a binary b/w mask
    int output_color_mode_;
    
    
    // of platter, usually 640x480
    int width_;
    int height_;
    boost::shared_ptr<lcm::LCM> lcm_;
    
    pcl::PolygonMesh::Ptr combined_mesh_ptr_;
    
    uint8_t* img_buffer_;
};




#endif
