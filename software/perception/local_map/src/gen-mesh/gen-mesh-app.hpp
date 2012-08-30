#ifndef gen_mesh_app_hpp_
#define gen_mesh_app_hpp_

#include <lcm/lcm.h>

#include <pointcloud_utils/pointcloud_lcm.hpp>
#include <pointcloud_utils/pointcloud_vis.hpp>

#include "gen-mesh.hpp"
///////////////////////////////////////////////////////////////
class gen_mesh_app{
  public:
    gen_mesh_app(lcm_t* publish_lcm);
    
    ~gen_mesh_app(){
    }

    void do_app ();
    void send_cylinder (pcl::PolygonMesh::Ptr &mesh);
  private:

    lcm_t* publish_lcm_;

    pointcloud_vis* pc_vis_;

    Isometry3dTime null_poseT;
};    

#endif
