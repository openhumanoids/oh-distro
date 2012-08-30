#ifndef gen_mesh_HPP_
#define gen_mesh_HPP_


//#include <pointcloud_utils/pointcloud_lcm.hpp>
//#include <pointcloud_utils/pointcloud_vis.hpp>
#include "pcl/PolygonMesh.h"


///////////////////////////////////////////////////////////////
class gen_mesh{
  public:
    gen_mesh();
    
    ~gen_mesh(){
    }

    void gen_cylinder (pcl::PolygonMesh::Ptr &mesh,double radius,double length);

  private:

};    

#endif
