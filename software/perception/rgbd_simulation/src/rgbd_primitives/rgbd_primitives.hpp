#ifndef rgbd_primitives_HPP_
#define rgbd_primitives_HPP_

#include <iostream>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>


//using namespace pcl;
//using namespace pcl::io;

class rgbd_primitives {
  public:
    rgbd_primitives ();
    
    pcl::PolygonMesh::Ptr getCylinderWithTransform(Eigen::Isometry3d transform, double base, double top, double height);
    pcl::PolygonMesh::Ptr getCubeWithTransform(Eigen::Isometry3d transform, double xdim, double ydim, double zdim);

    pcl::PolygonMesh::Ptr getCylinder(double base, double top, double height, int slices, int stacks);
    pcl::PolygonMesh::Ptr getCube(double xdim, double ydim, double zdim);

  private:
};




#endif
