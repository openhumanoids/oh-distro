#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>


// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include "pcl/PolygonMesh.h"
#include <pcl/common/transforms.h>


#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds and write mesh
#include <ConciseArgs>

#include <affordance/AffordanceUtils.hpp>

using namespace std;
using namespace pcl;
using namespace pcl::io;

int main(int argc, char*argv[]){
  if(argc!=3){
    cout << "usage: ply2pcd in out\n";
    exit(-1);
  }

  AffordanceUtils affutils;
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  affutils.getModelAsLists(argv[1], points, triangles);
  cout << points.size() << endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=affutils.getCloudFromAffordance(points);
  cloud->width = 1; cloud->height = cloud->size();
  cout << cloud->size() << endl;
  pcl::PCDWriter writer;
  writer.write (argv[2], *cloud, false);  
}
