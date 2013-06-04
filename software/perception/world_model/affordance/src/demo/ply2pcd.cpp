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

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds and write mesh
#include <ConciseArgs>

#include <affordance/AffordanceUtils.hpp>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace Eigen;

int main(int argc, char*argv[]){
  if(argc!=3){
    cout << "usage: ply2pcd in out\n";
    exit(-1);
  }

  AffordanceUtils affutils;
  std::vector< std::vector< float > > points;
  std::vector< std::vector< int > > triangles;
  affutils.getModelAsLists(argv[1], points, triangles);

  float incr=0.01;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i=0;i<triangles.size();i++){
    for(int j=0;j<3;j++){
      vector<float>& p1v = points[triangles[i][j]]; 
      vector<float>& p2v = points[triangles[i][(j+1)%3]]; 
      Vector3f p1(p1v[0],p1v[1],p1v[2]);
      Vector3f p2(p2v[0],p2v[1],p2v[2]);
      float size = (p2-p1).norm();
      Vector3f d = (p2-p1)/size;
      for(float f=0;;f+=incr){
        if(f>size) f=size;
        Vector3f p = p1+d*f;
        //cout << f << " " << p.transpose() << endl;
        pcl::PointXYZRGB pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
        cloud->points.push_back(pt);      
        if(f==size) break;
      }
    }
  }

  cloud->width = 1; cloud->height = cloud->size();
  cout << cloud->size() << endl;

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  pcl::PCDWriter writer;
  writer.write (argv[2], *cloud_filtered, false);  
}
