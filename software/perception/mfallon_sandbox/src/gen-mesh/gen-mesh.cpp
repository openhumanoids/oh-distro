#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointT;


#include "gen-mesh.hpp"



gen_mesh::gen_mesh(){
}



void gen_mesh::gen_cylinder(pcl::PolygonMesh::Ptr &mesh,double radius,double length){

  // Params:
  //double radius =10;
  //double length = 20;

  int numPoints =30;
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  for (int i = 0; i < numPoints; i++) {
    pcl::PointXYZRGB pt;

    double th1 =   i *( 2*M_PI/numPoints);
    double th2 =   (i+1) *( 2*M_PI/numPoints);

    /// Top Half:
    pt.x = radius*cos(th1);
    pt.y = radius*sin(th1);
    pt.z = 0;
    cloud->points.push_back(pt);
    pt.z = length;
    cloud->points.push_back(pt);

    pt.x = radius*cos(th2);
    pt.y = radius*sin(th2);
    pt.z = 0;
    cloud->points.push_back(pt);

    pcl::Vertices apoly_out;
    apoly_out.vertices.push_back(i*6);
    apoly_out.vertices.push_back(i*6 + 1);
    apoly_out.vertices.push_back(i*6 + 2);
    mesh->polygons.push_back(apoly_out);

    /// Bottom Half:
    pt.x = radius*cos(th2);
    pt.y = radius*sin(th2);
    pt.z = 0;
    cloud->points.push_back(pt);
    pt.z = length;
    cloud->points.push_back(pt);

    pt.x = radius*cos(th1);
    pt.y = radius*sin(th1);
    pt.z = length;
    cloud->points.push_back(pt);

    pcl::Vertices apoly_out2;
    apoly_out2.vertices.push_back(i*6 + 3);
    apoly_out2.vertices.push_back(i*6 + 4);
    apoly_out2.vertices.push_back(i*6 + 5);
    mesh->polygons.push_back(apoly_out2);

  }

  cloud->width    = 1;
  cloud->height   = cloud->points.size();
  cloud->is_dense = false;
  pcl::toROSMsg (*cloud, mesh->cloud);
}
