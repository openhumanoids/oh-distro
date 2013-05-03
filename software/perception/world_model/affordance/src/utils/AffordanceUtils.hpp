#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>


// define the following in order to eliminate the deprecated headers warning
//#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/io/vtk_lib_io.h>
#include "pcl/PolygonMesh.h"
#include <pcl/common/transforms.h>


#include <rgbd_simulation/rgbd_primitives.hpp>


class AffordanceUtils {
  public:
    AffordanceUtils ();

    Eigen::Isometry3d getPose(double xyz[3], double rpy[3]);
    //std::vector<std::string> param_names, std::vector<double> params );

    // just return the raw points:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudFromAffordance(std::vector< std::vector< float > > &points);
    
    // sample the mesh:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloudFromAffordance(std::vector< std::vector< float > > &points,
          std::vector< std::vector< int > > &triangles, double pts_per_msquared);
    
    pcl::PolygonMesh::Ptr getMeshFromAffordance(std::vector< std::vector< float > > &points, 
                  std::vector< std::vector< int > > &triangles, Eigen::Isometry3d & transform);
    
    pcl::PolygonMesh::Ptr getMeshFromAffordance(std::vector< std::vector< float > > &points, 
                  std::vector< std::vector< int > > &triangles);
    
    
    ////// Conversion from/to Plane from XYZYPR affordance params
    void setPlaneFromXYZYPR(double xyz[3], double rpy[3], 
                   std::vector<float> &plane_coeffs, Eigen::Vector3d &plane_centroid);
    
    void setXYZRPYFromPlane(double xyz[3], double rpy[3], 
                   std::vector<float> plane_coeffs, Eigen::Vector3d plane_centroid);
    
    void setXYZRPYFromIsometry3d(double xyz[3], double rpy[3], 
                   Eigen::Isometry3d &pose);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getBoundingBoxCloud(double bounding_xyz[], double bounding_rpy[], double bounding_lwh[]);

    void printAffordance(std::vector<std::string> &param_names, std::vector<double> &params,
                         std::stringstream &ss){
      for (size_t j=0; j< param_names.size(); j++){
        ss << param_names[j] << ": " << params[j] << " | ";
      }      
    }
    
  private:
    boost::shared_ptr<rgbd_primitives>  prim_;
    
};


