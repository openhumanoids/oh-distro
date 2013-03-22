#ifndef GROW_CLOUD_HPP_
#define GROW_CLOUD_HPP_

#include <iostream>

#include <lcm/lcm.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

// Find Floor:
#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
// person filtering:
#include <pcl/filters/passthrough.h>
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/extract_indices.h"
// Plane Extractions:
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/concave_hull.h"
#include "pcl/PolygonMesh.h"
#include "pcl/octree/octree.h"


#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

using namespace std;
typedef pcl::PointXYZRGB PointT;
// removed due to conflict with VS_SLAM
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;



typedef struct _BasicPlane
{
  int64_t utime;
  std::string name; // a unique name eg the file name
  int major; // which major plane 
  int minor; // which minor plane
  //Hull of Plane (either convex or concave - depending on algorithm)
  pcl::PointCloud<pcl::PointXYZRGB> cloud ;
   pcl::ModelCoefficients coeffs;  
  Eigen::Vector4f centroid;

  // this number of points in the original soure cloud.
  // Used for voting when combining to a final cloud
  int n_source_points; 
  
  // New members added in jun17_2011: mfallon
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Affine3f transform000; // transform to move the plane to 0,0,0 [ie centroid=0,0,0] aligned to the x,y axis (i think)
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
  
} BasicPlane;




class GrowCloud {
  public:
    GrowCloud (); 

    inline void setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input)
    {
      incloud   = input;
    }    

    virtual inline void setLCM(lcm_t *publish_lcm_in){
      publish_lcm = publish_lcm_in;
    }    

    
    
    
    // function to 'grow' point cloud from a set of points, 
    // seperatating disconnected points into different cloud using octrees
    // @input: a point cloud with all points on a plane. points may be from disconnected clouds
    // @output: individual clouds    
    void doGrowCloud (vector<BasicPlane> &outstack);
    
    virtual inline void setMinCloudSize(int min_cloud_size_in){
      min_cloud_size_ = min_cloud_size_in;
    }    
    
    // function to return a plane stack's coeffs in string
    void printPlaneStackCoeffs (vector<BasicPlane> &planeStack,  std::stringstream &ss);
    // ditto for the bounding points 
    void printPlaneStackHull (vector<BasicPlane> &planeStack,  std::stringstream &ss);
    
    void visualizePlanes(vector<BasicPlane> &plane_stack,pointcloud_vis* pc_vis_ , int null_id );
    
  protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud;    
    
    // the pose that this cloud is connected to (so as to publish onto a collections pose)
    // if pose_coll_id is negative, don't publish to LCM:
    int pose_coll_id;
    int64_t pose_element_id;
    
    int verbose_text;
    int verbose_lcm;
    lcm_t *publish_lcm;     
    
    // settings:
    int min_cloud_size_;
};

#endif  //#ifndef _GROW_CLOUD_HPP_
