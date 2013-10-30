#ifndef FILTER_PLANES_HPP_
#define FILTER_PLANES_HPP_

// example: one class, two objects
#include <iostream>
#include <limits.h>

#include <lcm/lcm.h>

#include "pcl/pcl_macros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/concave_hull.h"

#include "visualization/collections.hpp"

//#include "kmcl/kmcl_utils.hpp"
#include "grow_cloud.hpp"





using namespace std;
// removed due to conflict with VS_SLAM
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;


class FilterPlanes {
  public:
    FilterPlanes ();

    inline void setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input)
    {
      cloud   = input;
    }
    
    virtual inline void setPoseIDs(int pose_coll_id_in,int64_t pose_element_id_in){
      pose_coll_id = pose_coll_id_in;
      pose_element_id = pose_element_id_in;      
    }
    
    virtual inline void setLCM(lcm_t *publish_lcm_in, int verbose_lcm_in = 1000 ){
      publish_lcm = publish_lcm_in;
      verbose_lcm = verbose_lcm_in;
    }    
    
    virtual inline void setDistanceThreshold(double distance_threshold_in){
      distance_threshold_ = distance_threshold_in;
    }
    
    virtual inline void setStopProportion(double stop_proportion_in){
      stop_proportion_ = stop_proportion_in;
    }
    
    virtual inline void setStopCloudSize(int stop_cloud_size_in){
      stop_cloud_size_ = stop_cloud_size_in;
    }
    

    // Filter out the major planes in a kinect PointCloud
    // send these planes to the lcm-viewer coloured by their mean colour
    // Version 2 of the algorithm: after extracting major planes:
    // - statistical outlyier removal of plane points
    // - sequentially grow a plane from points using nearest neaighbour ocktrees
    void filterPlanes(vector<BasicPlane> &plane_stack);

  protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;    
    
    // the pose that this cloud is connected to (so as to publish onto a collections pose)
    // if pose_coll_id is negative, don't publish to LCM:
    int pose_coll_id;
    int64_t pose_element_id;
    
    int verbose_text;

    // 0 means publish nothing
    // 1 means publish very important only [runs without issue]
    // 2 means publish important
    // 3 means publish all
    int verbose_lcm;
    lcm_t *publish_lcm;   
    
    // distance for ransac
    double distance_threshold_;
    // fraction at which we stop: [0,1]
    double stop_proportion_;
    // number of points at which we should stop:
    int stop_cloud_size_;
};


#endif  //#ifndef _FILTER_PLANES_HPP_
