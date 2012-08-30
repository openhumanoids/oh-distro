#ifndef FILTER_CLOUDTOLASER_HPP_
#define FILTER_CLOUDTOLASER_HPP_

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


// #include "kmcl/kmcl_utils.hpp"

// #include <lcmtypes/bot_core.h>


using namespace std;
// removed due to conflict with VS_SLAM
//typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;


class FilterCloudToLaser {
  public:
    FilterCloudToLaser ();

    ~FilterCloudToLaser ();
    

    inline void setInputCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input)
    {
      incloud   = input;
    }
    
    virtual inline void setPoseIDs(int pose_coll_id_in,int64_t pose_element_id_in){
      pose_coll_id = pose_coll_id_in;
      pose_element_id = pose_element_id_in;      
    }
    
    inline void setLCM(lcm_t *publish_lcm_in){
      publish_lcm = publish_lcm_in;
    }    

    inline void setMaxHeight(double max_height_in){
      max_height = max_height_in;
    }    
    inline void setMinHeight(double min_height_in){
      min_height = min_height_in;
    }    

    // Lightly Filter the point cloud and then publish it to LCM
    // Filter out long ranges, null ranges and a bottom line
    bool doCloud2Laser(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outcloud,bot_core_planar_lidar_t &outlidar);

  protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud;
    double min_height,max_height;
    
    // simulated lidar settings:
    double rad0;
    double radstep;
    double radmax;
    double range_max;
    int nranges;
    float *ranges;
    
    // the pose that this cloud is connected to (so as to publish onto a collections pose)
    // if pose_coll_id is negative, don't publish to LCM:
    int pose_coll_id;
    int64_t pose_element_id;
    
    int verbose_text;
    int verbose_lcm;
    lcm_t *publish_lcm;    
};

#endif  //#ifndef _FILTER_CLOUDTOLASER_HPP_
