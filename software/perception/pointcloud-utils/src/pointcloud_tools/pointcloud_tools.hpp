#ifndef POINTCLOUD_TOOLS_HPP_
#define POINTCLOUD_TOOLS_HPP_


#include <lcm/lcm.h>
#include <iostream>


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pcl/ModelCoefficients.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"

#include <pcl/filters/passthrough.h>
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"
#include "pcl/surface/concave_hull.h"
#include "pcl/PolygonMesh.h"
#include "pcl/octree/octree.h"

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/ia_ransac.h"

#include <bot_core/bot_core.h>

#include <vector>
#include <algorithm>

using namespace pcl;
using namespace pcl::io;


#include <pointcloud_utils/pointcloud_math.hpp>

class pointcloud_tools {
  public:
    pointcloud_tools (lcm_t* publish_lcm);

  private:
    lcm_t *publish_lcm_; 

};


#endif

