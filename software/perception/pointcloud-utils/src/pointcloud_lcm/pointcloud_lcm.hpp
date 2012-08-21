#ifndef POINTCLOUD_LCM_HPP_
#define POINTCLOUD_LCM_HPP_

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

class pointcloud_lcm {
  public:
    pointcloud_lcm (lcm_t* publish_lcm);

  private:
    lcm_t *publish_lcm_; 

};




inline void
convertLidar(const float * ranges, int numPoints, double thetaStart,
        double thetaStep,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
	double maxRange = 1e10,
	double validRangeStart = -1000, double validRangeEnd = 1000)
{
  int count = 0;
  double theta = thetaStart;

  cloud->width   = numPoints;
  cloud->height   = 1;
  cloud->points.resize (numPoints);


    for (int i = 0; i < numPoints; i++) {
        if (ranges[i] > .1 && ranges[i] < maxRange && theta > validRangeStart
                && theta < validRangeEnd) { 
            //hokuyo driver seems to report maxRanges as .001 :-/
            //project to body centered coordinates
            cloud->points[count].x = ranges[i] * cos(theta);
	    cloud->points[count].y = ranges[i] * sin(theta);
            count++;
        }
        theta += thetaStep;
    }
  // Resize outgoing cloud
  cloud->width   = count;
  cloud->points.resize (count);
}

#endif
