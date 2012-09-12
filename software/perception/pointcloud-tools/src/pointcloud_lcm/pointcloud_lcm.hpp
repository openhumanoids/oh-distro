#ifndef POINTCLOUD_LCM_HPP_
#define POINTCLOUD_LCM_HPP_

#include <lcm/lcm.h>
#include <iostream>

#include <vector>
#include <algorithm>

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
#include <lcmtypes/pointcloud_tools.h>


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



void unpack_pointcloud2(const putils_pointcloud2_t *msg,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

  // 1. Copy fields - this duplicates /pcl/ros/conversions.h for "fromROSmsg"
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  cloud->width   = msg->width;
  cloud->height   = msg->height;
  uint32_t num_points = msg->width * msg->height;
  cloud->points.resize (num_points);
  cloud->is_dense = false;//msg->is_dense;
  uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud->points[0]);
  uint32_t cloud_row_step = static_cast<uint32_t> (sizeof (pcl::PointXYZRGB) * cloud->width);
  const uint8_t* msg_data = &msg->data[0];
  memcpy (cloud_data, msg_data, msg->data_nbytes );

  // 2. HACK/Workaround
  // for some reason in pcl1.5/6, this callback results
  // in RGB data whose offset is not correctly understood
  // Instead of an offset of 12bytes, its offset is set to be 16
  // this fix corrects for the issue:
  sensor_msgs::PointCloud2 msg_cld;
  pcl::toROSMsg(*cloud, msg_cld);
  msg_cld.fields[3].offset = 12;
  pcl::fromROSMsg (msg_cld, *cloud);

  //std::cerr << "Received Cloud with " << cloud->points.size () << " data points." << std::endl;

  // Transform cloud to that its in robotic frame:
  // Could also apply the cv->robotic transform directly
  double x_temp;
  for(int j=0; j<cloud->points.size(); j++) {
    x_temp = cloud->points[j].x;
    cloud->points[j].x = cloud->points[j].z;
    cloud->points[j].z = - cloud->points[j].y;
    cloud->points[j].y = - x_temp;
  }
}


#endif
