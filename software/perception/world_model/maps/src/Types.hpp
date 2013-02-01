#ifndef _maps_Types_hpp_
#define _maps_Types_hpp_

#include <pcl/point_types.h>
#include <Eigen/Geometry>

// forward declaration
namespace octomap {
  class OcTree;
}

namespace maps {
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  struct PointSet {
    int64_t mTimestamp;
    float mMaxRange;
    PointCloud::Ptr mCloud;
  };

  struct Octree {
    boost::shared_ptr<octomap::OcTree> mTree;
    Eigen::Isometry3f mTransform;
  };

}

#endif
