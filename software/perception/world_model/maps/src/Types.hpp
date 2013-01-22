#ifndef _maps_Types_hpp_
#define _maps_Types_hpp_

#include <pcl/point_types.h>

namespace maps {
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  struct PointSet {
    int64_t mTimestamp;
    PointCloud::Ptr mCloud;
  };

}

#endif
