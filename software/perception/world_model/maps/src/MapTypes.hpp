#ifndef _MapTypes_hpp_
#define _MapTypes_hpp_

#include <pcl/point_types.h>

namespace maptypes {
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;
}

#endif
