#ifndef _maps_Types_hpp_
#define _maps_Types_hpp_

#include <pcl/point_types.h>
#include <Eigen/Geometry>

// forward declaration
namespace lcm {
  class LCM;
}

namespace maps {
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  typedef boost::shared_ptr<lcm::LCM> LcmPtr;

  struct PointSet {
    int64_t mTimestamp;
    float mMinRange;
    float mMaxRange;
    PointCloud::Ptr mCloud;
  };

  struct TriangleMesh {
    std::vector<Eigen::Vector3f> mVertices;
    std::vector<Eigen::Vector3i> mFaces;
    typedef boost::shared_ptr<TriangleMesh> Ptr;    
  };
}

#endif
