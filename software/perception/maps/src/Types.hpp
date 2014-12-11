#ifndef _maps_Types_hpp_
#define _maps_Types_hpp_

#include <memory>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

// forward declaration
namespace lcm {
  class LCM;
}

namespace maps {
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  struct PointSet {
    int64_t mTimestamp;
    PointCloud::Ptr mCloud;
  };

  struct TriangleMesh {
    std::vector<Eigen::Vector3f> mVertices;
    std::vector<Eigen::Vector3i> mFaces;
    std::vector<Eigen::Vector3f> mNormals;
    typedef std::shared_ptr<TriangleMesh> Ptr;    
  };
}

#endif
