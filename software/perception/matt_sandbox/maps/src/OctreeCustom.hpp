#ifndef _OctreeCustom_hpp_
#define _OctreeCustom_hpp_

#include <pcl/octree/octree.h>

template <typename PointT>
class OctreeCustom : public pcl::octree::OctreePointCloudOccupancy<PointT> {
public:
  OctreeCustom(const double iResolution) :
    pcl::octree::OctreePointCloudOccupancy<PointT>(iResolution) {}

  // TODO: custom methods here, like delta computation and
  // deserialize from compressed representation
};

#endif
