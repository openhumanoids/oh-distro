#ifndef _OctreeCustom_hpp_
#define _OctreeCustom_hpp_

#include <pcl/octree/octree.h>

template <typename PointT>
class OctreeCustom : public pcl::octree::OctreePointCloudOccupancy<PointT> {
public:
  OctreeCustom(const double iResolution) :
    pcl::octree::OctreePointCloudOccupancy<PointT>(iResolution) {}

  // TODO: need deep copy?
  // TODO: some custom stuff here, like delta comp and deserialize
};

#endif
