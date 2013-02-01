#include "MapView.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <octomap/octomap.h>

using namespace maps;

MapView::
MapView(const Spec& iSpec) {
  mSpec = iSpec;
}

MapView::
~MapView() {
}

MapView::Spec MapView::
getSpec() const {
  return mSpec;
}

bool MapView::
set(const maps::PointCloud& iCloud) {
  boost::mutex::scoped_lock lock(mMutex);
  mCloud.reset(new maps::PointCloud(iCloud));
  return true;
}

bool MapView::
set(const maps::Octree& iTree) {
  boost::mutex::scoped_lock lock(mMutex);
  mCloud.reset(new maps::PointCloud());
  octomap::OcTree::leaf_iterator iter = iTree.mTree->begin_leafs();
  for (; iter != iTree.mTree->end_leafs(); ++iter) {
    if (iTree.mTree->isNodeOccupied(*iter)) {
      maps::PointCloud::PointType pt;
      pt.x = iter.getX();
      pt.y = iter.getY();
      pt.z = iter.getZ();
      mCloud->push_back(pt);
    }
  }
  pcl::transformPointCloud(*mCloud, *mCloud, iTree.mTransform);
  return true;
}

MapView::Ptr MapView::
clone() const {
  boost::mutex::scoped_lock lock(mMutex);
  Ptr view(new MapView(mSpec));
  view->mCloud.reset(new maps::PointCloud());
  pcl::copyPointCloud(*mCloud, *view->mCloud);
  return view;
}

maps::PointCloud::Ptr MapView::
getAsPointCloud() const {
  boost::mutex::scoped_lock lock(mMutex);
  maps::PointCloud::Ptr cloud(new maps::PointCloud(*mCloud));
  return cloud;
}
