#include "MapView.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <octomap/octomap.h>

using namespace maps;

MapView::Spec::
Spec() {
  mMapId = mViewId = 0;
  mActive = false;
  mRelativeTime = false;
  mRelativeLocation = false;
  mType = TypeCloud;
  mResolution = 0;
  mFrequency = 0;
  mTimeMin = mTimeMax = 0;
}

bool MapView::Spec::
operator==(const Spec& iSpec) const {
  bool eq = (mMapId == iSpec.mMapId) &&
    (mViewId == iSpec.mViewId) &&
    (mActive == iSpec.mActive) &&
    (mRelativeTime == iSpec.mRelativeTime) &&
    (mRelativeLocation == iSpec.mRelativeLocation) &&
    (mType == iSpec.mType) &&
    (mResolution == iSpec.mResolution) &&
    (mFrequency == iSpec.mFrequency) &&
    (mTimeMin == iSpec.mTimeMin) &&
    (mTimeMax == iSpec.mTimeMax) &&
    (mClipPlanes.size() == iSpec.mClipPlanes.size());
  if (!eq) {
    return false;
  }
  for (int i = 0; i < mClipPlanes.size(); ++i) {
    if (mClipPlanes[i] != iSpec.mClipPlanes[i]) {
      return false;
    }
  }
  return true;
}

bool MapView::Spec::
operator!=(const Spec& iSpec) const {
  return !(*this == iSpec);
}




MapView::
MapView(const Spec& iSpec) {
  mSpec = iSpec;
  mCloud.reset(new maps::PointCloud());
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
  view->mCloud = mCloud;
  return view;
}

MapView::Ptr MapView::
clone(const Spec& iSpec) const {
  boost::mutex::scoped_lock lock(mMutex);
  Ptr view(new MapView(iSpec));
  view->mCloud = mCloud;
  return view;
}

maps::PointCloud::Ptr MapView::
getAsPointCloud() const {
  boost::mutex::scoped_lock lock(mMutex);
  maps::PointCloud::Ptr cloud(new maps::PointCloud(*mCloud));
  return cloud;
}

MapView::HeightMap::Ptr MapView::
getAsHeightMap(const float iResolution, const float iMaxHeight) const {
  // get points
  maps::PointCloud cloud;
  {
    boost::mutex::scoped_lock lock(mMutex);
    cloud = *mCloud;
  }

  // determine 2d extents
  maps::PointCloud::PointType minPoint, maxPoint;
  pcl::getMinMax3D(cloud, minPoint, maxPoint);

  // compute transform from image to reference coordinates
  Eigen::Affine3f imgToRef = Eigen::Affine3f::Identity();
  imgToRef(0,0) = imgToRef(1,1) = iResolution;
  imgToRef(0,3) = minPoint.x;
  imgToRef(1,3) = minPoint.y;
  Eigen::Affine3f refToImg = imgToRef.inverse();
  
  // initialize height map data
  HeightMap::Ptr heightMap(new HeightMap());
  const float unobservedValue = -std::numeric_limits<float>::max();
  heightMap->mTransform = imgToRef;
  heightMap->mWidth = ceil((maxPoint.x - minPoint.x)/iResolution) + 1;
  heightMap->mHeight = ceil((maxPoint.y - minPoint.y)/iResolution) + 1;
  int totalPixels = heightMap->mWidth * heightMap->mHeight;
  heightMap->mData.resize(totalPixels);
  std::fill(heightMap->mData.begin(), heightMap->mData.end(), unobservedValue);

  // add height values into height map
  // TODO: consider case of going from lower res to higher res
  pcl::transformPointCloud(cloud, cloud, refToImg);
  for (int i = 0; i < cloud.size(); ++i) {
    int x(cloud[i].x + 0.5f), y(cloud[i].y + 0.5f);
    int index = y*heightMap->mWidth + x;
    heightMap->mData[index] = std::max(heightMap->mData[index], cloud[i].z);
  }

  return heightMap;
}
