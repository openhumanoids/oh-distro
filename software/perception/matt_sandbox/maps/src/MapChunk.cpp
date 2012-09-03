#include "MapChunk.hpp"

#include <boost/functional/hash.hpp>
#include <pcl/octree/octree_impl.h>

MapChunk::
MapChunk() {
  setId(-1);
  setLastUpdateTime(-1);
  setTransformToLocal(Eigen::Affine3d::Identity());
  setBounds(Eigen::Vector3d(-1e20, -1e20, -1e20),
            Eigen::Vector3d(1e20, 1e20, 1e20));
  setResolution(0.01);
}

MapChunk::
~MapChunk() {
}

void MapChunk::
clear() {
  mOctree->deleteTree();
}

void MapChunk::
setId(const int64_t iId) {
  mId = iId;
}

int64_t MapChunk::
getId() const {
  return mId;
}

void MapChunk::
setLastUpdateTime(const int64_t iTime) {
  mLastUpdateTime = iTime;
}

int64_t MapChunk::
getLastUpdateTime() const {
  return mLastUpdateTime;
}

void MapChunk::
setTransformToLocal(const Eigen::Affine3d& iTransform) {
  mTransformToLocal = iTransform;
}

Eigen::Affine3d MapChunk::
getTransformToLocal() const {
  return mTransformToLocal;
}

bool MapChunk::
setBounds(const Eigen::Vector3d& iMin, const Eigen::Vector3d& iMax) {
  if ((iMin[0] > iMax[0]) || (iMin[1] > iMax[1]) || (iMin[2] > iMax[2])) {
    return false;
  }
  mBoundMin = iMin;
  mBoundMax = iMax;
  return true;
}

Eigen::Vector3d MapChunk::
getBoundMin() const {
  return mBoundMin;
}

Eigen::Vector3d MapChunk::
getBoundMax() const {
  return mBoundMax;
}

void MapChunk::
setResolution(const double iResolution) {
  mResolution = iResolution;
  mOctree.reset(new Octree(mResolution));
}

double MapChunk::
getResolution() const {
  return mResolution;
}

bool MapChunk::
add(const PointCloud& iCloud) {
  /*
  for (int i = 0; i < iCloud.size(); ++i) {
    const pcl::PointXYZ& pt = iCloud.points[i];
    mOctree->setOccupiedVoxelAtPoint(pt);
  }
  */

  PointCloud::Ptr cloud(new PointCloud(iCloud));
  mOctree->setInputCloud(cloud);
  mOctree->addPointsFromInputCloud();
  return false;
}

bool MapChunk::
remove(const PointCloud& iCloud) {
  for (int i = 0; i < iCloud.size(); ++i) {
    mOctree->deleteVoxelAtPoint(iCloud.points[i]);
  }
  return false;
}

void MapChunk::
deepCopy(const MapChunk& iChunk) {
  *this = iChunk;
  mOctree.reset(new Octree(mResolution));
  *mOctree = *(iChunk.mOctree);
}

MapChunk::VoxelIterator MapChunk::
getVoxelIterator() const {
  return VoxelIterator(*mOctree);
}

bool MapChunk::
findDifferences(const MapChunk& iMap, PointCloud& oAdded,
                PointCloud& oRemoved) {

  // find newly added points
  oAdded.clear();
  double boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ;
  iMap.mOctree->getBoundingBox(boundMinX, boundMinY, boundMinZ,
                               boundMaxX, boundMaxY, boundMaxZ);
  double resolution = iMap.mOctree->getResolution();
  Eigen::Vector3d boundMin(boundMinX, boundMinY, boundMinZ);
  MapChunk::VoxelIterator iter = iMap.getVoxelIterator();
  for (; (*iter) != NULL; ++iter) {
    pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
    if (mOctree->existLeaf(key.x, key.y, key.z)) {
      continue;
    }

    Eigen::Vector3d pt(key.x, key.y, key.z);
    pt = (pt.array() + 0.5) * resolution;
    pt += boundMin;
    oAdded.push_back(PointCloud::PointType(pt[0], pt[1], pt[2]));
  }

  // find newly removed points
  oRemoved.clear();
  mOctree->getBoundingBox(boundMinX, boundMinY, boundMinZ,
                          boundMaxX, boundMaxY, boundMaxZ);
  resolution = mOctree->getResolution();
  boundMin = Eigen::Vector3d(boundMinX, boundMinY, boundMinZ);
  for (iter = getVoxelIterator(); (*iter) != NULL; ++iter) {
    pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
    if (iMap.mOctree->existLeaf(key.x, key.y, key.z)) {
      continue;
    }

    Eigen::Vector3d pt(key.x, key.y, key.z);
    pt = (pt.array() + 0.5) * resolution;
    pt += boundMin;
    oRemoved.push_back(PointCloud::PointType(pt[0], pt[1], pt[2]));
  }
  
  return true;
}

void MapChunk::
serialize(std::vector<char>& oBytes) const {
  // TODO
}

void MapChunk::
deserialize(const std::vector<char>& iBytes) {
  // TODO
}
