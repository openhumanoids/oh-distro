#include "MapChunk.hpp"

#include <boost/functional/hash.hpp>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::octree::OctreePointCloudOccupancy<MapChunk::PointCloud::PointType> OccupancyOctree;

MapChunk::
MapChunk() {
  setId(-1);
  setLastUpdateTime(-1);
  setTransformToLocal(Eigen::Isometry3d::Identity());
  setBounds(Eigen::Vector3d(-1e20, -1e20, -1e20),
            Eigen::Vector3d(1e20, 1e20, 1e20));
  mBackingPoints.reset(new PointCloud());
  storeBackingPoints(true);
  setResolution(0.01);
}

MapChunk::
~MapChunk() {
}

void MapChunk::
clear() {
  mOctree->deleteTree();
  mBackingPoints.reset(new PointCloud());
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
setTransformToLocal(const Eigen::Isometry3d& iTransform) {
  mTransformToLocal = iTransform;
}

Eigen::Isometry3d MapChunk::
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
  mNeedsUpdate = true;
}

double MapChunk::
getResolution() const {
  return mResolution;
}

void MapChunk::storeBackingPoints(const bool iVal) {
  mStoreBackingPoints = iVal;
  mNeedsUpdate = true;
}

void MapChunk::
updateStructures() {
  if (!mNeedsUpdate) {
    return;
  }

  if (mStoreBackingPoints) {
    mOctree.reset(new Octree(mResolution));
  }
  else {
    // TODO: cast may not be kosher
    mOctree.reset((Octree*)(new OccupancyOctree(mResolution)));
  }
  mBackingPoints.reset(new PointCloud());
  mOctree->setInputCloud(mBackingPoints);
  mNeedsUpdate = false;
}


bool MapChunk::
add(const PointCloud::Ptr& iCloud) {
  updateStructures();

  if (mStoreBackingPoints) {
    for (int i = 0; i < iCloud->size(); ++i) {
      mOctree->addPointToCloud(iCloud->points[i], mBackingPoints);
    }
  }
  else {
    OccupancyOctree* occ = (OccupancyOctree*)mOctree.get();
    occ->setOccupiedVoxelsAtPointsFromCloud(iCloud);
  }
  return true;
}

bool MapChunk::
remove(const PointCloud::Ptr& iCloud) {
  updateStructures();

  if (mStoreBackingPoints) {
    // TODO: is there a better way? what should the behavior be?
    // TODO: how do we know which points were removed by this operation?
    for (int i = 0; i < iCloud->size(); ++i) {
      mOctree->deleteVoxelAtPoint(iCloud->points[i]);
    }
  }
  else {
    for (int i = 0; i < iCloud->size(); ++i) {
      mOctree->deleteVoxelAtPoint(iCloud->points[i]);
    }
  }
  return true;
}

void MapChunk::
deepCopy(const MapChunk& iChunk) {
  *this = iChunk;
  mNeedsUpdate = true;
  if (!iChunk.mNeedsUpdate) {
    updateStructures();
    *mOctree = *(iChunk.mOctree);
    *mBackingPoints = *(iChunk.mBackingPoints);
  }
}

MapChunk::PointCloud::Ptr MapChunk::
getAsPointCloud(const bool iVoxelCenters) const {
  PointCloud::Ptr cloud(new PointCloud());

  if (mNeedsUpdate) {
    return cloud;
  }

  if (iVoxelCenters || !mStoreBackingPoints) {
    Octree::AlignedPointTVector vect;
    mOctree->getOccupiedVoxelCenters(vect);
    // TODO: is there a way to set this in one shot rather than loop?
    for (int i = 0; i < vect.size(); ++i) {
      cloud->push_back(vect[i]);
    }
  }
  else {
    if (mOctree->getIndices() != NULL) {
      // TODO: do we need to filter these?
      pcl::ExtractIndices<PointCloud::PointType> filt(true);
      filt.setInputCloud(mOctree->getInputCloud());
      filt.setIndices(mOctree->getIndices());
      filt.filter(*cloud);
    }
    else {
      *cloud = *(mOctree->getInputCloud());
    }
  }
  return cloud;
}


// TODO: different method required if using original points rather than voxels
bool MapChunk::
findDifferences(const MapChunk& iMap, PointCloud::Ptr& oAdded,
                PointCloud::Ptr& oRemoved) {

  updateStructures();

  // find newly added points
  if (oAdded == NULL) {
    oAdded.reset(new PointCloud());
  }
  oAdded->clear();
  double boundMinX, boundMinY, boundMinZ, boundMaxX, boundMaxY, boundMaxZ;
  iMap.mOctree->getBoundingBox(boundMinX, boundMinY, boundMinZ,
                               boundMaxX, boundMaxY, boundMaxZ);
  double resolution = iMap.mOctree->getResolution();
  Eigen::Vector3d boundMin(boundMinX, boundMinY, boundMinZ);
  Octree::LeafNodeIterator iter(*iMap.mOctree);
  for (; (*iter) != NULL; ++iter) {
    pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
    if (mOctree->existLeaf(key.x, key.y, key.z)) {
      continue;
    }

    Eigen::Vector3d pt(key.x, key.y, key.z);
    pt = (pt.array() + 0.5) * resolution;
    pt += boundMin;
    oAdded->push_back(PointCloud::PointType(pt[0], pt[1], pt[2]));
  }

  // find newly removed points
  if (oRemoved == NULL) {
    oRemoved.reset(new PointCloud());
  }
  oRemoved->clear();
  mOctree->getBoundingBox(boundMinX, boundMinY, boundMinZ,
                          boundMaxX, boundMaxY, boundMaxZ);
  resolution = mOctree->getResolution();
  boundMin = Eigen::Vector3d(boundMinX, boundMinY, boundMinZ);
  
  for (iter = Octree::LeafNodeIterator(*mOctree); (*iter) != NULL; ++iter) {
    pcl::octree::OctreeKey key = iter.getCurrentOctreeKey();
    if (iMap.mOctree->existLeaf(key.x, key.y, key.z)) {
      continue;
    }

    Eigen::Vector3d pt(key.x, key.y, key.z);
    pt = (pt.array() + 0.5) * resolution;
    pt += boundMin;
    oRemoved->push_back(PointCloud::PointType(pt[0], pt[1], pt[2]));
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
