#include "MapManager.hpp"

#include "PointDataBuffer.hpp"
#include "MapChunk.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

MapManager::
MapManager() {
  mPointDataBuffer.reset(new PointDataBuffer());
  setMapResolution(0.01);
  setMapDimensions(Eigen::Vector3d(10,10,10));
  setDataBufferLength(1000);
  mNextMapId = 1;
  std::cout << "MapManager: constructed" << std::endl;
}

MapManager::
~MapManager() {
  std::cout << "MapManager: destructed" << std::endl;
}

void MapManager::
clear() {
  mActiveMap.reset();
  mActiveMapPrev.reset();
  mMaps.clear();
  mPointDataBuffer->clear();
  std::cout << "MapManager: cleared all state" << std::endl;
}

void MapManager::
setMapResolution(const double iResolution) {
  mMapResolution = iResolution;
  std::cout << "MapManager: map resolution set to " << iResolution <<
    std::endl;
}

void MapManager::
setMapDimensions(const Eigen::Vector3d iDims) {
  mMapDimensions = iDims;
  std::cout << "MapManager: map dimensions set to (" <<
    iDims[0] << "," << iDims[1] << "," << iDims[2] << ")" << std::endl;
}

void MapManager::
setDataBufferLength(const int iLength) {
  mDataBufferLength = iLength;
  std::cout << "MapManager: point cloud buffer length set to " <<
    iLength << std::endl;
}


bool MapManager::
createMap(const Eigen::Isometry3d& iToLocal) {
  MapPtr chunk(new MapChunk());
  chunk->setId(mNextMapId);
  ++mNextMapId;
  chunk->setTransformToLocal(iToLocal);
  chunk->setBounds(-mMapDimensions/2, mMapDimensions/2);
  chunk->setResolution(mMapResolution);
  mActiveMap = chunk;

  mActiveMapPrev.reset(new MapChunk());
  mActiveMapPrev->deepCopy(*mActiveMap);

  std::cout << "MapManager: added new map, id=" << chunk->getId() <<
    std::endl;

  return true;
}

bool MapManager::
useMap(const int64_t iId) {
  MapCollection::iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    std::cout << "MapManager: could not find requested map with id=" << iId <<
      std::endl;
    return false;
  }
  mActiveMap = item->second;
  std::cout << "MapManager: switched to map " << item->second->getId() <<
    std::endl;
  return true;
}

int64_t MapManager::
getActiveMapId() const {
  return (mActiveMap == NULL) ? -1 : mActiveMap->getId();
}

bool MapManager::
clearActiveMap() {
  if (mActiveMap == NULL) {
    return false;
  }
  mActiveMap->clear();
  std::cout << "MapManager: cleared active map" << std::endl;
  return true;
}

bool MapManager::
add(const int64_t iTime, const PointCloud& iPoints,
    const Eigen::Isometry3d& iToLocal, const bool iBuffer) {
  if (iBuffer) {
    PointDataBuffer::PointSet pointSet;
    pointSet.mTimestamp = iTime;
    pointSet.mPoints.reset(new PointCloud(iPoints));
    pointSet.mToLocal = iToLocal;
    mPointDataBuffer->add(pointSet);
    std::cout << "MapManager: added " << iPoints.size() <<
      " points to buffer" << std::endl;
  }
  if (mActiveMap == NULL) {
    return false;
  }

  // transform points
  PointCloud::Ptr points(new PointCloud);
  Eigen::Isometry3d chunkToLocal = mActiveMap->getTransformToLocal();
  Eigen::Affine3f matx((chunkToLocal.inverse()*iToLocal).cast<float>());
  pcl::transformPointCloud(iPoints, *points, matx);

  // crop points
  pcl::CropBox<PointCloud::PointType> cropper;
  Eigen::Vector3d boundMin = mActiveMap->getBoundMin();
  Eigen::Vector3d boundMax = mActiveMap->getBoundMax();
  cropper.setMin(Eigen::Vector4f(boundMin[0], boundMin[1], boundMin[2], 1));
  cropper.setMax(Eigen::Vector4f(boundMax[0], boundMax[1], boundMax[2], 1));
  cropper.setInputCloud(points);
  PointCloud pointsFinal;
  cropper.filter(pointsFinal);
  
  // add points
  mActiveMap->add(pointsFinal);
  std::cout << "MapManager: added " << pointsFinal.size() <<
    " points to current map" << std::endl;  
  return true;
}

bool MapManager::
removeFromMap(const PointCloud& iCloud) {
  if (mActiveMap == NULL) {
    return false;
  }
  PointCloud newCloud;
  Eigen::Affine3f matx(mActiveMap->getTransformToLocal().
                       inverse().cast<float>());
  pcl::transformPointCloud(iCloud, newCloud, matx);
  mActiveMap->remove(newCloud);
  return true;
}

bool MapManager::
fuseAll() {
  if (mActiveMap == NULL) {
    return false;
  }

  clearActiveMap();
  PointDataBuffer::PointSetGroup::const_iterator iter;
  for (iter = mPointDataBuffer->begin();
       iter != mPointDataBuffer->end(); ++iter) {
    const PointDataBuffer::PointSet& pointSet = iter->second;
    add(pointSet.mTimestamp, *pointSet.mPoints, pointSet.mToLocal, false);
  }

  return true;
}

bool MapManager::
computeDelta(MapDelta& oDelta) {
  if ((mActiveMap == NULL) || (mActiveMapPrev == NULL)) {
    return false;
  }
  oDelta.mCurrentTime = mActiveMap->getLastUpdateTime();
  oDelta.mPreviousTime = mActiveMapPrev->getLastUpdateTime();
  mActiveMapPrev->findDifferences(*mActiveMap, oDelta.mAdded, oDelta.mRemoved);
  std::cout << "MapManager: found delta: " << oDelta.mAdded.size() <<
    " added, " << oDelta.mRemoved.size() << " removed" << std::endl;
    
  return true;
}

MapManager::PointCloud::Ptr MapManager::
getPointCloud() const {
  PointCloud::Ptr pts;
  if (mActiveMap != NULL) {
    pts = mActiveMap->getAsPointCloud();
    Eigen::Affine3f matx(mActiveMap->getTransformToLocal().cast<float>());
    pcl::transformPointCloud(*pts, *pts, matx);    
  }
  return pts;
}

bool MapManager::
resetDelta() {
  if ((mActiveMap == NULL) || (mActiveMapPrev == NULL)) {
    return false;
  }
  mActiveMapPrev->deepCopy(*mActiveMap);
  std::cout << "MapManager: reset delta" << std::endl;
  return true;
}

void MapManager::
serialize(std::vector<char>& oBytes) const {
  // TODO
}

void MapManager::
deserialize(const std::vector<char>& iBytes) {
  // TODO
}
