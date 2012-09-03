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
}

MapManager::
~MapManager() {
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
}

void MapManager::
setMapDimensions(const Eigen::Vector3d iDims) {
  mMapDimensions = iDims;
}

void MapManager::
setDataBufferLength(const int iLength) {
  mDataBufferLength = iLength;
}


bool MapManager::
createMap(const Eigen::Affine3d& iToLocal) {
  MapPtr chunk(new MapChunk());
  chunk->setId(mNextMapId);
  ++mNextMapId;
  chunk->setTransformToLocal(iToLocal);
  chunk->setBounds(-mMapDimensions/2, mMapDimensions/2);
  chunk->setResolution(mMapResolution);
  mActiveMap = chunk;
  std::cout << "MapManager: added new map, id=" << chunk->getId() <<
    std::endl;

  mActiveMapPrev.reset(new MapChunk());
  mActiveMapPrev->deepCopy(*mActiveMap);

  return true;
}

bool MapManager::
useMap(const int64_t iId) {
  MapCollection::iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
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
  return true;
}

bool MapManager::
add(const int64_t iTime, const PointCloud& iPoints,
    const Eigen::Affine3d& iToLocal, const bool iBuffer) {
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
  Eigen::Affine3d chunkToLocal = mActiveMap->getTransformToLocal();
  Eigen::Affine3f matx(chunkToLocal.inverse()*iToLocal);
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
  Eigen::Affine3f matx(mActiveMap->getTransformToLocal().inverse());
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
  return true;
}

bool MapManager::
resetDelta() {
  if ((mActiveMap == NULL) || (mActiveMapPrev == NULL)) {
    return false;
  }
  mActiveMapPrev->deepCopy(*mActiveMap);
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
