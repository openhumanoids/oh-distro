#include "MapManager.hpp"

#include "PointDataBuffer.hpp"
#include "LocalMap.hpp"

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
createMap(const Eigen::Isometry3d& iToLocal, const int iId) {
  LocalMap::Ptr localMap(new LocalMap());
  if (iId < 0) {
    localMap->setId(mNextMapId);
    ++mNextMapId;
  }
  else {
    localMap->setId(iId);
    mNextMapId = std::max(mNextMapId, iId+1);
  }
  localMap->setTransformToLocal(iToLocal);
  localMap->setBounds(-mMapDimensions/2, mMapDimensions/2);
  localMap->setResolution(mMapResolution);
  mMaps[localMap->getId()] = localMap;
  mActiveMap = localMap;

  std::cout << "MapManager: added new map, id=" << localMap->getId() <<
    std::endl;

  return true;
}

bool MapManager::
hasMap(const int64_t iId) {
  return (mMaps.find(iId) != mMaps.end());
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

LocalMap::Ptr MapManager::
getActiveMap() const {
  return mActiveMap;
}

bool MapManager::
addToBuffer(const int64_t iTime, const PointCloud::Ptr& iPoints,
            const Eigen::Isometry3d& iToLocal) {
  PointDataBuffer::PointSet pointSet;
  pointSet.mTimestamp = iTime;
  pointSet.mPoints = iPoints;
  pointSet.mToLocal = iToLocal;
  mPointDataBuffer->add(pointSet);
  std::cout << "MapManager: added " << iPoints->size() <<
    " points to buffer" << std::endl;
}

bool MapManager::
updatePose(const int64_t iTime, const Eigen::Isometry3d& iToLocal) {
  return mPointDataBuffer->update(iTime, iToLocal);
}

bool MapManager::
fuseAll() {
  if (mActiveMap == NULL) {
    return false;
  }

  mActiveMap->clear();
  PointDataBuffer::PointSetGroup::const_iterator iter;
  for (iter = mPointDataBuffer->begin();
       iter != mPointDataBuffer->end(); ++iter) {
    const PointDataBuffer::PointSet& pointSet = iter->second;
    mActiveMap->add(pointSet.mPoints, pointSet.mToLocal);
  }

  return true;
}

bool MapManager::
computeDelta(MapDelta& oDelta) {
  if (mActiveMap == NULL) {
    return false;
  }
  oDelta.mAdded.reset(new PointCloud());
  oDelta.mRemoved.reset(new PointCloud());
  mActiveMap->getChanges(oDelta.mAdded, oDelta.mRemoved);
  std::cout << "MapManager: found delta: " << oDelta.mAdded->size() <<
    " added, " << oDelta.mRemoved->size() << " removed" << std::endl;
  return true;
}

bool MapManager::
resetDeltaBase() {
  if (mActiveMap == NULL) {
    return false;
  }
  mActiveMap->resetChangeReference();
  std::cout << "MapManager: reset delta base" << std::endl;
  return true;
}
