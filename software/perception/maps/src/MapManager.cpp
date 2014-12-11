#include "MapManager.hpp"

#include "PointDataBuffer.hpp"
#include "LocalMap.hpp"
#include "LidarScan.hpp"

using namespace maps;

MapManager::
MapManager() {
  mVerbose = false;
  mNextMapId = 1;
  mPointData.reset(new PointDataBuffer());
  mPointData->setMaxLength(1000);  // TODO: could make this a parameter
}

MapManager::
~MapManager() {
}

void MapManager::
clear() {
  mMaps.clear();
  std::cout << "MapManager: cleared all state" << std::endl;
}

void MapManager::
setVerbose(const bool iVal) {
  std::cout << "MapManager: verbose set to " <<
    (iVal ? "true" : "false") << std::endl;
  mVerbose = iVal;
}

int64_t MapManager::
createMap(const LocalMap::Spec& iSpec) {
  LocalMap::Spec spec = iSpec;
  if ((spec.mId < 0) || hasMap(spec.mId)) {
    spec.mId = mNextMapId;
  }
  LocalMap::Ptr localMap(new LocalMap(spec));
  mNextMapId = spec.mId+1;
  mMaps[spec.mId] = localMap;
  std::cout << "MapManager: created map id=" << spec.mId << std::endl;
  return spec.mId;
}

bool MapManager::
hasMap(const int64_t iId) const {
  return (mMaps.find(iId) != mMaps.end());
}

LocalMap::Ptr MapManager::
getMap(const int64_t iId) const {
  MapCollection::const_iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    return LocalMap::Ptr();
  }
  return item->second;
}

std::vector<int64_t> MapManager::
getAllMapIds(const bool iActiveOnly) const {
  std::vector<int64_t> ids;
  for (MapCollection::const_iterator iter = mMaps.begin();
       iter != mMaps.end(); ++iter) {
    if (!iActiveOnly || iter->second->isActive()) {
      ids.push_back(iter->first);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

bool MapManager::
startUpdatingMap(const int64_t iId) {
  MapCollection::const_iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    return false;
  }
  item->second->setActive(true);
  std::cout << "MapManager: start updating map id=" << iId << std::endl;
  return true;
}

bool MapManager::
stopUpdatingMap(const int64_t iId) {
  MapCollection::const_iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    return false;
  }
  item->second->setActive(false);
  std::cout << "MapManager: stop updating map id=" << iId << std::endl;
  return true;
}

bool MapManager::
clearMap(const int64_t iId) {
  MapCollection::const_iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    return false;
  }
  item->second->clear();
  std::cout << "MapManager: cleared map id=" << iId << std::endl;
  return true;
}

bool MapManager::
deleteMap(const int64_t iId) {
  MapCollection::const_iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    return false;
  }
  mMaps.erase(item);
  std::cout << "MapManager: deleted map id=" << iId << std::endl;
  return true;
}

int64_t MapManager::
snapshotMap(const int64_t iId) {
  return snapshotMap(iId, -1, -1);
}

int64_t MapManager::
snapshotMap(const int64_t iId,
            const int64_t iStartTime, const int64_t iEndTime) {
  MapCollection::const_iterator item = mMaps.find(iId);
  if (item == mMaps.end()) {
    return -1;
  }

  // create new map with matching spec
  LocalMap::Ptr localMap = item->second;
  LocalMap::Spec spec = localMap->getSpec();
  spec.mActive = false;
  int64_t idNew = createMap(spec);

  // populate data
  std::vector<maps::PointSet> pointSets =
    localMap->getPointData()->get(iStartTime, iEndTime);
  LocalMap::Ptr localMapNew = mMaps[idNew];
  std::shared_ptr<PointDataBuffer> pointData = localMapNew->getPointData();
  pointData->clear();
  for (int i = 0; i < pointSets.size(); ++i) {
    pointData->add(pointSets[i]);
  }

  std::cout << "MapManager: snapshot map id=" << iId << std::endl;
  return idNew;
}

bool MapManager::
addData(const maps::PointSet& iPointSet, const int64_t iMapId) {
  // add to internal point buffer
  mPointData->add(iPointSet);

  // add to maps
  MapCollection::iterator iter;
  for (iter = mMaps.begin(); iter != mMaps.end(); ++iter) {
    LocalMap::Ptr localMap = iter->second;
    if (!localMap->isActive()) {
      continue;
    }
    if ((iMapId <= 0) || (localMap->getId() == iMapId)) {
      localMap->addData(iPointSet);
    }
  }
  return true;
}

bool MapManager::
addData(const maps::LidarScan& iScan, const int64_t iMapId) {
  // add to internal point buffer
  PointSet pointSet;
  iScan.get(pointSet);
  mPointData->add(pointSet);

  // add to maps
  MapCollection::iterator iter;
  for (iter = mMaps.begin(); iter != mMaps.end(); ++iter) {
    LocalMap::Ptr localMap = iter->second;
    if (!localMap->isActive()) {
      continue;
    }
    if ((iMapId <= 0) || (localMap->getId() == iMapId)) {
      localMap->addData(iScan);
    }
  }
  return true;
}

const std::shared_ptr<PointDataBuffer> MapManager::
getPointData() const {
  return mPointData;
}
