#include "PointDataBuffer.hpp"

#include "Utils.hpp"

#include <pcl/io/io.h>
#include <pcl/common/transforms.h>

// TODO: may want to add internal queue to immediately handle "add" calls

using namespace maps;

PointDataBuffer::
PointDataBuffer() {
  setMaxLength(1000);
}

PointDataBuffer::
~PointDataBuffer() {
}

void PointDataBuffer::
setMaxLength(const int iLength) {
  std::lock_guard<std::mutex> lock(mMutex);
  mMaxLength = iLength;
  if (mMaxLength >= 0) {
    while (mTimes.size() > mMaxLength) {
      TimeGroup::iterator iter = mTimes.begin();
      mData.erase(*iter);
      mTimes.erase(iter);
    }
  }
}

int PointDataBuffer::
getMaxLength() const {
  return mMaxLength;
}

void PointDataBuffer::
clear() {
  std::lock_guard<std::mutex> lock(mMutex);
  mData.clear();
  mTimes.clear();
}

void PointDataBuffer::
add(const PointSet& iData) {
  std::lock_guard<std::mutex> lock(mMutex);
  mData[iData.mTimestamp] = iData;
  mTimes.insert(iData.mTimestamp);
  if (mMaxLength >= 0) {
    while (mData.size() > mMaxLength) {
      TimeGroup::iterator iter = mTimes.begin();
      mData.erase(*iter);
      mTimes.erase(iter);
    }
  }
}

maps::PointSet PointDataBuffer::
get(const int64_t iTimestamp) {
  std::lock_guard<std::mutex> lock(mMutex);
  PointSetGroup::const_iterator item = mData.find(iTimestamp);
  maps::PointSet pointSet;
  if (item == mData.end()) {
    pointSet.mTimestamp = 0;
    return pointSet;
  }
  pointSet = item->second;
  return pointSet;
}

std::vector<maps::PointSet> PointDataBuffer::
get(const int64_t iTimestamp1, const int64_t iTimestamp2) {
  std::lock_guard<std::mutex> lock(mMutex);
  std::vector<maps::PointSet> pointSets;
  TimeGroup::const_iterator iter1 = (iTimestamp1 < 0) ? mTimes.begin() :
    mTimes.lower_bound(iTimestamp1);
  TimeGroup::const_iterator iter2 = (iTimestamp2 < 0) ? mTimes.end() :
    mTimes.upper_bound(iTimestamp2);
  TimeGroup::const_iterator iter;
  if (iter1 == mTimes.end()) {
    return pointSets;
  }

  for (iter = iter1; (iter != iter2) && (iter != mTimes.end()); ++iter) {
    PointSetGroup::const_iterator item = mData.find(*iter);
    if (item != mData.end()) {
      PointSet curSet = item->second;
      pointSets.push_back(curSet);
    }
  }
  return pointSets;
}

std::vector<maps::PointSet> PointDataBuffer::
getAll() {
  return get(-1, -1);
}

int64_t PointDataBuffer::
getTimeMin() const {
  if (mTimes.size() == 0) {
    return -1;
  }
  return *mTimes.begin();
}

int64_t PointDataBuffer::
getTimeMax() const {
  if (mTimes.size() == 0) {
    return -1;
  }
  return *mTimes.rbegin();
}

maps::PointCloud::Ptr PointDataBuffer::
getAsCloud(const int64_t iTimestamp1, const int64_t iTimestamp2) {
  std::vector<maps::PointSet> pointSets = get(iTimestamp1, iTimestamp2);
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  for (int i = 0; i < pointSets.size(); ++i) {
    Eigen::Affine3f xform = Utils::getPose(*pointSets[i].mCloud);
    maps::PointCloud curCloud;
    pcl::transformPointCloud(*pointSets[i].mCloud, curCloud, xform);
    *cloud += curCloud;
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

PointDataBuffer::Ptr PointDataBuffer::
clone() {
  std::lock_guard<std::mutex> lock(mMutex);
  std::shared_ptr<PointDataBuffer> buf(new PointDataBuffer());
  buf->mMaxLength = mMaxLength;
  buf->mData = mData;
  buf->mTimes = mTimes;
  return buf;
}
