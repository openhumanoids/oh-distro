#include "PointDataBuffer.hpp"

#include <pcl/io/io.h>

// TODO: may want to add internal queue to immediately handle "add" calls

PointDataBuffer::
PointDataBuffer() {
  setMaxLength(1000);
}

PointDataBuffer::
~PointDataBuffer() {
}

void PointDataBuffer::
setMaxLength(const int iLength) {
  mMaxLength = iLength;
  boost::mutex::scoped_lock lock(mMutex);
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
  boost::mutex::scoped_lock lock(mMutex);
  mData.clear();
  mTimes.clear();
}

void PointDataBuffer::
add(const PointSet& iData) {
  boost::mutex::scoped_lock lock(mMutex);
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

bool PointDataBuffer::
update(const int64_t iTimestamp, const Eigen::Isometry3d& iToLocal) {
  boost::mutex::scoped_lock lock(mMutex);
  PointSetGroup::iterator item = mData.find(iTimestamp);
  if (item == mData.end()) {
    return false;
  }
  item->second.mToLocal = iToLocal;
  return true;
}

PointDataBuffer::PointSet PointDataBuffer::
get(const int64_t iTimestamp) {
  boost::mutex::scoped_lock lock(mMutex);
  PointSetGroup::const_iterator item = mData.find(iTimestamp);
  PointSet pointSet;
  if (item == mData.end()) {
    pointSet.mTimestamp = 0;
    return pointSet;
  }
  pointSet = item->second;
  pcl::copyPointCloud(*(item->second.mPoints), *pointSet.mPoints);
  return pointSet;
}

std::vector<PointDataBuffer::PointSet> PointDataBuffer::
get(const int64_t iTimestamp1, const int64_t iTimestamp2) {
  boost::mutex::scoped_lock lock(mMutex);
  std::vector<PointSet> pointSets;
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
      pcl::copyPointCloud(*(item->second.mPoints), *curSet.mPoints);
      pointSets.push_back(curSet);
    }
  }
  return pointSets;
}

maptypes::PointCloud::Ptr PointDataBuffer::
getAsCloud(const int64_t iTimestamp1, const int64_t iTimestamp2) {
  std::vector<PointSet> pointSets = get(iTimestamp1, iTimestamp2);
  maptypes::PointCloud::Ptr cloud(new maptypes::PointCloud());
  for (int i = 0; i < pointSets.size(); ++i) {
    Eigen::Isometry3d xform = pointSets[i].mToLocal;
    for (int j = 0; j < pointSets[i].mPoints->points.size(); ++j) {
      maptypes::PointCloud::PointType point = pointSets[i].mPoints->points[j];
      Eigen::Vector3d p = xform*Eigen::Vector3d(point.x, point.y, point.z);
      point.x = p(0);
      point.y = p(1);
      point.z = p(2);
      cloud->points.push_back(point);
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}


bool PointDataBuffer::
lock() {
  mMutex.lock();
  return true;
}

bool PointDataBuffer::
unlock() {
  mMutex.unlock();
  return true;
}
  

// TODO: not threadsafe
PointDataBuffer::PointSetGroup::const_iterator PointDataBuffer::
begin() const {
  return mData.begin();
}


PointDataBuffer::PointSetGroup::const_iterator PointDataBuffer::
end() const {
  return mData.end();
}
