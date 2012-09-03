#include "PointDataBuffer.hpp"

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
}

int PointDataBuffer::
getMaxLength() const {
  return mMaxLength;
}

void PointDataBuffer::
clear() {
  mData.clear();
  mTimes.clear();
}

void PointDataBuffer::
add(const PointSet& iData) {
  mData[iData.mTimestamp] = iData;
  mTimes.insert(iData.mTimestamp);
  while (mData.size() > mMaxLength) {
    std::set<int64_t>::iterator iter = mTimes.begin();
    mData.erase(*iter);
    mTimes.erase(iter);
  }
}

PointDataBuffer::PointSetGroup::const_iterator PointDataBuffer::
begin() const {
  return mData.begin();
}


PointDataBuffer::PointSetGroup::const_iterator PointDataBuffer::
end() const {
  return mData.end();
}
