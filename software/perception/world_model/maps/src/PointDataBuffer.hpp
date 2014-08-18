#ifndef _maps_PointDataBuffer_hpp_
#define _maps_PointDataBuffer_hpp_

#include <unordered_map>
#include <set>
#include <thread>
#include <memory>
#include <mutex>
#include <Eigen/Geometry>

#include "Types.hpp"

namespace maps {

class PointDataBuffer {
public:
  typedef std::shared_ptr<PointDataBuffer> Ptr;

protected:
  typedef std::unordered_map<int64_t, PointSet> PointSetGroup;
  typedef std::set<int64_t> TimeGroup;

public:
  PointDataBuffer();
  ~PointDataBuffer();

  void setMaxLength(const int iLength);
  int getMaxLength() const;

  void clear();
  void add(const PointSet& iData);

  PointSet get(const int64_t iTimestamp);
  std::vector<PointSet> get(const int64_t iTimestamp1,
                            const int64_t iTimestamp2);
  std::vector<PointSet> getAll();

  int64_t getTimeMin() const;
  int64_t getTimeMax() const;

  maps::PointCloud::Ptr getAsCloud(const int64_t iTimestamp1,
                                   const int64_t iTimestamp2);

  Ptr clone();

protected:
  int mMaxLength;
  PointSetGroup mData;
  TimeGroup mTimes;

  std::mutex mMutex;
};

}

#endif
