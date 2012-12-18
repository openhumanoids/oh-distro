#ifndef _PointDataBuffer_hpp_
#define _PointDataBuffer_hpp_

#include <unordered_map>
#include <set>
#include <Eigen/Geometry>
#include <boost/thread/mutex.hpp>

#include "MapTypes.hpp"

class PointDataBuffer {
public:
  struct PointSet {
    int64_t mTimestamp;
    maptypes::PointCloud::Ptr mPoints;
    Eigen::Isometry3d mToLocal;
  };

  typedef std::unordered_map<int64_t, PointSet> PointSetGroup;
  typedef std::set<int64_t> TimeGroup;

public:
  PointDataBuffer();
  ~PointDataBuffer();

  void setMaxLength(const int iLength);
  int getMaxLength() const;

  void clear();
  void add(const PointSet& iData);
  bool update(const int64_t iTimestamp, const Eigen::Isometry3d& iToLocal);

  PointSet get(const int64_t iTimestamp);
  std::vector<PointSet> get(const int64_t iTimestamp1,
                            const int64_t iTimestamp2);

  maptypes::PointCloud::Ptr getAsCloud(const int64_t iTimestamp1,
                                       const int64_t iTimestamp2);

  bool lock();
  bool unlock();

  PointSetGroup::const_iterator begin() const;
  PointSetGroup::const_iterator end() const;

protected:
  int mMaxLength;
  PointSetGroup mData;
  TimeGroup mTimes;

  boost::mutex mMutex;
};

#endif
