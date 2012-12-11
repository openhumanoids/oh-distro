#ifndef _PointDataBuffer_hpp_
#define _PointDataBuffer_hpp_

#include <unordered_map>
#include <set>
#include <Eigen/Geometry>

#include "MapTypes.hpp"

class PointDataBuffer {
public:
  struct PointSet {
    int64_t mTimestamp;
    maptypes::PointCloud::Ptr mPoints;
    Eigen::Isometry3d mToLocal;
  };

  typedef std::unordered_map<int64_t, PointSet> PointSetGroup;


public:
  PointDataBuffer();
  ~PointDataBuffer();

  void setMaxLength(const int iLength);
  int getMaxLength() const;

  void clear();
  void add(const PointSet& iData);
  bool update(const int64_t iTimestamp, const Eigen::Isometry3d& iToLocal);

  PointSetGroup::const_iterator begin() const;
  PointSetGroup::const_iterator end() const;

protected:
  int mMaxLength;
  PointSetGroup mData;
  std::set<int64_t> mTimes;
};

#endif
