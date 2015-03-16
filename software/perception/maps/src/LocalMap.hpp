#ifndef _maps_LocalMap_hpp_
#define _maps_LocalMap_hpp_

#include <memory>
#include <deque>
#include <mutex>
#include <Eigen/Geometry>

#include "Types.hpp"

namespace maps {

class PointDataBuffer;
class PointCloudView;
class OctreeView;
class DepthImageView;
class ScanBundleView;
class LidarScan;

class LocalMap {
public:
  typedef std::shared_ptr<LocalMap> Ptr;

  // structure for specifying map params
  struct Spec {
    int64_t mId;
    int mPointBufferSize;
    bool mActive;
    float mResolution;

    Spec() {
      mId = -1;
      mPointBufferSize = 1000;
      mActive = true;
      mResolution = 0.1;
    }
  };

  // abstract class for filtering points
  class Filter {
  public:
    typedef std::shared_ptr<Filter> Ptr;
    virtual void operator()(LidarScan& ioScan) {}
    virtual void operator()(maps::PointSet& ioPoints) {}
  };

  // class for filtering scans based on ranges
  class RangeFilter : public Filter {
  protected:
    float mRangeMin;
    float mRangeMax;
  public:
    RangeFilter();
    void setValidRanges(const float iMin, const float iMax);
    void operator()(LidarScan& ioScan);
  };

  // class for filtering scans based on range differentials
  class RangeDiffFilter : public Filter {
  protected:
    float mDiffMax;
    float mRangeMax;
  public:
    RangeDiffFilter();
    void set(const float iDiffMax, const float iRangeMax);
    void operator()(LidarScan& ioScan);
  };

  // class for filtering scans based on line of sight angles
  class RangeAngleFilter : public Filter {
  protected:
    float mThetaMin;
  public:
    RangeAngleFilter();
    void set(const float iThetaMin);
    void operator()(LidarScan& ioScan);
  };

  // structure for specifying data volume in space and time
  struct SpaceTimeBounds {
    std::vector<Eigen::Vector4f> mPlanes;
    int64_t mTimeMin;
    int64_t mTimeMax;

  public:
    SpaceTimeBounds();
    SpaceTimeBounds(const std::vector<Eigen::Vector4f>& iPlanes,
                    const int64_t iMinTime=-1, const int64_t iMaxTime=-1);

    void set(const std::vector<Eigen::Vector4f>& iPlanes,
             const int64_t iMinTime=-1, const int64_t iMaxTime=-1);
  };

  
public:
  LocalMap(const Spec& iSpec);
  virtual ~LocalMap();

  // clear all data
  void clear();

  int64_t getId() const;
  int64_t getStateId() const;
  int getMaxPointDataBufferSize() const;
  Spec getSpec() const;

  // set/get whether this map should reject additional data
  void setActive(const bool iVal);
  bool isActive() const;

  // add filters
  void addFilter(const std::shared_ptr<Filter>& iFilter);

  // transform points to reference frame, crop against bounds, and add
  bool addData(const maps::PointSet& iPointSet);
  bool addData(const LidarScan& iScan);

  // get point data buffer
  const std::shared_ptr<PointDataBuffer> getPointData() const;

  // get scan data buffer
  std::deque<std::shared_ptr<LidarScan> > getScanData() const;

  // export this entire representation as an ordinary point cloud
  std::shared_ptr<PointCloudView>
  getAsPointCloud(const float iResolution=0,
                  const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;

  // export this entire representation as an octree
  std::shared_ptr<OctreeView>
  getAsOctree(const float iResolution, const bool iTraceRays=false,
              const Eigen::Vector3f& iOrigin=Eigen::Vector3f(0,0,0),
              const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;

  // export this entire representation as a depth image
  std::shared_ptr<DepthImageView>
  getAsDepthImage(const int iWidth, const int iHeight,
                  const Eigen::Projective3f& iProjector,
                  const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;
  std::shared_ptr<DepthImageView>
  getAsDepthImage(const int iWidth, const int iHeight,
                  const Eigen::Projective3f& iProjector,
                  const int iAccumMethod,
                  const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;

  // export this entire representation as a scan bundle
  std::shared_ptr<ScanBundleView>
  getAsScanBundle(const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;

protected:
  int64_t mStateId;
  Spec mSpec;
  bool mIsFrozen;
  std::shared_ptr<PointDataBuffer> mPointData;
  std::deque<std::shared_ptr<LidarScan> > mScanData;
  std::vector<Filter::Ptr> mFilters;
  mutable std::mutex mScanMutex;
};

}

#endif
