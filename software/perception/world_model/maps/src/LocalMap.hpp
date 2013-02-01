#ifndef _maps_LocalMap_hpp_
#define _maps_LocalMap_hpp_

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

#include "Types.hpp"

namespace octomap {
  class OcTree;
}

namespace maps {

class PointDataBuffer;

class LocalMap {
public:
  typedef boost::shared_ptr<LocalMap> Ptr;

  // structure for specifying map params
  struct Spec {
    int64_t mId;
    Eigen::Vector3f mBoundMin;
    Eigen::Vector3f mBoundMax;
    int mPointBufferSize;
    bool mActive;
    float mOctreeResolution;

    Spec() {
      mId = -1;
      mBoundMin = Eigen::Vector3f(-1e10, -1e10, -1e10);
      mBoundMax = Eigen::Vector3f(1e10, 1e10, 1e10);
      mPointBufferSize = 1000;
      mActive = true;
      mOctreeResolution = 0.1;
    }
  };


  // structure for specifying data volume in space and time
  struct SpaceTimeBounds {
    std::vector<Eigen::Vector4f> mPlanes;
    int64_t mMinTime;
    int64_t mMaxTime;

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
  Eigen::Vector3f getBoundMin() const;
  Eigen::Vector3f getBoundMax() const;
  std::vector<Eigen::Vector4f> getBoundPlanes() const;  // TODO: may not need
  Spec getSpec() const;

  // set/get whether this map should reject additional data
  void setActive(const bool iVal);
  bool isActive() const;

  // transform points to reference frame, crop against bounds, and add
  bool addData(const maps::PointSet& iPointSet);

  // get point data buffer
  const boost::shared_ptr<PointDataBuffer> getPointData() const;

  // export this entire representation as an ordinary point cloud
  maps::PointCloud::Ptr
  getAsPointCloud(const float iResolution=0,
                  const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;

  // export this entire representation as an octree
  maps::Octree
  getAsOctree(const float iResolution, const bool iTraceRays=false,
              const Eigen::Vector3f& iShift=Eigen::Vector3f(0,0,0),
              const SpaceTimeBounds& iBounds=SpaceTimeBounds()) const;

  /*
  // export this representation as height map
  HeightMap getAsHeightMap(const int iDownSample=1,
                           const float iMaxHeight=1e20) const;

  // export this representation as depth map
  DepthMap getAsDepthMap(const Eigen::Projective3d& iLocalToImage,
                         const int iWidth, const int iHeight) const;
  */


protected:
  int64_t mStateId;
  Spec mSpec;
  bool mIsFrozen;
  boost::shared_ptr<PointDataBuffer> mPointData;
  maps::Octree mOctree;
};

}

#endif
