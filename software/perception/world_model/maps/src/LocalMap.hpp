#ifndef _LocalMap_hpp_
#define _LocalMap_hpp_

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

#include "MapTypes.hpp"

namespace octomap {
  class OcTree;
}

class LocalMap {
public:
  typedef boost::shared_ptr<LocalMap> Ptr;
  typedef octomap::OcTree Octree;
  typedef boost::shared_ptr<Octree> OctreePtr;

public:
  // TODO: abstract this later
  struct HeightMap {
    int mWidth;
    int mHeight;
    float mMinZ;
    float mMaxZ;
    std::vector<float> mData;
    Eigen::Affine3d mTransformToLocal;
  };

  struct DepthMap {
    int mWidth;
    int mHeight;
    std::vector<float> mData;
    Eigen::Projective3d mTransform;  // local to image
  };

public:
  LocalMap();
  virtual ~LocalMap();

  // clear all state
  void clear();

  // set/get internal id
  void setId(const int64_t iId);
  int64_t getId() const;

  // set/get internal state id
  void setStateId(const int64_t iId);
  int64_t getStateId() const;

  // set/get transform to local frame
  void setTransformToLocal(const Eigen::Isometry3d& iTransform);
  Eigen::Isometry3d getTransformToLocal() const;

  // set/get 3d bounds for this volume (in map coord frame)
  bool setBounds(const Eigen::Vector3d& iMin, const Eigen::Vector3d& iMax);
  Eigen::Vector3d getBoundMin() const;
  Eigen::Vector3d getBoundMax() const;

  // set/get resolution of smallest octree voxels
  void setResolution(const double iResolution);
  double getResolution() const;

  // transform to local frame, add to current map
  bool add(const maptypes::PointCloud::Ptr& iPoints,
           const Eigen::Isometry3d& iToLocal,
           const bool iRayTraceFromOrigin=false);

  // export this entire representation as an ordinary point cloud
  maptypes::PointCloud::Ptr getAsPointCloud() const;

  // export this representation as height map
  HeightMap getAsHeightMap(const int iDownSample=1,
                           const float iMaxHeight=1e20) const;

  // export this representation as depth map
  DepthMap getAsDepthMap(const Eigen::Projective3d& iLocalToImage,
                         const int iWidth, const int iHeight) const;

  // export raw underlying octree bytes 
  void getAsRaw(std::vector<uint8_t>& oBytes) const;

  // for change detection
  void resetChangeReference();
  void getChanges(maptypes::PointCloud::Ptr& oAdded,
                  maptypes::PointCloud::Ptr& oRemoved);
  void applyChanges(const maptypes::PointCloud::Ptr& iAdded,
                    const maptypes::PointCloud::Ptr& iRemoved);

  // serialize to bytes
  void serialize(std::vector<char>& oBytes) const;

  // deserialize from bytes
  void deserialize(const std::vector<char>& iBytes);


protected:
  int64_t mId;
  int64_t mStateId;
  double mResolution;
  Eigen::Isometry3d mTransformToLocal;
  Eigen::Vector3d mBoundMin;
  Eigen::Vector3d mBoundMax;
  OctreePtr mOctree;
};

#endif
