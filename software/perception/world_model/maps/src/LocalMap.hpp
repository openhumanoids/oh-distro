#ifndef _LocalMap_hpp_
#define _LocalMap_hpp_

#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>

#include <lcmtypes/octomap/raw_t.hpp>

class LocalMap {
public:
  typedef boost::shared_ptr<LocalMap> Ptr;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef octomap::OcTree Octree;
  typedef boost::shared_ptr<Octree> OctreePtr;

public:
  // TODO: abstract this later
  struct HeightMap {
    int mWidth;
    int mHeight;
    std::vector<float> mData;
    Eigen::Affine3d mTransformToLocal;
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
  bool add(const PointCloud::Ptr& iPoints,
           const Eigen::Isometry3d& iToLocal,
           const bool iRayTraceFromOrigin=false);

  // export this entire representation as an ordinary point cloud
  PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;

  // export this representation as height map
  // TODO: set desired resolution (perhaps as integer power of 2 factor)
  HeightMap getAsHeightMap(const int iDownSample=1,
                           const float iMaxHeight=1e20) const;

  // export to viewable lcm type
  octomap::raw_t getAsRaw() const;

  // for change detection
  void resetChangeReference();
  void getChanges(PointCloud::Ptr& oAdded, PointCloud::Ptr& oRemoved);
  void applyChanges(const PointCloud::Ptr& iAdded,
                    const PointCloud::Ptr& iRemoved);

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
