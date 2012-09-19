#ifndef _MapChunk_hpp_
#define _MapChunk_hpp_

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include "OctreeCustom.hpp"

class MapChunk {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef OctreeCustom<pcl::PointXYZ> Octree;
  typedef boost::shared_ptr<Octree> OctreePtr;

public:
  // constructor/destructor
  MapChunk();
  ~MapChunk();

  // clear all state
  void clear();

  // set/get internal id
  void setId(const int64_t iId);
  int64_t getId() const;

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

  // buffer input points, transform to local frame, add to current map
  bool add(const PointCloud::Ptr& iPoints,
           const Eigen::Isometry3d& iToLocal);

  // remove a set of points
  bool remove(const PointCloud::Ptr& iCloud);

  // copy contents of input map into this one
  void deepCopy(const MapChunk& iChunk);

  // export this entire representation as an ordinary point cloud
  PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;

  // determine points that were added and removed wrt input map
  bool findDifferences(const MapChunk& iMap, PointCloud::Ptr& oAdded,
                       PointCloud::Ptr& oRemoved);

  // serialize to bytes
  void serialize(std::vector<char>& oBytes) const;

  // deserialize from bytes
  void deserialize(const std::vector<char>& iBytes);

protected:
  int64_t mId;
  double mResolution;
  Eigen::Isometry3d mTransformToLocal;
  Eigen::Vector3d mBoundMin;
  Eigen::Vector3d mBoundMax;
  OctreePtr mOctree;
};

#endif
