#ifndef _maps_VoxelGridView_hpp_
#define _maps_VoxelGridView_hpp_

#include "ViewBase.hpp"

namespace maps {

class OccupancyGrid;

class VoxelGridView : public ViewBase {
public:
  typedef boost::shared_ptr<VoxelGridView> Ptr;
public:
  VoxelGridView();
  ~VoxelGridView();

  void setResolution(const float iResX, const float iResY, const float iResZ);
  boost::shared_ptr<OccupancyGrid> getGrid() const;

  const Type getType() const;
  ViewBase::Ptr clone() const;
  void set(const maps::PointCloud::Ptr& iCloud);
  maps::PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;
  maps::TriangleMesh::Ptr getAsMesh(const bool iTransform=true) const;
  bool getClosest(const Eigen::Vector3f& iPoint,
                  Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal);

protected:
  boost::shared_ptr<OccupancyGrid> mGrid;
};

}

#endif
