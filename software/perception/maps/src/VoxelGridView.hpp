#ifndef _maps_VoxelGridView_hpp_
#define _maps_VoxelGridView_hpp_

#include "ViewBase.hpp"

namespace maps {

class OccupancyGrid;

class VoxelGridView : public ViewBase {
public:
  typedef std::shared_ptr<VoxelGridView> Ptr;
public:
  VoxelGridView();
  ~VoxelGridView();

  void setResolution(const float iResX, const float iResY, const float iResZ);
  std::shared_ptr<OccupancyGrid> getGrid() const;

  const Type getType() const;
  ViewBase::Ptr clone() const;
  void set(const maps::PointCloud::Ptr& iCloud);
  maps::PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;

protected:
  std::shared_ptr<OccupancyGrid> mGrid;
};

}

#endif
