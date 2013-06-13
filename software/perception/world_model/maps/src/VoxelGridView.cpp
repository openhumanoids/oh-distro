#include "VoxelGridView.hpp"

#include "VoxelGrid.hpp"

using namespace maps;

namespace maps {
class OccupancyGrid : public VoxelGrid<int8_t> {
};
}

VoxelGridView::
VoxelGridView() {
  mGrid.reset(new OccupancyGrid());
  // TODO: set resolution
}

VoxelGridView::
~VoxelGridView() {
}

void VoxelGridView::
setResolution(const float iResX, const float iResY, const float iResZ) {
  // TODO
}

std::shared_ptr<OccupancyGrid> VoxelGridView::
getGrid() const {
  return mGrid;
}

const ViewBase::Type VoxelGridView::
getType() const {
  return TypeVoxelGrid;
}

ViewBase::Ptr VoxelGridView::
clone() const {
  VoxelGridView* view = new VoxelGridView(*this);
  // TODO: copy grid
  return ViewBase::Ptr(view);
}

void VoxelGridView::
set(const maps::PointCloud::Ptr& iCloud) {
  // TODO
}

maps::PointCloud::Ptr VoxelGridView::
getAsPointCloud(const bool iTransform) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  // TODO: find max min extents
  
  // TODO: get cloud and transform
  return cloud;
}
