#include "PointCloudView.hpp"

#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace maps;

PointCloudView::
PointCloudView() {
  mCloud.reset(new maps::PointCloud());
  mResolution = 0;
}

PointCloudView::
~PointCloudView() {
}

void PointCloudView::
setResolution(const float iResolution) {
  mResolution = iResolution;
}

maps::PointCloud::Ptr PointCloudView::
getPointCloud() const {
  return mCloud;
}

const ViewBase::Type PointCloudView::
getType() const {
  return TypePointCloud;
}

ViewBase::Ptr PointCloudView::
clone() const {
  PointCloudView* view = new PointCloudView(*this);
  pcl::copyPointCloud(*mCloud, *view->mCloud);
  return ViewBase::Ptr(view);
}

void PointCloudView::
set(const maps::PointCloud::Ptr& iCloud) {
  if (mResolution > 0) {
    pcl::VoxelGrid<maps::PointCloud::PointType> grid;
    grid.setInputCloud(iCloud);
    grid.setLeafSize(mResolution,mResolution,mResolution);
    grid.filter(*mCloud);
  }
  else {
    pcl::copyPointCloud(*iCloud, *mCloud);
  }
}

maps::PointCloud::Ptr PointCloudView::
getAsPointCloud(const bool iTransform) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud(*mCloud));
  if (iTransform) {
    pcl::transformPointCloud(*cloud, *cloud,
                             Eigen::Affine3f(mTransform.matrix()).inverse());
  }
  /* NOTE: for projective transformation
  for (int i = 0; i < cloud->size(); ++i) {
    maps::PointCloud::PointType& pt = (*cloud)[i];
    Eigen::Vector4f projPt = mTransform*Eigen::Vector4f(pt.x,pt.y,pt.z,1);
    pt.getVector3fMap() = projPt.head<3>()/projPt[3];
  }
  */
  return cloud;
}
