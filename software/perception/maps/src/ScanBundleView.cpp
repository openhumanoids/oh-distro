#include "ScanBundleView.hpp"

#include <pcl/common/transforms.h>

using namespace maps;

ScanBundleView::
ScanBundleView() {
}

ScanBundleView::
~ScanBundleView() {
}

void ScanBundleView::
set(const std::vector<maps::LidarScan::Ptr>& iScans) {
  mScans = iScans;
}

std::vector<maps::LidarScan::Ptr> ScanBundleView::
getScans() const {
  return mScans;
}

int ScanBundleView::
getNumScans() const {
  return mScans.size();
}

const ViewBase::Type ScanBundleView::
getType() const {
  return TypeScanBundle;
}

ViewBase::Ptr ScanBundleView::
clone() const {
  ScanBundleView* view = new ScanBundleView(*this);
  for (auto& scan : view->mScans) scan.reset(new LidarScan(*scan));
  return ViewBase::Ptr(view);
}

void ScanBundleView::
set(const maps::PointCloud::Ptr& iCloud) {
  std::cout << "error: cannot set point cloud in scan bundle view" << std::endl;
}

maps::PointCloud::Ptr ScanBundleView::
getAsPointCloud(const bool iTransform) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  maps::PointCloud curCloud;
  for (const auto scan : mScans) {
    scan->get(curCloud, true);
    *cloud += curCloud;
  }
  if (iTransform) {
    pcl::transformPointCloud(*cloud, *cloud,
                             Eigen::Affine3f(mTransform.matrix()).inverse());
  }
  return cloud;
}
