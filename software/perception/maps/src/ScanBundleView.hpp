#ifndef _maps_ScanBundleView_hpp_
#define _maps_ScanBundleView_hpp_

#include "ViewBase.hpp"
#include "LidarScan.hpp"

namespace maps {

class ScanBundleView : public ViewBase {
public:
  typedef std::shared_ptr<ScanBundleView> Ptr;
public:
  ScanBundleView();
  ~ScanBundleView();

  void set(const std::vector<maps::LidarScan::Ptr>& iScans);
  std::vector<maps::LidarScan::Ptr> getScans() const;
  int getNumScans() const;

  const Type getType() const;
  ViewBase::Ptr clone() const;
  void set(const maps::PointCloud::Ptr& iCloud);
  maps::PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;

protected:
  std::vector<maps::LidarScan::Ptr> mScans;
};

}

#endif
