#ifndef _maps_PointCloudView_hpp_
#define _maps_PointCloudView_hpp_

#include "ViewBase.hpp"

namespace maps {

class PointCloudView : public ViewBase {
public:
  typedef std::shared_ptr<PointCloudView> Ptr;
public:
  PointCloudView();
  ~PointCloudView();

  void setResolution(const float iResolution);
  maps::PointCloud::Ptr getPointCloud() const;  

  const Type getType() const;
  ViewBase::Ptr clone() const;
  void set(const maps::PointCloud::Ptr& iCloud);
  maps::PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;

protected:
  float mResolution;
  maps::PointCloud::Ptr mCloud;
};

}

#endif
