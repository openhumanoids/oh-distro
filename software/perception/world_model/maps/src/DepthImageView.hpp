#ifndef _maps_DepthImageView_hpp_
#define _maps_DepthImageView_hpp_

#include "ViewBase.hpp"

namespace pcl {
  class RangeImage;
}

namespace maps {

class DepthImageView : public ViewBase {
public:
  typedef boost::shared_ptr<DepthImageView> Ptr;
public:
  DepthImageView();
  ~DepthImageView();

  // NOTE: the returned "range image" actually consists of depth values
  boost::shared_ptr<pcl::RangeImage> getRangeImage() const;
  void setSize(const int iWidth, const int iHeight);

  void set(const std::vector<float>& iData,
           const int iWidth=0, const int iHeight=0);

  const Type getType() const;
  ViewBase::Ptr clone() const;
  void set(const maps::PointCloud::Ptr& iCloud);
  maps::PointCloud::Ptr getAsPointCloud(const bool iTransform=true) const;
  maps::TriangleMesh::Ptr getAsMesh(const bool iTransform=true) const;
  bool getClosest(const Eigen::Vector3f& iPoint,
                  Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal);

protected:
  boost::shared_ptr<pcl::RangeImage> mImage;
};

}

#endif
