#ifndef _maps_DepthImage_hpp_
#define _maps_DepthImage_hpp_

#include <vector>
#include <memory>
#include <Eigen/Geometry>
#include "Types.hpp"

namespace pcl {
  class RangeImage;
}

namespace maps {

class DepthImage {
public:
  enum Type {
    TypeDisparity,
    TypeDepth,
    TypeRange,
  };

  enum AccumulationMethod {
    AccumulationMethodClosest,
    AccumulationMethodFurthest,
    AccumulationMethodMean,
    AccumulationMethodMedian,
    AccumulationMethodClosestPercentile,
    AccumulationMethodRobustBlend,
  };

  typedef std::shared_ptr<DepthImage> Ptr;

  struct Helper;

public:
  DepthImage();
  ~DepthImage();

  void setSize(const int iWidth, const int iHeight);
  int getWidth() const;
  int getHeight() const;

  const float getInvalidValue(const Type iType) const;

  bool setData(const std::vector<float>& iData, const Type iType);
  const std::vector<float>& getData(const Type iType) const;

  void setOrthographic(const bool iVal);
  bool isOrthographic() const;

  void setPose(const Eigen::Isometry3f& iPose);
  Eigen::Isometry3f getPose() const;

  void setCalib(const Eigen::Matrix3f& iCalib);
  Eigen::Matrix3f getCalib() const;

  void setProjector(const Eigen::Projective3f& iProjector);
  Eigen::Projective3f getProjector() const;

  void setAccumulationMethod(const AccumulationMethod iMethod);

  void create(const maps::PointCloud::Ptr& iCloud);

  maps::PointCloud::Ptr getAsPointCloud() const;
  boost::shared_ptr<pcl::RangeImage> getAsRangeImage() const;
  Eigen::Vector3f project(const Eigen::Vector3f& iPoint,
                          const Type iType) const;
  Eigen::Vector3f unproject(const Eigen::Vector3f& iPoint,
                            const Type iType) const;

protected:
  std::shared_ptr<Helper> mHelper;
};

}

#endif
