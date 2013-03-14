#ifndef _maps_DepthImage_hpp_
#define _maps_DepthImage_hpp_

#include <vector>
#include <boost/shared_ptr.hpp>
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

  typedef boost::shared_ptr<DepthImage> Ptr;

public:
  DepthImage();
  ~DepthImage();

  void setSize(const int iWidth, const int iHeight);
  int getWidth() const;
  int getHeight() const;

  const float getInvalidValue(const Type iType) const;

  bool setData(const std::vector<float>& iData, const Type iType);
  std::vector<float> getData(const Type iType) const;

  void setOrthographic(const bool iVal);
  bool isOrthographic() const;

  void setPose(const Eigen::Isometry3f& iPose);
  Eigen::Isometry3f getPose() const;

  void setCalib(const Eigen::Matrix3f& iCalib);
  Eigen::Matrix3f getCalib() const;

  void setProjector(const Eigen::Projective3f& iProjector);
  Eigen::Projective3f getProjector() const;

  void create(const maps::PointCloud::Ptr& iCloud);

  maps::PointCloud::Ptr getAsPointCloud() const;
  boost::shared_ptr<pcl::RangeImage> getAsRangeImage() const;
  Eigen::Vector3f project(const Eigen::Vector3f& iPoint,
                          const Type iType) const;
  Eigen::Vector3f unproject(const Eigen::Vector3f& iPoint,
                            const Type iType) const;

protected:
  void updateMatrices();
  bool isValid(const float iValue);

protected:
  int mWidth;
  int mHeight;
  std::vector<float> mData;
  Eigen::Isometry3f mPose;
  Eigen::Isometry3f mPoseInv;
  Eigen::Matrix3f mCalib;
  Eigen::Matrix3f mCalibInv;
  Eigen::Projective3f mProjector;
  Eigen::Projective3f mProjectorInv;
  bool mIsOrthographic;
};

}

#endif
