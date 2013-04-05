#ifndef _tracking_CameraModel_hpp_
#define _tracking_CameraModel_hpp_

#include <Eigen/Geometry>

class CameraModel {
public:
  CameraModel();

  void setCalibMatrix(const Eigen::Matrix3f& iMatrix);
  Eigen::Matrix3f getCalibMatrix() const;

  void setPose(const Eigen::Isometry3f& iPose);
  Eigen::Isometry3f getPose() const;

  Eigen::Vector3f pixelToRay(const Eigen::Vector2f& iPixel);
  Eigen::Vector3f pixelToRay(const Eigen::Vector3f& iPixel);
  Eigen::Vector3f pixelToRay(const float iX, const float iY);
  Eigen::Vector3f pixelToRay(const float iX, const float iY, const float iZ);

  Eigen::Vector3f rayToPixel(const Eigen::Vector3f& iRay);

protected:
  Eigen::Matrix3f mCalib;
  Eigen::Matrix3f mCalibInv;
  Eigen::Isometry3f mPose;
  Eigen::Isometry3f mPoseInv;
};

#endif
