#include "DepthImage.hpp"

#include <pcl/range_image/range_image_planar.h>

#include "Utils.hpp"

namespace pcl {
class RangeImageProjective : public RangeImagePlanar {
public:
  RangeImage* getNew() const { return new RangeImageProjective(); }
  Ptr makeShared() { return Ptr(new RangeImageProjective(*this)); }

  void setPose(const Eigen::Isometry3f& iPose) {
    to_world_system_ = iPose;
    to_range_image_system_ = iPose.inverse();
  }
};

class RangeImageOrthographic : public RangeImageProjective {
public:
  RangeImage* getNew() const { return new RangeImageOrthographic(); }
  Ptr makeShared() { return Ptr(new RangeImageOrthographic(*this)); }

  inline void calculate3DPoint(float iX, float iY, float iDepth,
                               Eigen::Vector3f& oPoint) const {
    oPoint = mProjectorInv*Eigen::Vector3f(iX, iY, iDepth);
  }

  inline void getImagePoint(const Eigen::Vector3f& iPoint,
                            float& oX, float& oY, float& oDepth) const {
    Eigen::Vector3f pt = mProjector*iPoint;
    oX = pt[0];
    oY = pt[1];
    oDepth = pt[2];
  }

  void setDepthImage(const float* iDepth, int iWidth, int iHeight,
                     float iCenterX, float iCenterY,
                     float iFocalX, float iFocalY, float iAngRes=-1) {
    RangeImagePlanar::setDepthImage(iDepth, iWidth, iHeight, iCenterX,
                                    iCenterY, iFocalX, iFocalY, iAngRes);
    Eigen::Affine3f calib = Eigen::Affine3f::Identity();
    calib(0,0) = focal_length_x_;
    calib(1,1) = focal_length_y_;
    calib(0,3) = center_x_;
    calib(1,3) = center_y_;
    mProjector = calib*to_range_image_system_;
    mProjectorInv = mProjector.inverse();
  }

protected:
  Eigen::Affine3f mProjector;
  Eigen::Affine3f mProjectorInv;
};

}



using namespace maps;

DepthImage::
DepthImage() {
  mWidth = 0;
  mHeight = 0;
  mPose = Eigen::Isometry3f::Identity();
  mCalib = Eigen::Matrix3f::Identity();
  mProjector = Eigen::Projective3f::Identity();
  updateMatrices();
}

DepthImage::
~DepthImage() {
}

void DepthImage::
setSize(const int iWidth, const int iHeight) {
  mWidth = iWidth;
  mHeight = iHeight;
  mData.resize(mWidth*mHeight);
}

int DepthImage::
getWidth() const {
  return mWidth;
}

int DepthImage::
getHeight() const {
  return mHeight;
}

const float DepthImage::
getInvalidValue(const Type iType) const {
  if (mIsOrthographic) return std::numeric_limits<float>::infinity();
  return (iType==TypeDisparity ? 0 : std::numeric_limits<float>::infinity());
}

bool DepthImage::
setData(const std::vector<float>& iData, const Type iType) {
  if (iData.size() != mWidth*mHeight) return false;

  if (mIsOrthographic || (iType == TypeDisparity)) mData = iData;
  else {
    if (iType == TypeDepth) {
      for (int i = 0; i < iData.size(); ++i) {
        mData[i] = 1/iData[i];
      }
    }
    else if (iType == TypeRange) {
      float invalidValue = getInvalidValue(iType);
      float invalidDisparity = getInvalidValue(TypeDisparity);
      for (int i = 0, idx = 0; i < mHeight; ++i) {
        for (int j = 0; j < mWidth; ++j, ++idx) {
          Eigen::Vector3f pt = mCalibInv*Eigen::Vector3f(j,i,1);
          if (iData[idx] == invalidValue) mData[idx] = invalidDisparity;
          else mData[idx] = pt.norm()/(pt[2]*iData[idx]);
        }
      }
    }
  }
  return true;
}

std::vector<float> DepthImage::
getData(const Type iType) const {
  if (mIsOrthographic || (iType == TypeDisparity)) return mData;

  std::vector<float> data(mData.size());
  if (iType == TypeDepth) {
    for (int i = 0; i < mData.size(); ++i) {
      data[i] = 1/mData[i];
    }
  }
  else if (iType == TypeRange) {
    float invalidValue = getInvalidValue(iType);
    float invalidDisparity = getInvalidValue(TypeDisparity);
    for (int i = 0, idx = 0; i < mHeight; ++i) {
      for (int j = 0; j < mWidth; ++j, ++idx) {
        Eigen::Vector3f pt = mCalibInv*Eigen::Vector3f(j,i,1);
        if (mData[idx] == invalidDisparity) data[idx] = invalidValue;
        else data[idx] = (mCalibInv*Eigen::Vector3f(j,i,1)/mData[idx]).norm();
      }
    }
  }
  return data;
}

void DepthImage::
setOrthographic(const bool iVal) {
  mIsOrthographic = iVal;
  updateMatrices();
}

bool DepthImage::
isOrthographic() const {
  return mIsOrthographic;
}

void DepthImage::
setPose(const Eigen::Isometry3f& iPose) {
  mPose = iPose;
  updateMatrices();
}

Eigen::Isometry3f DepthImage::
getPose() const {
  return mPose;
}

void DepthImage::
setCalib(const Eigen::Matrix3f& iCalib) {
  mCalib = iCalib;
  updateMatrices();
}

Eigen::Matrix3f DepthImage::
getCalib() const {
  return mCalib;
}

void DepthImage::
setProjector(const Eigen::Projective3f& iProjector) {
  mProjector = iProjector;
  Utils::factorViewMatrix(mProjector, mCalib, mPose, mIsOrthographic);
  updateMatrices();
}

Eigen::Projective3f DepthImage::
getProjector() const {
  return mProjector;
}

void DepthImage::
updateMatrices() {
  Utils::composeViewMatrix(mProjector, mCalib, mPose, mIsOrthographic);
  mCalibInv = mCalib.inverse();
  mPoseInv = mPose.inverse();
  mProjectorInv = mProjector.inverse();
}

void DepthImage::
create(const maps::PointCloud::Ptr& iCloud) {
  std::fill(mData.begin(), mData.end(), getInvalidValue(TypeDisparity));
  for (int i = 0; i < iCloud->size(); ++i) {
    maps::PointCloud::PointType& ptCur = (*iCloud)[i];
    Eigen::Vector4f pt(ptCur.x, ptCur.y, ptCur.z, 1);
    pt = mProjector*pt;
    Eigen::Vector3f proj = pt.head<3>()/pt[3];
    int x(proj[0]+0.5f), y(proj[1]+0.5f);
    if ((x < 0) || (x >= mWidth) || (y < 0) || (y >= mHeight)) continue;
    int idx = y*mWidth + x;
    if ((mIsOrthographic && (proj[2] < mData[idx])) ||
        (!mIsOrthographic && (proj[2] > mData[idx]))) {
      mData[idx] = proj[2];
    }
  }
}

maps::PointCloud::Ptr DepthImage::
getAsPointCloud() const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  cloud->reserve(mWidth*mHeight);
  float invalidValue = getInvalidValue(TypeDisparity);
  for (int i = 0, idx = 0; i < mHeight; ++i) {
    for (int j = 0; j < mWidth; ++j, ++idx) {
      float z = mData[idx];
      if (z == invalidValue) continue;
      Eigen::Vector3f pt(j,i,z);
      pt = unproject(pt, TypeDisparity);
      maps::PointCloud::PointType p;
      p.getVector3fMap() = pt;
      cloud->push_back(p);
    }
  }
  return cloud;
}

pcl::RangeImage::Ptr DepthImage::
getAsRangeImage() const {
  std::vector<float> depths = getData(TypeDepth);
  pcl::RangeImageProjective* rangeImage;
  if (mIsOrthographic) rangeImage = new pcl::RangeImageOrthographic();
  else rangeImage = new pcl::RangeImageProjective();
  rangeImage->setPose(mPose);
  rangeImage->setDepthImage(&depths[0], mWidth, mHeight,
                            mCalib(0,2), mCalib(1,2), mCalib(0,0), mCalib(1,1));
  return pcl::RangeImage::Ptr(rangeImage);
}

Eigen::Vector3f DepthImage::
project(const Eigen::Vector3f& iPoint, const Type iType) const {
  if (mIsOrthographic) {
    Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
    return (mProjector*pt).head<3>();
  }
  else {
    if (iType == TypeDepth) {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
      pt = mProjector*pt;
      float wInv = 1/pt[3];
      return Eigen::Vector3f(pt[0]*wInv, pt[1]*wInv, pt[2]);
    }
    else if (iType == TypeRange) {
      Eigen::Vector3f pt = mPoseInv*iPoint;
      float range = pt.norm();
      pt = mCalib*pt;
      float wInv = pt[2];
      return Eigen::Vector3f(pt[0]*wInv, pt[1]*wInv, range);
    }
    else {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
      pt = mProjector*pt;
      return pt.head<3>()/pt[3];
    }
  }
}

Eigen::Vector3f DepthImage::
unproject(const Eigen::Vector3f& iPoint, const Type iType) const {
  if (mIsOrthographic) {
    Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
    return (mProjectorInv*pt).head<3>();
  }
  else {
    if (iType == TypeDepth) {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], 1, 1/iPoint[2]);
      pt = mProjectorInv*pt;
      return pt.head<3>()/pt[3];
    }
    else if (iType == TypeRange) {
      Eigen::Vector3f pt = mCalibInv*iPoint;
      pt *= (iPoint[2]/pt.norm());
      return mPose*pt;
    }
    else {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
      pt = mProjectorInv*pt;
      return pt.head<3>()/pt[3];
    }
  }
}
