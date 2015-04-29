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


struct DepthImage::Helper {
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
  AccumulationMethod mAccumulationMethod;
  std::vector<bool> mDataNeedsUpdate;
  std::vector<std::vector<float> > mDataCache;

  Helper() {
    mWidth = 0;
    mHeight = 0;
    mPose = Eigen::Isometry3f::Identity();
    mCalib = Eigen::Matrix3f::Identity();
    mProjector = Eigen::Projective3f::Identity();
    mIsOrthographic = true;
    mAccumulationMethod = AccumulationMethodClosest;
    mDataCache.resize(3);
    mDataNeedsUpdate.resize(mDataCache.size());
    updateMatrices();
  }

  void updateMatrices() {
    Utils::composeViewMatrix(mProjector, mCalib, mPose, mIsOrthographic);
    mCalibInv = mCalib.inverse();
    mPoseInv = mPose.inverse();
    mProjectorInv = mProjector.inverse();
    invalidateData();
  }

  void invalidateData() {
    for (int i = 0; i < mDataNeedsUpdate.size(); ++i) {
      mDataNeedsUpdate[i] = true;
    }
    mDataNeedsUpdate[TypeDisparity] = false;
  }
};


DepthImage::
DepthImage() {
  mHelper.reset(new Helper());
}

DepthImage::
~DepthImage() {
}

void DepthImage::
setSize(const int iWidth, const int iHeight) {
  mHelper->mWidth = iWidth;
  mHelper->mHeight = iHeight;
  mHelper->mData.resize(mHelper->mWidth * mHelper->mHeight);
  mHelper->invalidateData();
}

int DepthImage::
getWidth() const {
  return mHelper->mWidth;
}

int DepthImage::
getHeight() const {
  return mHelper->mHeight;
}

const float DepthImage::
getInvalidValue(const Type iType) const {
  if (mHelper->mIsOrthographic) return std::numeric_limits<float>::infinity();
  return (iType==TypeDisparity ? 0 : std::numeric_limits<float>::infinity());
}

bool DepthImage::
setData(const std::vector<float>& iData, const Type iType) {
  if (iData.size() != mHelper->mWidth * mHelper->mHeight) return false;

  if (mHelper->mIsOrthographic || (iType == TypeDisparity)) {
    mHelper->mData = iData;
  }
  else {
    if (iType == TypeDepth) {
      for (int i = 0; i < iData.size(); ++i) {
        mHelper->mData[i] = 1/iData[i];
      }
    }
    else if (iType == TypeRange) {
      float invalidValue = getInvalidValue(iType);
      float invalidDisparity = getInvalidValue(TypeDisparity);
      for (int i = 0, idx = 0; i < mHelper->mHeight; ++i) {
        for (int j = 0; j < mHelper->mWidth; ++j, ++idx) {
          Eigen::Vector3f pt = mHelper->mCalibInv*Eigen::Vector3f(j,i,1);
          if (iData[idx] == invalidValue) {
            mHelper->mData[idx] = invalidDisparity;
          } else mHelper->mData[idx] = pt.norm()/(pt[2]*iData[idx]);
        }
      }
    }
  }
  mHelper->invalidateData();
  return true;
}

const std::vector<float>& DepthImage::
getData(const Type iType) const {
  if (mHelper->mIsOrthographic || (iType == TypeDisparity)) {
    return mHelper->mData;
  }

  if (mHelper->mDataNeedsUpdate[iType]) {
    std::vector<float>& data = mHelper->mDataCache[iType];
    int size = mHelper->mData.size();
    data.resize(size);

    if (iType == TypeDepth) {
      for (int i = 0; i < size; ++i) {
        data[i] = 1/mHelper->mData[i];
      }
    }

    else if (iType == TypeRange) {
      float invalidValue = getInvalidValue(iType);
      float invalidDisparity = getInvalidValue(TypeDisparity);
      for (int i = 0, idx = 0; i < mHelper->mHeight; ++i) {
        for (int j = 0; j < mHelper->mWidth; ++j, ++idx) {
          Eigen::Vector3f pt = mHelper->mCalibInv*Eigen::Vector3f(j,i,1);
          if (mHelper->mData[idx] == invalidDisparity) data[idx] = invalidValue;
          else data[idx] = (mHelper->mCalibInv*Eigen::Vector3f(j,i,1) /
                            mHelper->mData[idx]).norm();
        }
      }
    }

    mHelper->mDataNeedsUpdate[iType] = false;
  }

  return mHelper->mDataCache[iType];
}

void DepthImage::
setOrthographic(const bool iVal) {
  mHelper->mIsOrthographic = iVal;
  mHelper->updateMatrices();
}

bool DepthImage::
isOrthographic() const {
  return mHelper->mIsOrthographic;
}

void DepthImage::
setPose(const Eigen::Isometry3f& iPose) {
  mHelper->mPose = iPose;
  mHelper->updateMatrices();
}

Eigen::Isometry3f DepthImage::
getPose() const {
  return mHelper->mPose;
}

void DepthImage::
setCalib(const Eigen::Matrix3f& iCalib) {
  mHelper->mCalib = iCalib;
  mHelper->updateMatrices();
}

Eigen::Matrix3f DepthImage::
getCalib() const {
  return mHelper->mCalib;
}

void DepthImage::
setProjector(const Eigen::Projective3f& iProjector) {
  mHelper->mProjector = iProjector;
  Utils::factorViewMatrix(mHelper->mProjector, mHelper->mCalib,
                          mHelper->mPose, mHelper->mIsOrthographic);
  mHelper->updateMatrices();
}

Eigen::Projective3f DepthImage::
getProjector() const {
  return mHelper->mProjector;
}

void DepthImage::
setAccumulationMethod(const AccumulationMethod iMethod) {
  mHelper->mAccumulationMethod = iMethod;
}


void DepthImage::
create(const maps::PointCloud::Ptr& iCloud) {
  float invalidValue = getInvalidValue(TypeDisparity);
  std::fill(mHelper->mData.begin(), mHelper->mData.end(), invalidValue);
  
  std::vector<std::vector<float> > lists;
  AccumulationMethod method = mHelper->mAccumulationMethod;
  lists.resize(mHelper->mWidth * mHelper->mHeight);

  for (int i = 0; i < iCloud->size(); ++i) {
    maps::PointCloud::PointType& ptCur = (*iCloud)[i];
    Eigen::Vector4f pt(ptCur.x, ptCur.y, ptCur.z, 1);
    pt = mHelper->mProjector*pt;
    Eigen::Vector3f proj = pt.head<3>()/pt[3];
    int x(proj[0]+0.5f), y(proj[1]+0.5f);
    if ((x < 0) || (x >= mHelper->mWidth) ||
        (y < 0) || (y >= mHelper->mHeight)) continue;
    int idx = y*mHelper->mWidth + x;
    float z = proj[2];
    if (!mHelper->mIsOrthographic && (z <= 0)) continue;
    lists[idx].push_back(z);
  }

  // sort z lists if necessary
  if (method != AccumulationMethodMean) {
    for (auto& list : lists) std::sort(list.begin(), list.end());
  }

  float percentile;
  switch (method) {
  case AccumulationMethodClosest: percentile = 0.0f;  break;
  case AccumulationMethodFurthest: percentile = 1.0f;  break;
  case AccumulationMethodMedian: percentile = 0.5f;  break;
  case AccumulationMethodClosestPercentile: percentile = 0.1f;  break;
  case AccumulationMethodRobustBlend: percentile = 0.1f;  break;
  default: percentile = 0.5f;  break;
  }
  //if (mHelper->mIsOrthographic) percentile = 1.0f-percentile;

  // take the mean of the z list
  if (method == AccumulationMethodMean) {
    for (int i = 0; i < lists.size(); ++i) {
      const int n = lists[i].size();
      if (n == 0) continue;
      mHelper->mData[i] =
        (std::accumulate(lists[i].begin(), lists[i].end(), 0.0f))/n;
    }
  }

  // choose either mean or percentile z
  else if (method == AccumulationMethodRobustBlend) {
    for (int i = 0; i < lists.size(); ++i) {
      const auto& list = lists[i];
      const int n = list.size();
      if (n == 0) continue;
      float dz = list.back()-list.front();
      if (dz < 0.05) {
        mHelper->mData[i] = (std::accumulate(list.begin(), list.end(), 0.0f))/n;
      }
      else {
        mHelper->mData[i] = list[(int)(percentile*(n-1))];
      }
    }
  }

  // choose z at particular percentile in list
  else {
    for (int i = 0; i < lists.size(); ++i) {
      const int n = lists[i].size();
      if (n == 0) continue;
      mHelper->mData[i] = lists[i][(int)(percentile*(n-1))];
    }
  }

  mHelper->invalidateData();
}

maps::PointCloud::Ptr DepthImage::
getAsPointCloud() const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  cloud->reserve(mHelper->mWidth * mHelper->mHeight);
  float invalidValue = getInvalidValue(TypeDisparity);
  for (int i = 0, idx = 0; i < mHelper->mHeight; ++i) {
    for (int j = 0; j < mHelper->mWidth; ++j, ++idx) {
      float z = mHelper->mData[idx];
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
  const std::vector<float> depths = getData(TypeDepth);
  pcl::RangeImageProjective* rangeImage;
  if (mHelper->mIsOrthographic) rangeImage = new pcl::RangeImageOrthographic();
  else rangeImage = new pcl::RangeImageProjective();
  rangeImage->setPose(mHelper->mPose);
  rangeImage->setDepthImage(&depths[0], mHelper->mWidth, mHelper->mHeight,
                            mHelper->mCalib(0,2), mHelper->mCalib(1,2),
                            mHelper->mCalib(0,0), mHelper->mCalib(1,1));
  return pcl::RangeImage::Ptr(rangeImage);
}

Eigen::Vector3f DepthImage::
project(const Eigen::Vector3f& iPoint, const Type iType) const {
  if (mHelper->mIsOrthographic) {
    Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
    return (mHelper->mProjector*pt).head<3>();
  }
  else {
    if (iType == TypeDepth) {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
      pt = mHelper->mProjector*pt;
      float wInv = 1/pt[3];
      return Eigen::Vector3f(pt[0]*wInv, pt[1]*wInv, pt[2]);
    }
    else if (iType == TypeRange) {
      Eigen::Vector3f pt = mHelper->mPoseInv*iPoint;
      float range = pt.norm();
      pt = mHelper->mCalib*pt;
      float wInv = pt[2];
      return Eigen::Vector3f(pt[0]*wInv, pt[1]*wInv, range);
    }
    else {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
      pt = mHelper->mProjector*pt;
      return pt.head<3>()/pt[3];
    }
  }
}

Eigen::Vector3f DepthImage::
unproject(const Eigen::Vector3f& iPoint, const Type iType) const {
  if (mHelper->mIsOrthographic) {
    Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
    return (mHelper->mProjectorInv*pt).head<3>();
  }
  else {
    if (iType == TypeDepth) {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], 1, 1/iPoint[2]);
      pt = mHelper->mProjectorInv*pt;
      return pt.head<3>()/pt[3];
    }
    else if (iType == TypeRange) {
      Eigen::Vector3f pt = mHelper->mCalibInv*iPoint;
      pt *= (iPoint[2]/pt.norm());
      return mHelper->mPose*pt;
    }
    else {
      Eigen::Vector4f pt(iPoint[0], iPoint[1], iPoint[2], 1);
      pt = mHelper->mProjectorInv*pt;
      return pt.head<3>()/pt[3];
    }
  }
}
