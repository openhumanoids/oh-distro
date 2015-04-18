#include "DepthImage.hpp"

#include "Utils.hpp"

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
    for (int i = 0; i < (int)mDataNeedsUpdate.size(); ++i) {
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
  if ((int)iData.size() != mHelper->mWidth * mHelper->mHeight) return false;

  if (mHelper->mIsOrthographic || (iType == TypeDisparity)) {
    mHelper->mData = iData;
  }
  else {
    if (iType == TypeDepth) {
      for (int i = 0; i < (int)iData.size(); ++i) {
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
          else data[idx] = (pt / mHelper->mData[idx]).norm();
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
