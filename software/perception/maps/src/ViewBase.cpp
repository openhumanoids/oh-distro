#include "ViewBase.hpp"

using namespace maps;


ViewBase::Spec::
Spec() {
  mMapId = mViewId = -1;
  mActive = false;
  mTimeMode = TimeModeAbsolute;
  mRelativeLocation = false;
  mAccumulationMethod = AccumulationMethodClosest;
  mType = TypePointCloud;
  mResolution = 0;
  mFrequency = 0;
  mQuantizationMax = -1;
  mTimeMin = mTimeMax = -1;
  mTransform = Eigen::Projective3f::Identity();
  mWidth = mHeight = 0;
}

bool ViewBase::Spec::
operator==(const Spec& iSpec) const {
  bool eq = (mMapId == iSpec.mMapId) &&
    (mViewId == iSpec.mViewId) &&
    (mActive == iSpec.mActive) &&
    (mTimeMode == iSpec.mTimeMode) &&
    (mRelativeLocation == iSpec.mRelativeLocation) &&
    (mAccumulationMethod == iSpec.mAccumulationMethod) &&
    (mType == iSpec.mType) &&
    (mResolution == iSpec.mResolution) &&
    (mFrequency == iSpec.mFrequency) &&
    (mQuantizationMax == iSpec.mQuantizationMax) &&
    (mChannel == iSpec.mChannel) &&
    (mTimeMin == iSpec.mTimeMin) &&
    (mTimeMax == iSpec.mTimeMax) &&
    (mWidth == iSpec.mWidth) &&
    (mHeight == iSpec.mHeight) &&
    (mClipPlanes.size() == iSpec.mClipPlanes.size()) &&
    (mTransform.matrix() == iSpec.mTransform.matrix());
  if (!eq) {
    return false;
  }
  for (int i = 0; i < mClipPlanes.size(); ++i) {
    if (mClipPlanes[i] != iSpec.mClipPlanes[i]) {
      return false;
    }
  }
  return true;
}

bool ViewBase::Spec::
operator!=(const Spec& iSpec) const {
  return !(*this == iSpec);
}



ViewBase::
ViewBase() {
  mId = -1;
  mTransform = Eigen::Projective3f::Identity();
  mUpdateTime = -1;
}

ViewBase::
~ViewBase() {
}

void ViewBase::
setId(const int64_t iId) {
  mId = iId;
}

const int64_t ViewBase::
getId() const {
  return mId;
}

void ViewBase::
setTransform(const Eigen::Projective3f& iTransform) {
  mTransform = iTransform;
}

const Eigen::Projective3f ViewBase::
getTransform() const {
  return mTransform;
}

void ViewBase::
setUpdateTime(const int64_t iTime) {
  mUpdateTime = iTime;
}

const int64_t ViewBase::
getUpdateTime() const {
  return mUpdateTime;
}

maps::TriangleMesh::Ptr ViewBase::
getAsMesh(const bool iTransform) const {
  // must override this in derived classes
  maps::TriangleMesh::Ptr mesh;
  return mesh;
}


bool ViewBase::getClosest(const Eigen::Vector3f& iPoint,
                          Eigen::Vector3f& oPoint,
                          Eigen::Vector3f& oNormal) const {
  // must override this in derived classes
  return false;
}



bool ViewBase::
intersectRay(const Eigen::Vector3f& iOrigin, const Eigen::Vector3f& iDirection,
             Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal) const {
  // must override this in derived classes
  return false;
}
