#include "ViewClientWrapper.hpp"

#include <chrono>

#include <maps/BotWrapper.hpp>
#include <maps/ViewClient.hpp>
#include <maps/DepthImageView.hpp>

#include <drc_utils/Clock.hpp>

#include "FillMethods.hpp"
#include "MapLib.hpp"

using namespace mexmaps;

struct ViewClientWrapper::Listener : public maps::ViewClient::Listener {
  ViewClientWrapper* mWrapper;

  Listener(ViewClientWrapper* iWrapper) : mWrapper(iWrapper) {}

  void notifyData(const int64_t iViewId) {
    if (iViewId != mWrapper->mHeightMapViewId) return;
    auto view = std::dynamic_pointer_cast<maps::DepthImageView>
      (mWrapper->mViewClient->getView(iViewId));
    if (view != NULL) {
      mWrapper->mLastReceiptTime = drc::Clock::instance()->getCurrentWallTime();
      view->setNormalMethod(maps::DepthImageView::NormalMethodLeastSquares);
      view->setNormalRadius(mWrapper->mNormalRadius);
      if (mWrapper->mShouldFill) {
        auto startTime = std::chrono::high_resolution_clock::now();
        FillMethods fillMethods(mWrapper->mBotWrapper);
        fillMethods.fillUnderRobot(view, FillMethods::MethodRansac);
        fillMethods.fillIterative(view);
        fillMethods.fillConnected(view);
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
        //fprintf(stderr, " elapsed time: %d ms\n", duration.count());
      }
    }
  }

  void notifyCatalog(const bool iChanged) {}
};

ViewClientWrapper::
ViewClientWrapper(const int iId, const std::shared_ptr<lcm::LCM>& iLcm) {
  mId = iId;
  mLcm = iLcm;
  mLastReceiptTime = -1;
  mBotWrapper.reset(new maps::BotWrapper(mLcm, NULL, NULL));
  mNormalRadius = 0;
  mShouldFill = false;
  mViewClient.reset(new maps::ViewClient());
  mViewClient->setBotWrapper(mBotWrapper);
  mListener.reset(new Listener(this));
  mViewClient->addListener(mListener.get());
  setHeightMapChannel(MapHandle::kHeightMapChannel,
                      MapHandle::kHeightMapViewId);
  mHandle.reset(new MapHandle(this));
}

ViewClientWrapper::
~ViewClientWrapper() {
  stop();
}

void ViewClientWrapper::
setHeightMapChannel(const std::string& iChannel, const int iViewId) {
  mViewClient->removeAllViewChannels();
  mViewClient->addViewChannel(iChannel);
  mHeightMapViewId = iViewId;
}

bool ViewClientWrapper::
start() {
  mViewClient->start();
  requestHeightMap();
  return true;
}

bool ViewClientWrapper::
stop() {
  mViewClient->stop();
  return true;
}

maps::ViewBase::Ptr ViewClientWrapper::
getView() {
  int64_t curTime = drc::Clock::instance()->getCurrentWallTime();
  if ((curTime - mLastReceiptTime) > 1000000) {
    requestHeightMap();
    mLastReceiptTime = curTime;
  }

  auto view = std::dynamic_pointer_cast<maps::DepthImageView>
    (mViewClient->getView(mHeightMapViewId));
  return view;
}

void ViewClientWrapper::
requestHeightMap() {
  if (mHeightMapViewId != MapHandle::kHeightMapViewId) {
    return;
  }
  const Eigen::Vector3f minPt(-2, -5, -3);
  const Eigen::Vector3f maxPt(5, 5, 0.3);
  const float timeWindowSeconds = 5;

  maps::ViewBase::Spec spec;
  spec.mResolution = 0.05;
  spec.mWidth = int((maxPt[0] - minPt[0]) / spec.mResolution);
  spec.mHeight = int((maxPt[1] - minPt[1]) / spec.mResolution);
  spec.mTimeMin = -5*1e6;
  spec.mClipPlanes.push_back(Eigen::Vector4f( 1, 0, 0, -minPt[0]));
  spec.mClipPlanes.push_back(Eigen::Vector4f(-1, 0, 0,  maxPt[0]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 1, 0, -minPt[1]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0,-1, 0,  maxPt[1]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 0, 1, -minPt[2]));
  spec.mClipPlanes.push_back(Eigen::Vector4f( 0, 0,-1,  maxPt[2]));

  spec.mMapId = 1;
  spec.mViewId = MapHandle::kHeightMapViewId;
  spec.mType = maps::ViewBase::TypeDepthImage;
  spec.mChannel = MapHandle::kHeightMapChannel;
  spec.mFrequency = 1 / 2.0;
  spec.mQuantizationMax = 0;
  spec.mTimeMax = 0;
  spec.mRelativeTime = true;
  spec.mRelativeLocation = true;
  spec.mActive = true;
  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.translation() = Eigen::Vector3f(0,0,10);
  pose.linear() << 1,0,0, 0,-1,0, 0,0,-1;
  Eigen::Affine3f calib = Eigen::Affine3f::Identity();
  calib(0,0) = 1/spec.mResolution;
  calib(1,1) = 1/spec.mResolution;
  calib(0,3) = -minPt[0]*calib(0,0);
  calib(1,3) = -minPt[1]*calib(1,1);
  Eigen::Projective3f projector = calib*pose.inverse();
  spec.mTransform = projector;
  mViewClient->request(spec);
}
