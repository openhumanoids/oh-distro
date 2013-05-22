#include "ViewClientWrapper.hpp"

#include <Eigen/Sparse>

#include <maps/BotWrapper.hpp>
#include <maps/ViewClient.hpp>
#include <maps/DepthImage.hpp>
#include <maps/DepthImageView.hpp>

#include "MapLib.hpp"

using namespace mexmaps;

struct ViewClientWrapper::Listener : public maps::ViewClient::Listener {
  ViewClientWrapper* mWrapper;

  Listener(ViewClientWrapper* iWrapper) : mWrapper(iWrapper) {}

  void notifyData(const int64_t iViewId) {
    if (iViewId != mWrapper->mHeightMapViewId) return;
    auto view = std::static_pointer_cast<maps::DepthImageView>
      (mWrapper->mViewClient->getView(mWrapper->mHeightMapViewId));
    if (view != NULL) {
      view->setNormalMethod(maps::DepthImageView::NormalMethodLeastSquares);
      view->setNormalRadius(mWrapper->mNormalRadius);
      if (mWrapper->mShouldFill) {
        //mWrapper->fillView(view);
        mWrapper->fillViewPlanar(view);
      }
    }
  }

  void notifyCatalog(const bool iChanged) {}
};

ViewClientWrapper::
ViewClientWrapper(const int iId, const std::shared_ptr<lcm::LCM>& iLcm) {
  mId = iId;
  mLcm = iLcm;
  mBotWrapper.reset(new maps::BotWrapper(mLcm, NULL, NULL));
  mNormalRadius = 0;
  mShouldFill = false;
  mViewClient.reset(new maps::ViewClient());
  mViewClient->setBotWrapper(mBotWrapper);
  mListener.reset(new Listener(this));
  mViewClient->addListener(mListener.get());
  setHeightMapChannel(kHeightMapChannel, kHeightMapViewId);
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
  // TODO: make this check how much time has elapsed since last receipt
  requestHeightMap();

  auto view = std::static_pointer_cast<maps::DepthImageView>
    (mViewClient->getView(mHeightMapViewId));
  if (view != NULL) {
    view->setNormalMethod(maps::DepthImageView::NormalMethodLeastSquares);
    view->setNormalRadius(mNormalRadius);
    if (mShouldFill) {
      //fillView(view);
      fillViewPlanar(view);
    }
  }
  return view;
}

void ViewClientWrapper::
requestHeightMap() {
  if (mHeightMapViewId != kHeightMapViewId) {
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
  spec.mViewId = kHeightMapViewId;
  spec.mType = maps::ViewBase::TypeDepthImage;
  spec.mChannel = kHeightMapChannel;
  spec.mFrequency = 1;
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

void ViewClientWrapper::
fillView(maps::DepthImageView::Ptr& iView) {
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));

  // compute max z change
  Eigen::Projective3f mapToWorld = iView->getTransform().inverse();
  Eigen::Vector4f p0 = mapToWorld*Eigen::Vector4f(0,0,0,1);
  Eigen::Vector4f px = mapToWorld*Eigen::Vector4f(1,0,0,1);
  float dx = (px-p0).norm();
  const float kPi = std::acos(-1.0);
  float maxDiffZ = tan(45*kPi/180)*dx;

  // build invalid sample list
  const float invalidValue = img->getInvalidValue(type);
  std::vector<int> invalidMap(depths.size());
  std::vector<bool> pixelsToUpdate(depths.size());
  std::vector<int> invalidList;
  invalidList.reserve(depths.size());
  std::fill(invalidMap.begin(), invalidMap.end(), -1);
  std::fill(pixelsToUpdate.begin(), pixelsToUpdate.end(), false);
  int w(img->getWidth()), h(img->getHeight());
  for (int i = 0, idx = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j, ++idx) {
      bool shouldUpdatePixel = false;
      bool invalidPixel = false;
      float z = depths[idx];
      if (z == invalidValue) shouldUpdatePixel = invalidPixel = true;
      else {
        if ((i == h-1) || (j == w-1)) invalidPixel = true;
        else {
          float zx(depths[idx+1]), zy(depths[idx+w]);
          if ((zx == invalidValue) || (zy == invalidValue)) {
            invalidPixel = true;
          }
          else {
            float gx(fabs(zx-z)), gy(fabs(zy-z));
            if ((gx > maxDiffZ) || (gy > maxDiffZ)) invalidPixel = true;
          }
        }
      }
      if (invalidPixel) { 
        invalidMap[idx] = invalidList.size();
        invalidList.push_back(idx);
        if (shouldUpdatePixel) pixelsToUpdate[idx] = true;
      }
    }
  }
  if (invalidList.size() == 0) return;

  // set up sparse linear system using 4-way neighbors
  std::vector<Eigen::Triplet<float> > triplets;
  triplets.reserve(invalidList.size()*5);
  Eigen::VectorXf rhs(invalidList.size());
  rhs.setZero();
  int xMax(w-1), yMax(h-1);
  for (size_t i = 0; i < invalidList.size(); ++i) {
    int index = invalidList[i];
    int x = index%w;
    int y = index/w;
    bool left(x==0), right(x==xMax), top(y==0), bottom(y==yMax);
    float numNeighbors = int(!left) + int(!right) + int(!top) + int(!bottom);
    triplets.push_back(Eigen::Triplet<float>(i,i,numNeighbors));
    if (!left) {
      if (invalidMap[index-1] < 0) rhs[i] += depths[index-1];
      else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index-1],-1));
    }
    if (!right) {
      if (invalidMap[index+1] < 0) rhs[i] += depths[index+1];
      else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index+1],-1));
    }
    if (!top) {
      if (invalidMap[index-w] < 0) rhs[i] += depths[index-w];
      else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index-w],-1));
    }
    if (!bottom) {
      if (invalidMap[index+w] < 0) rhs[i] += depths[index+w];
      else triplets.push_back(Eigen::Triplet<float>(i,invalidMap[index+w],-1));
    }
  }

  // solve sparse system
  Eigen::SparseMatrix<float> lhs(invalidList.size(), invalidList.size());
  lhs.setFromTriplets(triplets.begin(), triplets.end());
  Eigen::SimplicialLLT<Eigen::SparseMatrix<float> > solver;
  solver.compute(lhs);
  if (solver.info() == Eigen::Success) {
    Eigen::VectorXf solution = solver.solve(rhs);

    // fill in values
    for (size_t i = 0; i < invalidList.size(); ++i) {
      if (pixelsToUpdate[invalidList[i]]) {
        depths[invalidList[i]] = solution[i];
      }
    }
    img->setData(depths, type);
  }
  else {
    std::cout << "Error: cannot solve sparse system" << std::endl;
  }
}

void ViewClientWrapper::
fillViewPlanar(maps::DepthImageView::Ptr& iView) {
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));

  // gather valid points
  std::vector<Eigen::Vector3f> points;
  points.reserve(depths.size());
  const float invalidValue = img->getInvalidValue(type);
  int w(img->getWidth()), h(img->getHeight());
  for (int i = 0, idx = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j, ++idx) {
      float z = depths[idx];
      if (z == invalidValue) continue;
      points.push_back(Eigen::Vector3f(j,i,z));
    }
  }
  int n = points.size();

  // set up equations
  Eigen::VectorXf rhs(n);
  Eigen::MatrixXf lhs(n,3);
  for (int i = 0; i < n; ++i) {
    rhs[i] = -points[i][2];
    lhs(i,0) = points[i][0];
    lhs(i,1) = points[i][1];
    lhs(i,2) = 1;
  }

  // plane fit
  Eigen::Vector3f sol;
  Eigen::VectorXf weights = Eigen::VectorXf::Ones(n);
  Eigen::VectorXf weightsPrev = weights;
  const float sigma2 = 0.1*0.1;
  const float weightThresh = 1e-3f * 1e-3f * n;
  for (int iter = 0; iter < 10; ++iter) {
    // solve
    Eigen::MatrixXf a = weights.asDiagonal()*lhs;
    Eigen::VectorXf b = weights.asDiagonal()*rhs;
    sol = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(b);

    // re-compute weights
    Eigen::VectorXf e2 = lhs*sol - rhs;
    e2 = e2.array()*e2.array();
    weights = 1-e2.array()/sigma2;
    weights = (weights.array() < 0).select(0.0f, weights);

    // check for convergence
    Eigen::VectorXf weightDiff = weightsPrev-weights;
    if (weightDiff.dot(weightDiff) < weightThresh) break;
    weightsPrev = weights;
  }

  // fill in values
  for (int i = 0, idx = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j, ++idx) {
      float z = depths[idx];
      if (z != invalidValue) continue;
      depths[idx] = -(sol[0]*j + sol[1]*i + sol[2]);
    }
  }
  img->setData(depths, type);
}
