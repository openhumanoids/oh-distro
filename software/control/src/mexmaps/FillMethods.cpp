#include "FillMethods.hpp"

#include <Eigen/Sparse>

#include <maps/BotWrapper.hpp>
#include <maps/DepthImage.hpp>
#include <maps/DepthImageView.hpp>

#include <opencv2/opencv.hpp>

using namespace mexmaps;


float FillMethods::
computeMedian(const Eigen::VectorXf& iData) {
  int n = iData.size();
  std::vector<float> data(iData.data(), iData.data()+n);
  std::sort(data.begin(), data.end());
  float medVal = (n%2 != 0) ? data[n/2] : 0.5*(data[n/2-1] + data[n/2]);
}  

Eigen::Vector3f FillMethods::
fitHorizontalPlaneRobust(const std::vector<Eigen::Vector3f>& iPoints,
                         const Eigen::Vector4f& iInitPlane) {

  // set up equations
  int n = iPoints.size();
  Eigen::VectorXf rhs(n);
  Eigen::MatrixXf lhs(n,3);
  for (int i = 0; i < n; ++i) {
    rhs[i] = -iPoints[i][2];
    lhs(i,0) = iPoints[i][0];
    lhs(i,1) = iPoints[i][1];
    lhs(i,2) = 1;
  }

  // use input plane if it is valid
  Eigen::Vector3f sol;
  float planeC = iInitPlane[2];
  if (fabs(planeC) < 1e-8) {
    //sol = lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(rhs);
    sol << 0,0,computeMedian(rhs);
  }
  else {
    sol = Eigen::Vector3f(iInitPlane[0], iInitPlane[1], iInitPlane[3]);
    sol /= planeC;
  }

  // iterative plane fit
  Eigen::VectorXf weights = Eigen::VectorXf::Ones(n);
  Eigen::VectorXf weightsPrev = weights;
  const float weightThresh = 1e-3f * 1e-3f * n;
  for (int iter = 0; iter < 10; ++iter) {

    // re-compute weights
    Eigen::VectorXf e2 = lhs*sol - rhs;
    e2.array() *= e2.array();
    float medVal = computeMedian(e2);
    float sigma2 = std::max(1.4826*sqrt(medVal)*4.7851, 1e-6);
    sigma2 *= sigma2;
    weights = 1-e2.array()/sigma2;
    weights = (weights.array() < 0).select(0.0f, weights);

    // check for convergence
    Eigen::VectorXf weightDiff = weightsPrev-weights;
    if (weightDiff.dot(weightDiff) < weightThresh) break;
    weightsPrev = weights;

    // solve
    Eigen::MatrixXf a = weights.asDiagonal()*lhs;
    Eigen::VectorXf b = weights.asDiagonal()*rhs;
    sol = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(b);
  }

  return sol;
}

FillMethods::
FillMethods(const std::shared_ptr<maps::BotWrapper>& iWrapper) {
  mBotWrapper = iWrapper;
}

void FillMethods::
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

void FillMethods::
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

  Eigen::Vector3f sol = fitHorizontalPlaneRobust(points);

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

void FillMethods::
fillUnderRobot(maps::DepthImageView::Ptr& iView) {
  const float radiusMeters = 0.5;

  // get view data
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));
  const float invalidValue = img->getInvalidValue(type);

  // get robot position and project into map
  Eigen::Isometry3f robotPose;
  if (!mBotWrapper->getTransform("body", "local", robotPose)) return;
  Eigen::Vector3f robotPosition = robotPose.translation();
  Eigen::Vector3f mapPos = img->project(robotPosition, type);
  Eigen::Vector3f radiusPosition = robotPosition +
    Eigen::Vector3f(1,0,0)*radiusMeters;
  Eigen::Vector3f mapRadiusPos = img->project(radiusPosition, type);
  float radiusPixels = (mapRadiusPos - mapPos).norm();

  // gather nearby points within radius
  int cx(mapPos[0] + 0.5f), cy(mapPos[1] + 0.5f), r(ceil(radiusPixels));
  int w(img->getWidth()), h(img->getHeight());
  std::vector<Eigen::Vector3f> points;
  points.reserve((2*r+1) * (2*r+1));
  for (int i = cy-r; i <= cy+r; ++i) {
    if ((i < 0) || (i >= h)) continue;
    for (int j = cx-r; j <= cx+r; ++j) {
      if ((j < 0) || (j >= w)) continue;
      int idx = i*w + j;
      float z = depths[idx];
      if (z == invalidValue) continue;
      points.push_back(Eigen::Vector3f(j,i,z));
    }
  }
  if (points.size() < 10) return;

  // solve for plane
  Eigen::Vector3f sol = fitHorizontalPlaneRobust(points);

  // fill in invalid values
  for (int i = cy-r; i <= cy+r; ++i) {
    if ((i < 0) || (i >= h)) continue;
    for (int j = cx-r; j <= cx+r; ++j) {
      if ((j < 0) || (j >= w)) continue;
      int idx = i*w + j;
      float z = depths[idx];
      if (z != invalidValue) continue;
      depths[idx] = -(sol[0]*j + sol[1]*i + sol[2]);
    }
  }
  img->setData(depths, type);
}

void FillMethods::
fillContours(maps::DepthImageView::Ptr& iView) {
  // get view data
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));
  const float invalidValue = img->getInvalidValue(type);

  // get robot position and heading, and project into map
  Eigen::Isometry3f robotPose;
  if (!mBotWrapper->getTransform("body", "local", robotPose)) return;
  Eigen::Vector3f robotPosition = robotPose.translation();
  Eigen::Vector3f robotHeading = robotPose.linear().col(0);
  Eigen::Vector3f robotForward = robotPosition + robotHeading;
  Eigen::Vector3f mapPosition = img->project(robotPosition, type);
  Eigen::Vector3f mapForward = img->project(robotForward, type);
  Eigen::Vector3f mapHeading = mapForward - mapPosition;
  mapHeading[2] = 0;
  mapHeading.normalize();

  // form mask indicating where fills need to occur
  int w(img->getWidth()), h(img->getHeight());
  cv::Mat validArea = cv::Mat::zeros(h,w,CV_8U);
  // TODO: fill in valid area
  cv::Mat depthImage(h,w,CV_32F,&depths[0]);
  cv::Mat badMask;
  cv::threshold(depthImage, badMask, 1e8, 255, cv::THRESH_BINARY);
  badMask.convertTo(badMask, CV_8U);
  cv::bitwise_and(badMask, validArea, badMask);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(badMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  for (size_t i = 0; i < contours.size(); ++i) {
    // gather points on boundary
    // how to get interior points??
  }
}
