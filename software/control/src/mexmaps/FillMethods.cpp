#include "FillMethods.hpp"

#include "RansacGeneric.hpp"

#include <Eigen/Sparse>

#include <lcm/lcm-cpp.hpp>

#include <maps/BotWrapper.hpp>
#include <maps/DepthImage.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/LcmTranslator.hpp>

#include <lcmtypes/drc/map_image_t.hpp>

#include <opencv2/opencv.hpp>
#include <fstream>

using namespace mexmaps;



FillMethods::
FillMethods(const std::shared_ptr<maps::BotWrapper>& iWrapper) {
  mBotWrapper = iWrapper;
  mDebug = true;
  mLatestGroundPlane << 0,0,0,0;
  mBotWrapper->getLcm()->subscribe("POSE_GROUND", &FillMethods::onGround, this);
}

void FillMethods::
onGround(const lcm::ReceiveBuffer* iBuf,
         const std::string& iChannel,
         const bot_core::pose_t* iMessage) {
  Eigen::Quaterniond q(iMessage->orientation[0], iMessage->orientation[1],
                       iMessage->orientation[2], iMessage->orientation[3]);
  Eigen::Vector3d pos(iMessage->pos[0], iMessage->pos[1], iMessage->pos[2]);
  mLatestGroundPlane.head<3>() = q.matrix().col(2);
  mLatestGroundPlane[3] = -mLatestGroundPlane.head<3>().dot(pos);
}


float FillMethods::
computeMedian(const Eigen::VectorXf& iData) {
  int n = iData.size();
  std::vector<float> data(iData.data(), iData.data()+n);
  std::sort(data.begin(), data.end());
  float medVal = (n%2 != 0) ? data[n/2] : 0.5*(data[n/2-1] + data[n/2]);
}

Eigen::Vector3f FillMethods::
fitHorizontalPlane(const std::vector<Eigen::Vector3f>& iPoints) {
  int n = iPoints.size();
  Eigen::VectorXf rhs(n);
  Eigen::MatrixXf lhs(n,3);
  for (int i = 0; i < n; ++i) {
    rhs[i] = -iPoints[i][2];
    lhs(i,0) = iPoints[i][0];
    lhs(i,1) = iPoints[i][1];
    lhs(i,2) = 1;
  }
  Eigen::Vector3f sol =
    lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(rhs);
  return sol;
}





struct HorizontalPlaneFitProblem {
  typedef Eigen::Vector3f Solution;

  std::vector<Eigen::Vector3f>* mPoints;
  FillMethods* mFillMethods;

  HorizontalPlaneFitProblem(FillMethods* iFillMethods) :
    mFillMethods(iFillMethods) {}

  int getNumDataPoints() const { return mPoints->size(); }
  int getSampleSize() const { return 3; }

  Solution estimate(const std::vector<int> iIndices) const {
    std::vector<Eigen::Vector3f> pts(iIndices.size());
    for (size_t i = 0; i < iIndices.size(); ++i) {
      pts[i] = (*mPoints)[iIndices[i]];
    }
    Solution plane = mFillMethods->fitHorizontalPlane(pts);
    return plane;
  }

  std::vector<double>
  computeSquaredErrors(const Solution& iPlane) const {
    size_t n = mPoints->size();
    std::vector<double> e2(n);
    for (size_t i = 0; i < n; ++i) {
      Eigen::Vector3f p = (*mPoints)[i];
      double e = p[0]*iPlane[0] + p[1]*iPlane[1] + p[2] + iPlane[2];
      e2[i] = e*e;
    }
    return e2;
  }
};



Eigen::Vector3f FillMethods::
fitHorizontalPlaneRansac(const std::vector<Eigen::Vector3f>& iPoints) {
  if (iPoints.size() < 3) return Eigen::Vector3f::Zero();
  HorizontalPlaneFitProblem problem(this);
  problem.mPoints = const_cast<std::vector<Eigen::Vector3f>*>(&iPoints);
  RansacGeneric<HorizontalPlaneFitProblem> ransacObj;
  ransacObj.setMaximumError(0.05);
  ransacObj.setRefineUsingInliers(true);
  RansacGeneric<HorizontalPlaneFitProblem>::Result result =
    ransacObj.solve(problem);
  if (result.mSuccess) return result.mSolution;
  return Eigen::Vector3f(0,0,0);
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

int FillMethods::
labelImage(cv::Mat& iMask, cv::Mat& oLabels) {
  uint8_t* mask = static_cast<uint8_t*>(iMask.data);
  uint16_t* labels = reinterpret_cast<uint16_t*>(oLabels.data);
  std::fill(labels, labels+iMask.rows*iMask.cols, 0);
  int curLabel = 1;
  for (int i = 0; i < iMask.rows; ++i) {
    for (int j = 0; j < iMask.cols; ++j, ++mask) {
      if (*mask == 0) continue;
      labelRecurse(iMask, i, j, curLabel, oLabels);
      ++curLabel;
    }
  }
  return curLabel-1;
}

void FillMethods::
labelRecurse(cv::Mat& iMask, const int iRow, const int iCol, const int iLabel,
             cv::Mat& oLabels) {
  int w(iMask.cols), h(iMask.rows);
  int idx = iRow*w + iCol;
  uint8_t* mask = static_cast<uint8_t*>(iMask.data);
  uint16_t* labels = reinterpret_cast<uint16_t*>(oLabels.data);
  labels[idx] = iLabel;
  mask[idx] = 0;
  if ((iRow > 0) && (mask[idx-w]>0)) {
    labelRecurse(iMask, iRow-1, iCol, iLabel, oLabels);
  }
  if ((iRow < h-1) && (mask[idx+w]>0)) {
    labelRecurse(iMask, iRow+1, iCol, iLabel, oLabels);
  }    
  if ((iCol > 0) && (mask[idx-1]>0)) {
    labelRecurse(iMask, iRow, iCol-1, iLabel, oLabels);
  }    
  if ((iCol < w-1) && (mask[idx+1]>0)) {
    labelRecurse(iMask, iRow, iCol+1, iLabel, oLabels);
  }
}

void FillMethods::
extractComponentsAndOutlines(const cv::Mat& iLabels, const int iNumLabels,
                             std::vector<std::vector<int> >& oIndices,
                             std::vector<std::set<int> >& oOutlines) {
  oIndices.resize(iNumLabels);
  oOutlines.resize(iNumLabels);
  int w(iLabels.cols), h(iLabels.rows);
  const uint16_t* labels = reinterpret_cast<uint16_t*>(iLabels.data);
  for (int i = 0; i < h; ++i) {
    for (int j = 0; j < w; ++j) {
      int idx = i*w + j;
      int label = labels[idx];
      if (label == 0) continue;
      --label;
      oIndices[label].push_back(idx);
      if ((i > 0) && (labels[idx-w]==0)) oOutlines[label].insert(idx-w);
      if ((i < h-1) && (labels[idx+w]==0)) oOutlines[label].insert(idx+w);
      if ((j > 0) && (labels[idx-1]==0)) oOutlines[label].insert(idx-1);
      if ((j < w-1) && (labels[idx+1]==0)) oOutlines[label].insert(idx+1);
    }
  }
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
fillUnderRobot(maps::DepthImageView::Ptr& iView, const Method iMethod) {
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

  Eigen::Vector3f sol;
  if (points.size() >= 10) {
    // solve for plane
    if (iMethod == MethodRobust) {
      sol = fitHorizontalPlaneRobust(points);
    }
    else {
      sol = fitHorizontalPlaneRansac(points);
    }
  }

  // try to get ground estimate from message
  else {
    if (mLatestGroundPlane.norm() < 1e-6) return;
    Eigen::Vector4f plane = mLatestGroundPlane.cast<float>();
    Eigen::Matrix4f planeTransform =
      iView->getTransform().inverse().matrix().transpose();
    plane = planeTransform*plane;
    sol << plane[0],plane[1],plane[3];
    sol /= plane[2];
    fprintf(stderr,"Filled in using ground plane message %f %f %f\n",
            sol[0], sol[1], sol[2]);
  }

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
  if (mDebug) {
    drc::map_image_t msg;
    maps::LcmTranslator::toLcm(*iView, msg);
    msg.map_id = 1;
    msg.view_id = 9999;
    msg.blob.utime = msg.utime;
    mBotWrapper->getLcm()->publish("MAP_DEBUG", &msg);
  }
}

/*
void FillMethods::
fillContours(maps::DepthImageView::Ptr& iView) {
  // get view data
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));
  const float invalidValue = img->getInvalidValue(type);

  // get robot position and bbox, and project into map
  Eigen::Isometry3f robotPose;
  if (!mBotWrapper->getTransform("body", "local", robotPose)) return;
  Eigen::Vector3f p0 = robotPose.translation();
  Eigen::Vector3f boundPoints[4];
  boundPoints[0] = -0.5*Eigen::Vector3f::UnitX();
  boundPoints[1] = +5.0*Eigen::Vector3f::UnitX();
  boundPoints[2] = -3.0*Eigen::Vector3f::UnitY();
  boundPoints[3] = +3.0*Eigen::Vector3f::UnitY();
  Eigen::Vector3f mapPoints[4];
  for (int i = 0; i < 4; ++i) {
    mapPoints[i] = robotPose*boundPoints[i];
    mapPoints[i] = img->project(mapPoints[i], type);
  }
  Eigen::Vector2i minPt(1000000,1000000), maxPt(-1000000,-1000000);
  for (int i = 0; i < 4; ++i) {
    minPt[0] = std::min(minPt[0], int(mapPoints[i][0]+0.5f));
    minPt[1] = std::min(minPt[1], int(mapPoints[i][1]+0.5f));
    maxPt[0] = std::max(maxPt[0], int(mapPoints[i][0]+0.5f));
    maxPt[1] = std::max(maxPt[1], int(mapPoints[i][1]+0.5f));
  }
  int w(img->getWidth()), h(img->getHeight());
  minPt[0] = std::max(0,minPt[0]);
  minPt[1] = std::max(0,minPt[1]);
  maxPt[0] = std::min(w-1,maxPt[0]);
  maxPt[1] = std::min(h-1,maxPt[1]);

  //
  // form mask indicating where fills need to occur
  //

  // first put box around robot
  cv::Mat validArea = cv::Mat::zeros(h,w,CV_8U);
  for (int i = minPt[1]; i <= maxPt[1]; ++i) {
    for (int j = minPt[0]; j <= maxPt[0]; ++j) {
      validArea.at<uint8_t>(i,j) = 255;
    }
  }

  // next find all inf values
  cv::Mat depthImage(h,w,CV_32F,&depths[0]);
  cv::Mat badMask;
  cv::threshold(depthImage, badMask, 1e8, 255, cv::THRESH_BINARY);
  badMask.convertTo(badMask, CV_8U);

  // and the two together and dilate so that outer extrema are found
  cv::bitwise_and(badMask, validArea, badMask);
  {
    std::ofstream ofs("/home/antone/badmask.txt");
    for (int i = 0; i < badMask.rows; ++i) {
      for (int j = 0; j < badMask.cols; ++j) {
        ofs << int(badMask.at<uint8_t>(i,j)) << " ";
      }
      ofs << std::endl;
    }
  }
  cv::dilate(badMask, badMask, cv::Mat());


  // get contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(badMask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

  // fit planes and label image
  std::vector<Eigen::Vector3f> planes(contours.size());
  cv::Mat labels = cv::Mat::zeros(h,w,CV_16U);
  for (size_t i = 0; i < contours.size(); ++i) {

    // fit plane
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(contours[i].size());
    for (size_t k = 0; k < contours[i].size(); ++k) {
      int x(contours[i][k].x), y(contours[i][k].y);
      float z = depths[y*w+x];
      if (z != invalidValue) pts.push_back(Eigen::Vector3f(x,y,z));
    }
    if ((pts.size() < 3) || (pts.size() > 250)) continue;
    planes[i] = fitHorizontalPlaneRobust(pts);

    // fill in labels
    cv::Scalar color = i+1;
    cv::drawContours(labels, contours, i, color, CV_FILLED);
  }

  {
    std::ofstream ofs("/home/antone/labels.txt");
    for (int i = 0; i < labels.rows; ++i) {
      for (int j = 0; j < labels.cols; ++j) {
        ofs << labels.at<uint16_t>(i,j) << " ";
      }
      ofs << std::endl;
    }
  }

  // fill in all values
  // TODO
}
*/

void FillMethods::
fillConnected(maps::DepthImageView::Ptr& iView) {
  // get view data
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));
  const float invalidValue = img->getInvalidValue(type);

  // get robot position and bbox, and project into map
  Eigen::Isometry3f robotPose;
  if (!mBotWrapper->getTransform("body", "local", robotPose)) return;
  Eigen::Vector3f p0 = robotPose.translation();
  Eigen::Vector3f boundPoints[4];
  boundPoints[0] = -0.5*Eigen::Vector3f::UnitX();
  boundPoints[1] = +5.0*Eigen::Vector3f::UnitX();
  boundPoints[2] = -3.0*Eigen::Vector3f::UnitY();
  boundPoints[3] = +3.0*Eigen::Vector3f::UnitY();
  Eigen::Vector3f mapPoints[4];
  for (int i = 0; i < 4; ++i) {
    mapPoints[i] = robotPose*boundPoints[i];
    mapPoints[i] = img->project(mapPoints[i], type);
  }
  Eigen::Vector2i minPt(1000000,1000000), maxPt(-1000000,-1000000);
  for (int i = 0; i < 4; ++i) {
    minPt[0] = std::min(minPt[0], int(mapPoints[i][0]+0.5f));
    minPt[1] = std::min(minPt[1], int(mapPoints[i][1]+0.5f));
    maxPt[0] = std::max(maxPt[0], int(mapPoints[i][0]+0.5f));
    maxPt[1] = std::max(maxPt[1], int(mapPoints[i][1]+0.5f));
  }
  int w(img->getWidth()), h(img->getHeight());
  minPt[0] = std::max(0,minPt[0]);
  minPt[1] = std::max(0,minPt[1]);
  maxPt[0] = std::min(w-1,maxPt[0]);
  maxPt[1] = std::min(h-1,maxPt[1]);

  //
  // form mask indicating where fills need to occur
  //

  // first put box around robot
  cv::Mat validArea = cv::Mat::zeros(h,w,CV_8U);
  for (int i = minPt[1]; i <= maxPt[1]; ++i) {
    for (int j = minPt[0]; j <= maxPt[0]; ++j) {
      validArea.at<uint8_t>(i,j) = 255;
    }
  }

  // next find all inf values
  cv::Mat depthImage(h,w,CV_32F,&depths[0]);
  cv::Mat badMask;
  cv::threshold(depthImage, badMask, 1e8, 255, cv::THRESH_BINARY);
  badMask.convertTo(badMask, CV_8U);

  // and the two together and dilate so that outer extrema are found
  cv::bitwise_and(badMask, validArea, badMask);
  {
    std::ofstream ofs("/home/antone/badmask.txt");
    for (int i = 0; i < badMask.rows; ++i) {
      for (int j = 0; j < badMask.cols; ++j) {
        ofs << int(badMask.at<uint8_t>(i,j)) << " ";
      }
      ofs << std::endl;
    }
  }
  std::vector<uint8_t> maskData(w*h);
  cv::Mat finalMask(h,w,CV_8U,maskData.data());
  badMask.copyTo(finalMask);

  // label 4-connected components
  std::vector<uint16_t> labelData(w*h);
  cv::Mat labels(h,w,CV_16U,labelData.data());
  int numLabels = labelImage(finalMask, labels);
  {
    std::ofstream ofs("/home/antone/labels.txt");
    for (int i = 0; i < labels.rows; ++i) {
      for (int j = 0; j < labels.cols; ++j) {
        ofs << labels.at<uint16_t>(i,j) << " ";
      }
      ofs << std::endl;
    }
  }

  // extract components and outlines
  std::vector<std::vector<int> > indices;
  std::vector<std::set<int> > outlines;
  extractComponentsAndOutlines(labels, numLabels, indices, outlines);

  // fit planes and fill
  for (int i = 0; i < numLabels; ++i) {
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(outlines[i].size());
    std::set<int>::const_iterator iter = outlines[i].begin();
    for (; iter != outlines[i].end(); ++iter) {
      int idx = *iter;
      float z = depths[idx];
      if (z == invalidValue) continue;
      pts.push_back(Eigen::Vector3f(idx%w, idx/w, z));
    }
    if (pts.size() < 3) continue;
    Eigen::Vector3f plane = fitHorizontalPlaneRobust(pts);

    std::vector<int>::const_iterator it = indices[i].begin();
    for (; it != indices[i].end(); ++it) {
      int idx = *it;
      float x(idx%w), y(idx/w);
      depths[idx] = -(plane[0]*x + plane[1]*y + plane[2]);
    }
  }

  img->setData(depths, type);
}


void FillMethods::
fillIterative(std::shared_ptr<maps::DepthImageView>& iView,
              const int iMaxPasses) {
  // get view data
  maps::DepthImage::Type type = maps::DepthImage::TypeDepth;
  maps::DepthImage::Ptr img = iView->getDepthImage();
  std::vector<float>& depths =
    const_cast<std::vector<float>&>(img->getData(type));
  const float invalidValue = img->getInvalidValue(type);

  int w(img->getWidth()), h(img->getHeight());
  int curPass = 0;
  while(true) {
    int updateCount = 0;

    for (int i = 1; i < h-1; ++i) {
      for (int j = 1; j < w-1; ++j) {
        int idx = i*w+j;
        float z = depths[idx];
        if (z != invalidValue) continue;

        // look at neighbors
        int goodNeighbors = 0;
        float total = 0;
        float val;
        val = depths[idx+1];
        if (val != invalidValue) { total += val; ++goodNeighbors; }
        val = depths[idx-1];
        if (val != invalidValue) { total += val; ++goodNeighbors; }
        val = depths[idx+w];
        if (val != invalidValue) { total += val; ++goodNeighbors; }
        val = depths[idx-w];
        if (val != invalidValue) { total += val; ++goodNeighbors; }

        // set to average if there are enough neighbors
        if (goodNeighbors < 3) continue;
        depths[idx] = total/goodNeighbors;
        ++updateCount;
      }
    }

    if (updateCount == 0) break;
    ++curPass;
    if ((iMaxPasses>0) && (curPass >= iMaxPasses)) break;
  }

  img->setData(depths, type);
}
