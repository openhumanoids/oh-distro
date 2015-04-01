#include "DepthImageView.hpp"

#include <iostream>

#include "DepthImage.hpp"
#include "RansacGeneric.hpp"

using namespace maps;

DepthImageView::
DepthImageView() {
  mImage.reset(new DepthImage());
  setNormalRadius(0);
  setNormalMethod(NormalMethodTriangle);
}

DepthImageView::
~DepthImageView() {
}

void DepthImageView::
setSize(const int iWidth, const int iHeight) {
  mImage->setSize(iWidth, iHeight);
}

void DepthImageView::
set(const DepthImage& iImage) {
  mImage.reset(new DepthImage(iImage));
  setTransform(mImage->getProjector());
}

std::shared_ptr<DepthImage> DepthImageView::
getDepthImage() const {
  return mImage;
}

void DepthImageView::
setNormalRadius(const int iRadiusPixels) {
  mNormalRadius = iRadiusPixels;
}

void DepthImageView::
setNormalMethod(const NormalMethod iMethod) {
  mNormalMethod = iMethod;
}

ViewBase::Ptr DepthImageView::
clone() const {
  return Ptr(new DepthImageView(*this));
}

const ViewBase::Type DepthImageView::
getType() const {
  return TypeDepthImage;
}

const std::vector<float>& DepthImageView::
getInnerData(const int iType) const {
  return mImage->getData(DepthImage::Type(iType));
}


bool DepthImageView::
interpolate(const float iX, const float iY, float& oDepth) const {
  const DepthImage::Type depthType = DepthImage::TypeDepth;
  const std::vector<float>& depths = getInnerData(depthType);
  int width(mImage->getWidth()), height(mImage->getHeight());
  int xInt(iX), yInt(iY);
  if ((xInt < 0) || (xInt >= width-1) || (yInt < 0) || (yInt >= height-1)) {
    return false;
  }
  float xFrac(iX-xInt), yFrac(iY-yInt);

  int idx = width*yInt + xInt;
  float z00 = depths[idx];
  float z11 = depths[idx+1+width];
  float invalidValue = mImage->getInvalidValue(depthType);
  if ((z00 == invalidValue) || (z11 == invalidValue)) return false;

  oDepth = 0;
  if (xFrac >= yFrac) {
    float z3 = depths[idx+1];
    if (z3 == invalidValue) return false;
    oDepth = xFrac*(z3 - z00) + yFrac*(z11 - z3) + z00;
  }
  else {
    float z3 = depths[idx+width];
    if (z3 == invalidValue) return false;
    oDepth = xFrac*(z11 - z3) + yFrac*(z3 - z00) + z00;
  }

  return true;
}

bool DepthImageView::
getClosest(const Eigen::Vector3f& iPoint,
           Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal) const {
  DepthImage::Type depthType = DepthImage::TypeDepth;
  Eigen::Vector3f proj = mImage->project(iPoint, depthType);
  if (!interpolate(proj[0], proj[1], proj[2])) return false;

  // do triangle-based interpolated point and normal
  if ((mNormalMethod == NormalMethodTriangle) ||
      ((mNormalRadius == 0) && (mNormalMethod != NormalMethodZ))) {
    return unproject(proj, oPoint, oNormal);
  }

  // do neighborhood plane fit to find normal
  else {
    // gather points in neighborhood
    const std::vector<float>& depths = getInnerData(depthType);
    const int width = mImage->getWidth();
    const int height = mImage->getHeight();
    const int xInt(proj[0]), yInt(proj[1]);
    const int xMin = std::max(0, xInt-mNormalRadius);
    const int xMax = std::min(width-1, xInt+mNormalRadius);
    const int yMin = std::max(0, yInt-mNormalRadius);
    const int yMax = std::min(height-1, yInt+mNormalRadius);
    const float invalidValue = mImage->getInvalidValue(depthType);
    std::vector<Eigen::Vector3f> points;
    points.reserve((2*mNormalRadius+1)*(2*mNormalRadius+1));
    for (int y = yMin; y <= yMax; ++y) {
      for (int x = xMin; x <= xMax; ++x) {
        int idx = y*width + x;
        float z = depths[idx];
        if (z == invalidValue) continue;
        Eigen::Vector3f pt =
          mImage->unproject(Eigen::Vector3f(x,y,z), depthType);
        points.push_back(pt);
      }
    }

    // form data matrix and solve
    Eigen::MatrixX3f matx(points.size(), 3);
    for (int i = 0; i < (int)points.size(); ++i) {
      matx.row(i) = points[i];
    }
    Eigen::Vector4f plane;
    switch (mNormalMethod) {
    case NormalMethodLeastSquares:
      if (!fitPlane(matx, Eigen::VectorXf(), plane)) return false;
      break;
    case NormalMethodRobustKernel:
      if (!fitPlaneRobust(matx, plane)) return false;
      break;
    case NormalMethodSampleConsensus:
      if (!fitPlaneSac(matx, plane)) return false;
      break;
    case NormalMethodZ:
      plane.head<3>() = Eigen::Vector3f::UnitZ();
      plane[3] = 0;  // TODO: use ground level
      break;
    default:
      std::cout << "Invalid normal method specified!" << std::endl;
      return false;
    }
    oNormal = plane.head<3>();

    // project point onto plane
    Eigen::Vector3f pt = mImage->unproject(proj, depthType);
    /* TODO: TEMP?
    Eigen::Vector3f p0 =
      mImage->unproject(proj-Eigen::Vector3f::UnitZ(), depthType);
    Eigen::Vector3f depthRay = pt-p0;
    float t = -(plane[3] + oNormal.dot(p0)) / oNormal.dot(depthRay);
    oPoint = p0 + depthRay*t;
    */
    oPoint = pt;
    Eigen::Vector3f viewRay = mTransform.inverse().translation() - pt;
    if (viewRay.dot(oNormal) < 0) oNormal = -oNormal;
    return true;
  }
}

bool DepthImageView::
unproject(const Eigen::Vector3f& iPoint, Eigen::Vector3f& oPoint,
          Eigen::Vector3f& oNormal) const {
  typedef Eigen::Vector3f Vec3f;
  const DepthImage::Type depthType = DepthImage::TypeDepth;
  oPoint = mImage->unproject(iPoint, depthType);
  int xInt(iPoint[0]), yInt(iPoint[1]);
  int width = mImage->getWidth();
  int idx = width*yInt + xInt;
  const std::vector<float>& depths = getInnerData(depthType);
  Vec3f p00 = mImage->unproject(Vec3f(xInt, yInt, depths[idx]), depthType);
  Vec3f p11 = mImage->unproject(Vec3f(xInt+1, yInt+1, depths[idx+1+width]),
                                      depthType);
  
  float xFrac(iPoint[0]-xInt), yFrac(iPoint[1]-yInt);
  float invalidValue = mImage->getInvalidValue(depthType);
  if (xFrac >= yFrac) {
    float z3 = depths[idx+1];
    if (z3 == invalidValue) return false;
    Vec3f p3 = mImage->unproject(Vec3f(xInt+1, yInt, z3), depthType);
    Vec3f d1(p00-p3), d2(p11-p3);
    oNormal = d1.cross(d2).normalized();
  }
  else {
    float z3 = depths[idx+width];
    if (z3 == invalidValue) return false;
    Vec3f p3 = mImage->unproject(Vec3f(xInt, yInt+1, z3), depthType);
    Vec3f d1(p00-p3), d2(p11-p3);
    oNormal = d2.cross(d1).normalized();
  }

  return true;
}

namespace {
  struct HorizontalPlaneFitProblem {
    typedef Eigen::Vector3f Solution;
    Eigen::MatrixX3f* mPoints;

    int getNumDataPoints() const { return mPoints->rows(); }
    int getSampleSize() const { return 3; }

    Solution estimate(const std::vector<int> iIndices) const {
      int n = iIndices.size();
      Eigen::VectorXf rhs(n);
      Eigen::MatrixXf lhs(n,3);
      for (int i = 0; i < n; ++i) {
        const Eigen::Vector3f& p = mPoints->row(iIndices[i]);
        rhs[i] = -p[2];
        lhs(i,0) = p[0];
        lhs(i,1) = p[1];
        lhs(i,2) = 1;
      }
      Eigen::Vector3f sol =
        lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(rhs);
      return sol;
    }

    std::vector<double>
    computeSquaredErrors(const Solution& iPlane) const {
      size_t n = mPoints->rows();
      std::vector<double> e2(n);
      for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3f& p = mPoints->row(i);
        double e = p[0]*iPlane[0] + p[1]*iPlane[1] + p[2] + iPlane[2];
        e2[i] = e*e;
      }
      return e2;
    }
  };
}

bool DepthImageView::
fitPlaneSac(const Eigen::MatrixX3f& iPoints, Eigen::Vector4f& oPlane) {
  HorizontalPlaneFitProblem problem;
  problem.mPoints = const_cast<Eigen::MatrixX3f*>(&iPoints);
  RansacGeneric<HorizontalPlaneFitProblem> ransacObj;
  ransacObj.setMaximumError(0.01);
  ransacObj.setRefineUsingInliers(true);
  ransacObj.setMaximumIterations(100);
  RansacGeneric<HorizontalPlaneFitProblem>::Result result =
    ransacObj.solve(problem);
  oPlane << result.mSolution[0],result.mSolution[1],1,result.mSolution[2];
  oPlane /= oPlane.head<3>().norm();
  return result.mSuccess;
}

bool DepthImageView::
fitPlaneRobust(const Eigen::MatrixX3f& iPoints, Eigen::Vector4f& oPlane) {
  const int n = iPoints.rows();
  if (n < 3) return false;

  Eigen::VectorXf weights = Eigen::VectorXf::Ones(n);
  Eigen::VectorXf weightsPrev = Eigen::VectorXf::Zero(n);
  const float weightThresh = (1e-3f * 1e-3f)*n;
  const int maxIter = 10;
  int iter;
  for (iter = 0; iter < maxIter; ++iter) {
    fitPlane(iPoints, weights, oPlane);

    // compute robust sigma
    Eigen::VectorXf dists2 = (iPoints*oPlane.head<3>()).array() + oPlane[3];
    dists2.array() *= dists2.array();
    std::sort(dists2.data(), dists2.data()+n);
    float medVal = (n%2 != 0) ? dists2[n/2] : 0.5*(dists2[n/2-1] + dists2[n/2]);
    float sigma2 = std::max(1.4826*sqrt(medVal)*4.7851, 1e-6);
    sigma2 *= sigma2;

    // check to see if weights are unchanged
    Eigen::VectorXf weightDiff = weightsPrev-weights;
    if (weightDiff.dot(weightDiff) < weightThresh) break;

    // recompute weights
    weightsPrev = weights;
    weights = 1-dists2.array()/sigma2;
    weights = (weights.array() < 0).select(0.0f, weights);
  }
  return true;
}

bool DepthImageView::
fitPlane(const Eigen::MatrixX3f& iPoints, const Eigen::VectorXf& iWeights,
         Eigen::Vector4f& oPlane) {
  if ((iWeights.size() != 0) && (iPoints.rows() != iWeights.size())) {
    return false;
  }
  if (iPoints.rows() < 3) return false;

  // compute mean
  Eigen::Vector3f mean;
  if (iWeights.size() > 0) {
    mean = (iWeights.asDiagonal()*iPoints).colwise().sum() / iWeights.sum();
  }
  else {
    mean = iPoints.colwise().sum() / iPoints.rows();
  }

  return fitPlane(iPoints, mean, iWeights, oPlane);
}

bool DepthImageView::
fitPlane(const Eigen::MatrixX3f& iPoints, const Eigen::Vector3f& iPointOnPlane,
         const Eigen::VectorXf& iWeights, Eigen::Vector4f& oPlane) {
  Eigen::MatrixX3f data(iPoints.rows(),3);
  for (int k = 0; k < 3; ++k) {
    data.col(k).array() = iPoints.col(k).array() - iPointOnPlane[k];
  }

  // compute principal direction via svd
  if (iWeights.size() > 0) {
    data = iWeights.asDiagonal()*data;
  }
  oPlane.head<3>() = data.jacobiSvd(Eigen::ComputeFullV).matrixV().col(2);

  // compute plane offset
  oPlane[3] = -iPointOnPlane.dot(oPlane.head<3>());

  return true;
}
