#include "DepthImageView.hpp"

#include "DepthImage.hpp"
#include "Utils.hpp"
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

void DepthImageView::
set(const maps::PointCloud::Ptr& iCloud) {
  mImage->setProjector(mTransform);
  mImage->create(iCloud);
}

maps::PointCloud::Ptr DepthImageView::
getAsPointCloud(const bool iTransform) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  if (iTransform) {
    cloud = mImage->getAsPointCloud();
  }
  else {
    DepthImage::Type depthType = DepthImage::TypeDisparity;
    cloud->reserve(mImage->getWidth()*mImage->getHeight());
    cloud->is_dense = false;
    const std::vector<float>& depths = getInnerData(depthType);
    const float invalidValue = mImage->getInvalidValue(depthType);
    for (int i = 0, idx = 0; i < mImage->getHeight(); ++i) {
      for (int j = 0; j < mImage->getWidth(); ++j, ++idx) {
        float z = depths[idx];
        if (z == invalidValue) continue;
        maps::PointCloud::PointType pt;
        pt.x = j;
        pt.y = i;
        pt.z = z;
        cloud->push_back(pt);
      }
    }
  }
  return cloud;
}

maps::TriangleMesh::Ptr DepthImageView::
getAsMesh(const bool iTransform) const {
  int width(mImage->getWidth()), height(mImage->getHeight());
  int numDepths = width*height;
  DepthImage::Type depthType = DepthImage::TypeDisparity;
  std::vector<float> depths = getInnerData(depthType);
  maps::TriangleMesh::Ptr mesh(new maps::TriangleMesh());

  // vertices
  std::vector<Eigen::Vector3f>& vertices = mesh->mVertices;
  vertices.reserve((width+1)*(height+1));
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      Eigen::Vector3f pt(j,i,depths[i*width+j]);
      if (iTransform) pt = mImage->unproject(pt, depthType);
      vertices.push_back(pt);
    }
  }

  // faces
  std::vector<Eigen::Vector3i>& faces = mesh->mFaces;
  faces.reserve(2*numDepths);
  std::vector<std::vector<int> > vertexMap(numDepths);
  float invalidValue = mImage->getInvalidValue(depthType);
  int faceCount = 0;
  for (int i = 0; i < height-1; ++i) {
    for (int j = 0; j < width-1; ++j) {
      int idx = i*width + j;
      double z00 = depths[idx];
      double z10 = depths[idx+1];
      double z01 = depths[idx+width];
      double z11 = depths[idx+width+1];
      bool valid00 = z00 != invalidValue;
      bool valid10 = z10 != invalidValue;
      bool valid01 = z01 != invalidValue;
      bool valid11 = z11 != invalidValue;
      int validSum = (int)valid00 + (int)valid10 + (int)valid01 + (int)valid11;
      if (validSum < 3) continue;

      Eigen::Vector3f p00(j,i,z00);
      Eigen::Vector3f p10(j+1,i,z10);
      Eigen::Vector3f p01(j,i+1,z01);
      Eigen::Vector3f p11(j+1,i+1,z11);

      if (validSum == 4) {
        faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1));
        vertexMap[idx].push_back(faceCount);
        vertexMap[idx+width].push_back(faceCount);
        vertexMap[idx+1].push_back(faceCount);
        ++faceCount;

        faces.push_back(Eigen::Vector3i(idx+1+width, idx+1, idx+width));
        vertexMap[idx+1+width].push_back(faceCount);
        vertexMap[idx+1].push_back(faceCount);
        vertexMap[idx+width].push_back(faceCount);
        ++faceCount;
      }	  
      else {
        if (!valid00) {
          faces.push_back(Eigen::Vector3i(idx+1, idx+width, idx+1+width));
          vertexMap[idx+1].push_back(faceCount);
          vertexMap[idx+width].push_back(faceCount);
          vertexMap[idx+1+width].push_back(faceCount);
          ++faceCount;
        }
        else if (!valid10) {
          faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1+width));
          vertexMap[idx].push_back(faceCount);
          vertexMap[idx+width].push_back(faceCount);
          vertexMap[idx+1+width].push_back(faceCount);
          ++faceCount;
        }
        else if (!valid01) {
          faces.push_back(Eigen::Vector3i(idx, idx+1+width, idx+1));
          vertexMap[idx].push_back(faceCount);
          vertexMap[idx+1+width].push_back(faceCount);
          vertexMap[idx+1].push_back(faceCount);
          ++faceCount;
        }
        else if (!valid11) {
          faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1));
          vertexMap[idx].push_back(faceCount);
          vertexMap[idx+width].push_back(faceCount);
          vertexMap[idx+1].push_back(faceCount);
          ++faceCount;
        }
      }
    }
  }

  // normals
  std::vector<Eigen::Vector3f> perFaceNormals(faces.size());
  for (int i = 0; i < faces.size(); ++i) {
    Eigen::Vector3f p1(vertices[faces[i][0]]);
    Eigen::Vector3f p2(vertices[faces[i][1]]);
    Eigen::Vector3f p3(vertices[faces[i][2]]);
    perFaceNormals[i] = (p3-p2).cross(p1-p2).normalized();
  }
  std::vector<Eigen::Vector3f>& normals = mesh->mNormals;
  normals.reserve(numDepths);
  for (int i = 0; i < vertexMap.size(); ++i) {
    int num = vertexMap[i].size();
    if (num == 0) normals.push_back(Eigen::Vector3f(0,0,0));
    else {
      Eigen::Vector3f avg(0,0,0);
      for (int j = 0; j < num; ++j) {
        avg += perFaceNormals[vertexMap[i][j]];
      }
      avg /= num;
      normals.push_back(avg.normalized());
    }
  }

  return mesh;
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
    for (int i = 0; i < points.size(); ++i) {
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

bool DepthImageView::
intersectRay(const Eigen::Vector3f& iOrigin,
             const Eigen::Vector3f& iDirection,
             Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal) const {
  const DepthImage::Type depthType = DepthImage::TypeDepth;

  // project origin and direction into image
  Eigen::Vector3f origin = mImage->project(iOrigin, depthType);
  Eigen::Vector3f pt = mImage->project(iOrigin+iDirection, depthType);
  Eigen::Vector3f direction = pt-origin;
  direction.normalize();

  // check for perfectly orthogonal ray
  if (fabs(fabs(direction[2])-1) < 1e-4) {
    if (!interpolate(origin[0], origin[1], origin[2])) return false;
    return unproject(origin, oPoint, oNormal);
  }

  Eigen::Vector3f startPt = origin;
  Eigen::Vector3f endPt = startPt + 1000*direction;
  
  float dx(fabs(direction[0])), dy(fabs(direction[1]));
  Eigen::Vector3f step(direction / ((dx > dy) ? dx : dy));

  // clip ray segment to bounds of image
  // TODO: 2d clip would be faster
  float tMin, tMax;
  int width = mImage->getWidth();
  int height = mImage->getHeight();
  if (!Utils::clipRay(startPt, endPt, Eigen::Vector3f(0,0,-1e10),
                      Eigen::Vector3f(width-1, height-1, 1e10),
                      startPt, endPt, tMin, tMax)) return false;

  // walk along ray
  // TODO: could make this faster using integer math
  int numSteps = floor((endPt-startPt).norm()/step.norm()) + 1;
  Eigen::Vector3f rayPt(startPt), rayPt1(startPt), rayPt2(startPt);
  bool found = false;
  bool initialized = false;
  float zMesh, zMesh1, zMesh2;
  bool signInit;
  for (int i = 0; i < numSteps; ++i, rayPt += step) {
    if (!interpolate(rayPt[0], rayPt[1], zMesh)) continue;
    bool sign = rayPt[2] > zMesh;
    if (!initialized) signInit = sign;
    if ((initialized) && (sign != signInit)) {
      rayPt2 = rayPt;
      zMesh2 = zMesh;
      found = true;
      break;
    }
    else {
      rayPt1 = rayPt;
      zMesh1 = zMesh;
    }
    initialized = true;
  }
  if (!found) return false;

  // we have now bounded the intersection to lie between meshPt1 and meshPt2
  // now interpolate the intersection point in between
  // TODO: this is approximate; it lies on the mesh but
  //   not necessarily on the ray
  float height1(fabs(rayPt1[2]-zMesh1)), height2(fabs(rayPt2[2]-zMesh2));
  oPoint = rayPt1 + (rayPt2-rayPt1)*height1/(height1+height2);
  oPoint = mImage->unproject(rayPt1, depthType);
  return true;
  if (!interpolate(oPoint[0], oPoint[1], oPoint[2])) return false;
  return unproject(oPoint, oPoint, oNormal);
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
