#include "DepthImageView.hpp"

#include "DepthImage.hpp"
#include "Utils.hpp"

using namespace maps;

DepthImageView::
DepthImageView() {
  mImage.reset(new DepthImage());
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

boost::shared_ptr<DepthImage> DepthImageView::
getDepthImage() const {
  return mImage;
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
    const std::vector<float>& depths = mImage->getData(depthType);
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
  std::vector<float> depths = mImage->getData(depthType);
  maps::TriangleMesh::Ptr mesh(new maps::TriangleMesh());

  // vertices
  mesh->mVertices.reserve((width+1)*(height+1));
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      Eigen::Vector3f pt(j,i,depths[i*width+j]);
      if (iTransform) pt = mImage->unproject(pt, depthType);
      mesh->mVertices.push_back(pt);
    }
  }

  // faces
  std::vector<Eigen::Vector3i>& faces = mesh->mFaces;
  faces.reserve(2*numDepths);
  float invalidValue = mImage->getInvalidValue(depthType);
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
        faces.push_back(Eigen::Vector3i(idx, idx+1, idx+width));
        faces.push_back(Eigen::Vector3i(idx+1+width, idx+1, idx+width));
      }	  
      else {
        if (!valid00) {
          faces.push_back(Eigen::Vector3i(idx+1, idx+width, idx+1+width));
        }
        else if (!valid10) {
          faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1+width));
        }
        else if (!valid01) {
          faces.push_back(Eigen::Vector3i(idx, idx+1+width, idx+1));
        }
        else if (!valid11) {
          faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1));
        }
      }
    }
  }

  return mesh;
}

bool DepthImageView::
interpolate(const float iX, const float iY, float& oDepth) const {
  const DepthImage::Type depthType = DepthImage::TypeDepth;
  const std::vector<float>& depths = mImage->getData(depthType);
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
  return unproject(proj, oPoint, oNormal);
}

bool DepthImageView::
unproject(const Eigen::Vector3f& iPoint, Eigen::Vector3f& oPoint,
          Eigen::Vector3f& oNormal) const {
  typedef Eigen::Vector3f Vec3f;
  const DepthImage::Type depthType = DepthImage::TypeDepth;
  Eigen::Vector3f outputPoint = mImage->unproject(iPoint, depthType);
  int xInt(iPoint[0]), yInt(iPoint[1]);
  int width = mImage->getWidth();
  int idx = width*yInt + xInt;
  const std::vector<float>& depths = mImage->getData(depthType);
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

  oPoint = outputPoint;

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
