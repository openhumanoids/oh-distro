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
getClosest(const Eigen::Vector3f& iPoint,
           Eigen::Vector3f& oPoint, Eigen::Vector3f& oNormal) {
  DepthImage::Type depthType = DepthImage::TypeDepth;
  Eigen::Vector3f proj = mImage->project(iPoint, depthType);
  const std::vector<float>& depths = mImage->getData(depthType);
  int width(mImage->getWidth()), height(mImage->getHeight());
  int xInt(proj[0]), yInt(proj[1]);
  if ((xInt < 0) || (xInt >= width-1) || (yInt < 0) || (yInt >= height-1)) {
    return false;
  }
  float xFrac = proj[0]-xInt;
  float yFrac = proj[0]-yInt;

  int idx = width*yInt + xInt;
  float z00 = depths[idx];
  float z11 = depths[idx+1+width];
  typedef Eigen::Vector3f Vec3f;
  Vec3f p00 = mImage->unproject(Vec3f(xInt, yInt, z00), depthType);
  Vec3f p11 = mImage->unproject(Vec3f(xInt+1, yInt+1, z11), depthType);

  if (xFrac >= yFrac) {
    float z3 = depths[idx+1];
    float zInterp = xFrac*(z3 - z00) + yFrac*(z11 - z3) + z00;
    Vec3f p3 = mImage->unproject(Vec3f(xInt+1, yInt, z3), depthType);
    Vec3f d1(p00-p3), d2(p11-p3);
    oNormal = d1.cross(d2).normalized();
  }
  else {
    float z3 = depths[idx+width];
    float zInterp = xFrac*(z11 - z3) + yFrac*(z3 - z00) + z00;
    Vec3f p3 = mImage->unproject(Vec3f(xInt, yInt+1, z3), depthType);
    Vec3f d1(p00-p3), d2(p11-p3);
    oNormal = d2.cross(d1).normalized();
  }

  return true;
}
