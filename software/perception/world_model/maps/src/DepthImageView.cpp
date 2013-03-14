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
    cloud->reserve(mImage->getWidth()*mImage->getHeight());
    cloud->is_dense = false;
    std::vector<float> depths = mImage->getData(DepthImage::TypeDisparity);
    float invalidValue = mImage->getInvalidValue(DepthImage::TypeDisparity);
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
  std::vector<float> depths = mImage->getData(DepthImage::TypeDisparity);
  maps::TriangleMesh::Ptr mesh(new maps::TriangleMesh());

  // vertices
  mesh->mVertices.reserve((width+1)*(height+1));
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      float z = depths[i*width+j];
      if (iTransform) {
        Eigen::Vector3f pt = mImage->unproject(Eigen::Vector3f(j,i,z),
                                               DepthImage::TypeDisparity);
        mesh->mVertices.push_back(pt);
      }
      else {
        mesh->mVertices.push_back(Eigen::Vector3f(j,i,z));
      }
    }
  }

  // faces
  std::vector<Eigen::Vector3i>& faces = mesh->mFaces;
  faces.reserve(2*numDepths);
  for (int i = 0; i < height-1; ++i) {
    for (int j = 0; j < width-1; ++j) {
      int idx = i*width + j;
      double z00 = depths[idx];
      double z10 = depths[idx+1];
      double z01 = depths[idx+width];
      double z11 = depths[idx+width+1];
      bool valid00 = pcl_isfinite(z00);
      bool valid10 = pcl_isfinite(z10);
      bool valid01 = pcl_isfinite(z01);
      bool valid11 = pcl_isfinite(z11);
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
  /* TODO
  // TODO: maybe interpolate depths not disparities
  Eigen::Vector3f proj = mImage->project(iPoint, DephImage::TypeDisparity);
  const std::vector<float>& disparities =
    mImage->getData(DepthImage::TypeDisparity);
  int width(mImage->getWidth()), height(mImage->getHeight());
  int xInt(proj[0]), yInt(proj[1]);
  if ((xInt < 0) || (xInt >= width-1) || (yInt < 0) || (yInt >= height-1)) {
    return false;
  }
  int idx = width*yInt + xInt;
  float z00 = disparities[idx];
  float z10 = disparities[idx+1];
  float z01 = disparities[idx+width];
  float z11 = disparities[idx+1+width];
  float xFrac = proj[0]-xInt;
  float yFrac = proj[0]-yInt;
  if (xFrac >= yFrac) {
  }
  else {
  }
  float zInterp = z00 + xFrac*(z10 - z00) + yFrac*(z01 - z00) +
    xFrac*yFrac*(z00 + z11 - z10 - z01);
  if (zInterp == mImage->getInvalidValue()) return false;
  oPoint = mImage->unproject(Eigen::Vector3f(proj[0], proj[1], zInterp),
                             DepthImage::TypeDisparity);
  Eigen::Vector3f p00 = mImage->unproject(Eigen::Vector3f(xInt, yInt, z00));
  Eigen::Vector3f p10 = mImage->unproject(Eigen::Vector3f(xInt+1, yInt, z10));
  Eigen::Vector3f p01 = mImage->unproject(Eigen::Vector3f(xInt, yInt+1, z01));
  Eigen::Vector3f p11 = mImage->unproject(Eigen::Vector3f(xInt+1, yInt+1, z11));
  // TODO: closest point
  // TODO: getClosest
  */
  return true;
}
