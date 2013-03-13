#include "DepthImageView.hpp"

#include <pcl/range_image/range_image.h>
#include <pcl/io/io.h>
#include "Utils.hpp"

using namespace maps;

namespace pcl {
class DepthImageProjective : public RangeImage {
public:
  typedef RangeImage BaseClass;
  typedef boost::shared_ptr<DepthImageProjective> Ptr;
  typedef boost::shared_ptr<const DepthImageProjective> ConstPtr;

public:
  DepthImageProjective() {
    mProjector = Eigen::Projective3f::Identity();
    mProjectorInv = Eigen::Projective3f::Identity();
    unobserved_point.range = std::numeric_limits<float>::infinity();
  }
  ~DepthImageProjective() {}

  RangeImage* getNew() const { return new DepthImageProjective(); }
  Ptr makeShared() { return Ptr(new DepthImageProjective(*this)); }

  void setParams(const int iWidth, const int iHeight,
                 const Eigen::Projective3f& iTransform,
                 const CoordinateFrame iFrame) {
    reset();
    width = iWidth;
    height = iHeight;
    mProjector = iTransform;
    mProjectorInv = mProjector.inverse();
    Eigen::Matrix3f calib;
    Eigen::Isometry3f pose;
    bool ortho;
    Utils::factorViewMatrix(mProjector, calib, pose, ortho);
    is_dense = false;
    getCoordinateFrameTransformation(iFrame, to_world_system_);
    to_world_system_ = pose * to_world_system_;
    to_range_image_system_ = to_world_system_.inverse(Eigen::Isometry);
    points.clear();
    points.resize(width*height, unobserved_point);
  }

  // create from point cloud and composite matrix
  template <typename PointCloudType> void
  create(const PointCloudType& iCloud, const int iWidth, const int iHeight,
         const Eigen::Projective3f& iTransform,
         const CoordinateFrame iFrame=CAMERA_FRAME,
         const float iNoiseLevel=0.0f, const float iMinDepth=0.0f) {
    setParams(iWidth, iHeight, iTransform, iFrame);
    int top(height), right(-1), bottom(-1), left(width);
    doZBuffer(iCloud, iNoiseLevel, iMinDepth, top, right, bottom, left);
    recalculate3DPointPositions();
  }

  // create from array of values (assumed to be iWidth*iHeight in length)
  void create(const std::vector<float>& iVals,
              const int iWidth, const int iHeight,
              const Eigen::Projective3f& iTransform,
              const CoordinateFrame iFrame=CAMERA_FRAME) {
    setParams(iWidth, iHeight, iTransform, iFrame);
    for (int y = 0; y < iHeight; ++y) {
      for (int x = 0; x < iWidth; ++x) {
        PointWithRange& curPoint = getPointNoCheck(x,y);
        float depth = iVals[y*iWidth+x];
        if (!pcl_isfinite(depth)) {
          curPoint = unobserved_point;
          continue;
        }
        Eigen::Vector3f pt;
        calculate3DPoint(x, y, depth, pt);
        curPoint.x = pt[0];
        curPoint.y = pt[1];
        curPoint.z = pt[2];
        curPoint.range = depth;
      }
    }
  }

  inline void calculate3DPoint(float iX, float iY, float iDepth,
                               Eigen::Vector3f& oPoint) const {
    Eigen::Vector4f pt = mProjectorInv*Eigen::Vector4f(iX, iY, iDepth, 1);
    oPoint = pt.head<3>()/pt[3];
  }

  inline void getImagePoint(const Eigen::Vector3f& iPoint,
                            float& oX, float& oY, float& oDepth) const {
    Eigen::Vector4f inPt(iPoint[0], iPoint[1], iPoint[2], 1);
    Eigen::Vector4f pt = mProjector*inPt;
    Eigen::Vector3f outPt = pt.head<3>()/pt[3];
    oX = outPt[0];
    oY = outPt[1];
    oDepth = outPt[2];
  }
      
  void getSubImage(int iOffsetX, int iOffsetY, int iWidth, int iHeight,
                   int iCombine, RangeImage& oImage) const {
    DepthImageProjective& img = *(static_cast<DepthImageProjective*>(&oImage));
    Eigen::Translation3f shift(-iOffsetX, -iOffsetY, 0);
    Eigen::Affine3f scale = Eigen::Affine3f::Identity();
    scale.linear() = Eigen::Scaling(1.0f/iCombine, 1.0f/iCombine, 1.0f);
    BaseClass::getSubImage(iOffsetX, iOffsetY, iWidth, iHeight, iCombine, img);
    img.image_offset_x_ = img.image_offset_y_ = 0;
    img.mProjector = scale*shift*mProjector;
    img.mProjectorInv = img.mProjector.inverse();
  }

  void getHalfImage(RangeImage& oImage) const {
    DepthImageProjective& img = *(static_cast<DepthImageProjective*>(&oImage));
    Eigen::Affine3f scale = Eigen::Affine3f::Identity();
    scale.linear() = Eigen::Scaling(0.5f, 0.5f, 1.0f);
    BaseClass::getHalfImage(img);
    img.mProjector = scale*mProjector;
    img.mProjectorInv = img.mProjector.inverse();
  }
      
protected:
  Eigen::Projective3f mProjector;
  Eigen::Projective3f mProjectorInv;
};
}

DepthImageView::
DepthImageView() {
  mImage.reset(new pcl::DepthImageProjective());
  mImage->width = mImage->height = 0;
}

DepthImageView::
~DepthImageView() {
}

boost::shared_ptr<pcl::RangeImage> DepthImageView::
getRangeImage() const {
  return mImage;
}

void DepthImageView::
setSize(const int iWidth, const int iHeight) {
  mImage->width = iWidth;
  mImage->height = iHeight;
}

void DepthImageView::
set(const std::vector<float>& iData, const int iWidth, const int iHeight) {
  int width(iWidth), height(iHeight);
  if (width == 0) width = mImage->width;
  if (height == 0) height = mImage->height;
  pcl::DepthImageProjective* img =
    dynamic_cast<pcl::DepthImageProjective*>(mImage.get());
  img->create(iData, width, height, mTransform);
}

ViewBase::Ptr DepthImageView::
clone() const {
  DepthImageView* view = new DepthImageView(*this);
  view->mImage = view->mImage->makeShared();
  return Ptr(view);
}

const ViewBase::Type DepthImageView::
getType() const {
  return TypeDepthImage;
}

void DepthImageView::
set(const maps::PointCloud::Ptr& iCloud) {
  pcl::DepthImageProjective* img =
    dynamic_cast<pcl::DepthImageProjective*>(mImage.get());
  img->create(*iCloud, mImage->width, mImage->height, mTransform);
}

maps::PointCloud::Ptr DepthImageView::
getAsPointCloud(const bool iTransform) const {
  maps::PointCloud::Ptr cloud(new maps::PointCloud());
  if (iTransform) {
    pcl::copyPointCloud(*mImage, *cloud);
  }
  else {
    cloud->reserve(mImage->width*mImage->height);
    cloud->is_dense = false;
    bool ortho = Utils::isOrthographic(mTransform.matrix());
    float* depths = mImage->getRangesArray();
    for (int i = 0, idx = 0; i < mImage->height; ++i) {
      for (int j = 0; j < mImage->width; ++j, ++idx) {
        if (!pcl_isfinite(depths[idx])) continue;
        maps::PointCloud::PointType pt;
        pt.x = j;
        pt.y = i;
        pt.z = depths[idx];
        cloud->push_back(pt);
      }
    }
  }
  return cloud;
}

maps::TriangleMesh::Ptr DepthImageView::
getAsMesh(const bool iTransform) const {
  int width(mImage->width), height(mImage->height);
  int numDepths = width*height;
  float* depths = mImage->getRangesArray();
  maps::TriangleMesh::Ptr mesh(new maps::TriangleMesh());

  // vertices
  mesh->mVertices.reserve((width+1)*(height+1));
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      float z = depths[i*width+j];
      if (iTransform) {
        Eigen::Vector3f pt;
        mImage->calculate3DPoint(j,i,z,pt);
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
      bool valid00 = fabs(z00) != std::numeric_limits<float>::infinity();
      bool valid10 = fabs(z10) != std::numeric_limits<float>::infinity();
      bool valid01 = fabs(z01) != std::numeric_limits<float>::infinity();
      bool valid11 = fabs(z11) != std::numeric_limits<float>::infinity();
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
  // TODO: getClosest
  return false;
}
