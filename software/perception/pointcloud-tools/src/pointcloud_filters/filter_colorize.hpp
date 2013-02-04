#ifndef _FilterColorize_hpp_
#define _FilterColorize_hpp_

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <bot_core/camtrans.h>
#include <Eigen/Geometry>
#include <image_utils/pixels.h>

class FilterColorize {

public:

  template<typename PointTypeIn, typename PointTypeOut>
  static bool colorize(const typename pcl::PointCloud<PointTypeIn>& iCloud,
                       const Eigen::Affine3d& iCloudToCamera,
                       const bot_core::image_t& iImage,
                       const BotCamTrans* iCamTrans,
                       typename pcl::PointCloud<PointTypeOut>& oCloud) {
    pcl::copyPointCloud(iCloud, oCloud);
    pcl::PointCloud<PointTypeOut> tempCloud;
    pcl::transformPointCloud(iCloud, tempCloud, iCloudToCamera.cast<float>());

    int numPoints = iCloud.size();
    for (int i = 0; i < numPoints; ++i) {
      double p[3] = {tempCloud[i].x, tempCloud[i].y, tempCloud[i].z};
      double pix[3];
      bot_camtrans_project_point(iCamTrans, p, pix);
      oCloud[i].r = oCloud[i].g = oCloud[i].b = 0;
      if (pix[2] < 0) {
        continue;
      }

      uint8_t r, g, b;
      if (interpolate(pix[0], pix[1], iImage, r, g, b)) {
        oCloud[i].r = r;
        oCloud[i].g = g;
        oCloud[i].b = b;
      }
    }

    return true;
  }

  static bool colorize(const drc::map_image_t& iDepthMap,
                       const Eigen::Affine3d& iLocalToCamera,
                       const bot_core::image_t& iImage,
                       const BotCamTrans* iCamTrans,
                       bot_core::image_t& oImage) {
    oImage.utime = iDepthMap.utime;
    oImage.width = iDepthMap.width;
    oImage.height = iDepthMap.height;
    oImage.row_stride = 3*iDepthMap.width;
    oImage.pixelformat = PIXEL_FORMAT_RGB;
    oImage.size = oImage.row_stride * oImage.height;
    oImage.nmetadata = 0;
    oImage.data.resize(oImage.size);

    Eigen::Matrix4d xform;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        xform(i,j) = iDepthMap.transform[i][j];
      }
    }
    xform = iLocalToCamera.matrix()*xform.inverse();

    for (int i = 0; i < iDepthMap.height; ++i) {
      for (int j = 0; j < iDepthMap.width; ++j) {
        double z;
        if (iDepthMap.format == drc::map_image_t::FORMAT_GRAY_FLOAT32) {
          z = ((float*)(&iDepthMap.data[0] + i*iDepthMap.row_bytes))[j];
          if (z < -1e10) {
            continue;
          }
        }
        else if (iDepthMap.format == drc::map_image_t::FORMAT_GRAY_UINT8) {
          uint8_t val = iDepthMap.data[i*iDepthMap.row_bytes + j];
          if (val == 0) {
            continue;
          }
          z = val;
        }
        else {
          continue;
        }

        Eigen::Vector4d pt = xform*Eigen::Vector4d(j,i,z,1);
        double p[3] = {pt(0)/pt(3),pt(1)/pt(3),pt(2)/pt(3)};
        double pix[3];
        bot_camtrans_project_point(iCamTrans, p, pix);
        if (pix[2] < 0) {
          continue;
        }
        uint8_t r,g,b;
        if (!interpolate(pix[0], pix[1], iImage, r, g, b)) {
          continue;
        }
        int outImageIndex = i*oImage.row_stride + 3*j;
        oImage.data[outImageIndex+0] = r;
        oImage.data[outImageIndex+1] = g;
        oImage.data[outImageIndex+2] = b;
      }
    }

    return true;
  }

  // depth map and image of same dimensions -> colorized point cloud
  template<typename T>
  static bool colorize(const drc::map_image_t& iDepthMap,
                       const bot_core::image_t& iImage,
                       typename pcl::PointCloud<T>::Ptr& oCloud) {
    int w(iDepthMap.width), h(iDepthMap.height);
    if ((w != iImage.width) || (h != iImage.height)) {
      return false;
    }

    // TODO: for now, reject compressed depth maps
    if (iDepthMap.compression != drc::map_image_t::COMPRESSION_NONE) {
      return false;
    }

    // TODO: for now, only accept rgb3 images
    if (iImage.pixelformat != PIXEL_FORMAT_RGB) {
      return false;
    }

    if (oCloud == NULL) {
      oCloud.reset(new pcl::PointCloud<T>());
    }
    oCloud->points.clear();

    Eigen::Matrix4d xform;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        xform(i,j) = iDepthMap.transform[i][j];
      }
    }
    xform = xform.inverse();

    for (int i = 0; i < h; ++i) {
      for (int j = 0; j < w; ++j) {
        double z;
        if (iDepthMap.format == drc::map_image_t::FORMAT_GRAY_FLOAT32) {
          z = ((float*)(&iDepthMap.data[0] + i*iDepthMap.row_bytes))[j];
          if (z < -1e10) {
            continue;
          }
        }
        else if (iDepthMap.format == drc::map_image_t::FORMAT_GRAY_UINT8) {
          uint8_t val = iDepthMap.data[i*iDepthMap.row_bytes + j];
          if (val == 0) {
            continue;
          }
          z = val;
        }
        else {
          continue;
        }

        Eigen::Vector4d pt = xform*Eigen::Vector4d(j,i,z,1);
        T newPoint;
        newPoint.x = pt(0)/pt(3);
        newPoint.y = pt(1)/pt(3);
        newPoint.z = pt(2)/pt(3);
        int imageIndex = i*iImage.row_stride + 3*j;
        newPoint.r = iImage.data[imageIndex+0];
        newPoint.g = iImage.data[imageIndex+1];
        newPoint.b = iImage.data[imageIndex+2];
        oCloud->points.push_back(newPoint);
      }
    }
    oCloud->width = oCloud->points.size();
    oCloud->height = 1;
    oCloud->is_dense = false;

    return true;
  }

private:

  static inline bool interpolate(const float iX, const float iY,
                                 const bot_core::image_t& iImage,
                                 uint8_t& oR, uint8_t& oG, uint8_t& oB) {
    int xInt(floor(iX)), yInt(floor(iY));
    if ((xInt < 0) || (xInt >= (iImage.width-1)) ||
        (yInt < 0) || (yInt >= (iImage.height-1))) {
      return false;
    }

    float xFrac = iX-xInt;
    float yFrac = iY-yInt;

    int idxUL = yInt*iImage.row_stride + 3*xInt;
    int idxUR = idxUL+3;
    int idxLL = idxUL+iImage.row_stride;
    int idxLR = idxLL+3;

    uint8_t out[3];
    for (int i = 0; i < 3; ++i) {
      int16_t valUL = iImage.data[idxUL+i];
      int16_t valUR = iImage.data[idxUR+i];
      int16_t valLL = iImage.data[idxLL+i];
      int16_t valLR = iImage.data[idxLR+i];
      float val = valUL + xFrac*(valUR - valUL) + yFrac*(valLL - valUL) +
        xFrac*yFrac*(valUL + valLR - valUR - valLL);
      out[i] = val + 0.5f;
    }
    oR = out[0];
    oG = out[1];
    oB = out[2];
    return true;
  }
};

#endif
