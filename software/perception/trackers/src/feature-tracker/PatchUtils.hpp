#ifndef _tracking_PatchUtils_hpp_
#define _tracking_PatchUtils_hpp_

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace tracking {

class CameraModel;

class PatchUtils {
public:
  static float normalizedCrossCorrelation(const cv::Mat& iA, const cv::Mat& iB);

  static bool interpolate(const cv::Mat& iImage, const Eigen::Vector2f& iCenter,
                          cv::Mat& oPatch);
  static bool interpolate(const cv::Mat& iImage,
                          const Eigen::Affine2f& iTransform, cv::Mat& oPatch);

  static bool computeGradients(const cv::Mat& iImage,
                               cv::Mat& oGx, cv::Mat& oGy);

  static Eigen::Affine2f linearize(const Eigen::Vector2f& iPix,
                                   const CameraModel& iRefCamera,
                                   const CameraModel& iCurCamera,
                                   const Eigen::Vector4f& iPlane);

  // TODO: for debug
  static bool save(const cv::Mat& iImage, const std::string& iFileName);
};

}

#endif
