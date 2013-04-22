#ifndef _tracking_KeyFrame_hpp_
#define _tracking_KeyFrame_hpp_

#include <memory>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace tracking {

class KeyFrame {
public:
  typedef std::shared_ptr<KeyFrame> Ptr;

  struct PyramidLevel {
    typedef std::shared_ptr<PyramidLevel> Ptr;
    cv::Mat mLeftImage;
    cv::Mat mRightImage;
    std::vector<cv::KeyPoint> mLeftKeyPoints;
    std::vector<cv::KeyPoint> mRightKeyPoints;
    cv::Mat mDisparity;
    Eigen::Affine2f mTransformToBase;
  };

public:
  KeyFrame();

  void setId(const int64_t iId);
  int64_t getId() const;

  void setNumPyramidLevels(const int iNum);
  void setFastThreshold(const int iThresh);
  void setSmoothingSigma(const float iSigma);
  void setShouldExtractFeatures(const bool iVal);

  void setData(const cv::Mat& iLeft, const cv::Mat& iRight,
               const cv::Mat& iDisparity);

  int getNumPyramidLevels() const;
  PyramidLevel::Ptr getPyramidLevel(const int iLevel) const;

  void setPose(const Eigen::Isometry3f& iPose);
  Eigen::Isometry3f getPose() const;

protected:
  cv::Mat convert(const cv::Mat& iImage) const;

protected:
  int mNumPyramidLevels;
  int mFastThreshold;
  float mSmoothingSigma;
  bool mShouldExtractFeatures;

  int64_t mId;
  PyramidLevel mOrigData;
  std::vector<PyramidLevel::Ptr> mPyramid;

  Eigen::Isometry3f mPose;
};

}

#endif
