#ifndef _tracking_KeyFrame_hpp_
#define _tracking_KeyFrame_hpp_

#include "PyramidLevel.hpp"

class KeyFrame {
public:
  typedef std::shared_ptr<KeyFrame> Ptr;

public:
  KeyFrame();

  void setId(const int iId);
  int getId() const;

  void setNumPyramidLevels(const int iNum);
  void setFastThreshold(const int iThresh);
  void setSmoothingSigma(const float iSigma);

  void setImage(const cv::Mat& iImage);

  cv::Mat getOrigImage() const;
  PyramidLevel::Ptr getPyramidLevel(const int iLevel);

protected:
  int mId;

  int mNumPyramidLevels;
  int mFastThreshold;
  float mSmoothingSigma;

  cv::Mat mOrigImage;
  std::vector<PyramidLevel::Ptr> mPyramid;
};


#endif
