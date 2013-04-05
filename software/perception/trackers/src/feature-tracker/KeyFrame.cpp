#include "KeyFrame.hpp"

KeyFrame::
KeyFrame() {
  setId(-1);
  setNumPyramidLevels(4);
  setFastThreshold(5);
  setSmoothingSigma(0);
}

void KeyFrame::
setId(const int iId) {
  mId = iId;
}

int KeyFrame::
getId() const {
  return mId;
}

void KeyFrame::
setNumPyramidLevels(const int iNum) {
  mNumPyramidLevels = iNum;
}

void KeyFrame::
setFastThreshold(const int iThresh) {
  mFastThreshold = iThresh;
}

void KeyFrame::
setSmoothingSigma(const float iSigma) {
  mSmoothingSigma = iSigma;
}

void KeyFrame::
setImage(const cv::Mat& iImage) {
  mOrigImage = iImage.clone();
  mPyramid.resize(mNumPyramidLevels);
  if (mSmoothingSigma > 0.01) {
    cv::GaussianBlur(mOrigImage, mPyramid[0]->mImage, cv::Size(5,5),
                     mSmoothingSigma, 0, cv::BORDER_REFLECT);
  }
  else {
    mPyramid[0]->mImage = mOrigImage;
  }
  for (size_t i = 1; i < mPyramid.size(); ++i) {
    cv::Size size(mPyramid[i-1]->mImage.cols/2, mPyramid[i-1]->mImage.rows/2);
    cv::pyrDown(mPyramid[i-1]->mImage, mPyramid[i]->mImage, size);
  }

  for (size_t i = 0; i < mPyramid.size(); ++i) {
    cv::FAST(mPyramid[i]->mImage, mPyramid[i]->mKeyPoints, mFastThreshold);
  }
}

cv::Mat KeyFrame::
getOrigImage() const {
  return mOrigImage;
}

PyramidLevel::Ptr KeyFrame::
getPyramidLevel(const int iLevel) {
  return mPyramid[iLevel];
}
