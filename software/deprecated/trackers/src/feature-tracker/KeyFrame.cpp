#include "KeyFrame.hpp"

using namespace tracking;

KeyFrame::
KeyFrame() {
  setId(-1);
  setNumPyramidLevels(4);
  setFastThreshold(5);
  setSmoothingSigma(0);
  setShouldExtractFeatures(true);
}

void KeyFrame::
setId(const int64_t iId) {
  mId = iId;
}

int64_t KeyFrame::
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
setShouldExtractFeatures(const bool iVal) {
  mShouldExtractFeatures = iVal;
}

cv::Mat KeyFrame::
convert(const cv::Mat& iImage) const {
  cv::Mat grayImage, outImage;
  if (iImage.channels() > 1) cv::cvtColor(iImage, grayImage, CV_BGR2GRAY);
  else grayImage = iImage;
  if (mSmoothingSigma > 0.01) {
    cv::GaussianBlur(grayImage, outImage, cv::Size(5,5),
                     mSmoothingSigma, 0, cv::BORDER_REFLECT);
  }
  else outImage = grayImage;

  return outImage;
}

void KeyFrame::
setData(const cv::Mat& iLeft, const cv::Mat& iRight,
        const cv::Mat& iDisparity) {
  // copy original input data
  mOrigData.mLeftImage = iLeft.clone();
  mOrigData.mRightImage = iRight.clone();
  mOrigData.mDisparity = iDisparity.clone();

  // create pyramid structure
  mPyramid.resize(mNumPyramidLevels);
  for (size_t i = 0; i < mPyramid.size(); ++i) {
    mPyramid[i].reset(new PyramidLevel());
  }

  // convert images to grayscale and smooth
  mPyramid[0]->mLeftImage = convert(mOrigData.mLeftImage);
  mPyramid[0]->mRightImage = convert(mOrigData.mRightImage);
  mPyramid[0]->mDisparity = mOrigData.mDisparity;

  // create remaining levels
  for (size_t i = 1; i < mPyramid.size(); ++i) {
    cv::Size size(mPyramid[i-1]->mLeftImage.cols/2,
                  mPyramid[i-1]->mLeftImage.rows/2);
    cv::pyrDown(mPyramid[i-1]->mLeftImage, mPyramid[i]->mLeftImage, size);
    size = cv::Size(mPyramid[i-1]->mRightImage.cols/2,
                    mPyramid[i-1]->mRightImage.rows/2);
    cv::pyrDown(mPyramid[i-1]->mRightImage, mPyramid[i]->mRightImage, size);
    size = cv::Size(mPyramid[i-1]->mDisparity.cols/2,
                    mPyramid[i-1]->mDisparity.rows/2);
    cv::pyrDown(mPyramid[i-1]->mDisparity/2, mPyramid[i]->mDisparity, size);
  }

  // transforms
  mPyramid[0]->mTransformToBase = Eigen::Affine2f::Identity();
  Eigen::Affine2f toPrev = Eigen::Affine2f::Identity();
  toPrev(0,0) = toPrev(1,1) = 2;
  for (size_t i = 1; i < mPyramid.size(); ++i) {
    mPyramid[i]->mTransformToBase = mPyramid[i-1]->mTransformToBase * toPrev;
  }

  // extract features
  if (mShouldExtractFeatures) {
    cv::FastFeatureDetector detector(mFastThreshold);
    for (size_t i = 0; i < mPyramid.size(); ++i) {
      detector.detect(mPyramid[i]->mLeftImage, mPyramid[i]->mLeftKeyPoints);
      // TODO: for now, don't need features from right image
      //detector.detect(mPyramid[i]->mRightImage, mPyramid[i]->mRightKeyPoints);
    }
  }
}

int KeyFrame::
getNumPyramidLevels() const {
  return mPyramid.size();
}

KeyFrame::PyramidLevel::Ptr KeyFrame::
getPyramidLevel(const int iLevel) const {
  return mPyramid[iLevel];
}

void KeyFrame::
setPose(const Eigen::Isometry3f& iPose) {
  mPose = iPose;
}

Eigen::Isometry3f KeyFrame::
getPose() const {
  return mPose;
}
