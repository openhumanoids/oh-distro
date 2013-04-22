#include "PatchUtils.hpp"

// TODO: TEMP
#include <fstream>

using namespace tracking;

float PatchUtils::
normalizedCrossCorrelation(const cv::Mat& iA, const cv::Mat& iB) {
  /*
  int numRows = iA.rows;
  int numCols = iA.cols;
  int sumA(0), sumB(0), sumSqA(0), sumSqB(0), sumAB(0);
  for (int i = 0; i < numRows; ++i) {
    const uint8_t* a = iA.ptr(i);
    const uint8_t* b = iB.ptr(i);
    for (int j = 0; j < numCols; ++j, ++a, ++b) {
      sumA += *a;
      sumB += *b;
      sumSqA += (*a * *a);
      sumSqB += (*b * *b);
      sumAB += (*a * *b);
    }
  }
  */

  float n = iA.rows*iA.cols;

  float sumA = cv::sum(iA)[0];
  float sumB = cv::sum(iB)[0];
  float sumSqA = iA.dot(iA);
  float sumSqB = iB.dot(iB);
  float sumAB = iA.dot(iB);

  float numer = n*sumAB - sumA*sumB;
  float term1 = n*sumSqA - sumA*sumA;
  float term2 = n*sumSqB - sumB*sumB;
  float denom = sqrt(term1*term2);

  return numer/denom;
}

template<typename InType, typename OutType>
static bool
interpolateImpl(const cv::Mat& iImage, const Eigen::Vector2f& iCenter,
                cv::Mat& oPatch) {
  oPatch = cv::Scalar::all(0);
  int xRadius = (oPatch.cols-1)/2;
  int yRadius = (oPatch.rows-1)/2;
  int xInt(iCenter[0]), yInt(iCenter[1]);
  if ((xInt < -xRadius) || (xInt >= iImage.cols + xRadius) ||
      (yInt < -yRadius) || (yInt >= iImage.rows + yRadius)) return false;
  float xFrac(iCenter[0] - xInt), yFrac(iCenter[1] - yInt);
  float oneMinusXFrac(1-xFrac), oneMinusYFrac(1-yFrac);
  float a00(oneMinusXFrac*oneMinusYFrac), a10(xFrac*oneMinusYFrac);
  float a01(oneMinusXFrac*yFrac), a11(xFrac*yFrac);
  for (int outRow = 0, inRow = yInt-yRadius;
       outRow < oPatch.rows; ++outRow, ++inRow) {
    if ((inRow < 0) || (inRow >= iImage.rows-1)) continue;
    const InType* inPtr0 = iImage.ptr<InType>(inRow) + xInt-xRadius;
    const InType* inPtr1 = iImage.ptr<InType>(inRow+1) + xInt-xRadius;
    OutType* outPtr = oPatch.ptr<OutType>(outRow);
    for (int outCol = 0, inCol = xInt-xRadius; outCol < oPatch.cols;
         ++outCol, ++inCol, ++inPtr0, ++inPtr1, ++outPtr) {
      if ((inCol < 0) || (inCol >= iImage.cols-1)) continue;
      InType val00 = *inPtr0;
      InType val10 = *(inPtr0+1);
      InType val01 = *inPtr1;
      InType val11 = *(inPtr1+1);
      *outPtr = a00*val00 + a10*val10 + a01*val01 + a11*val11;
    }
  }
  
  return true;
}

template<typename OutType>
static bool
interpolateDispatch(const cv::Mat& iImage, const Eigen::Vector2f& iCenter,
                    cv::Mat& oPatch) {
  switch(iImage.type()) {
  case CV_8UC1:
    return interpolateImpl<uint8_t, OutType>(iImage, iCenter, oPatch);
  case CV_32FC1:
    return interpolateImpl<float, OutType>(iImage, iCenter, oPatch);
  default: return false;
  }
}

bool PatchUtils::
interpolate(const cv::Mat& iImage, const Eigen::Vector2f& iCenter,
            cv::Mat& oPatch) {
  switch(oPatch.type()) {
  case CV_8UC1: return interpolateDispatch<uint8_t>(iImage, iCenter, oPatch);
  case CV_32FC1: return interpolateDispatch<float>(iImage, iCenter, oPatch);
  default: return false;
  }
}

template<typename InType, typename OutType>
static bool
interpolateImpl(const cv::Mat& iImage, const Eigen::Affine2f& iTransform,
                cv::Mat& oPatch) {
  oPatch = cv::Scalar::all(0);
  int xRadius = (oPatch.cols-1)/2;
  int yRadius = (oPatch.rows-1)/2;
  const Eigen::Affine2f& m = iTransform;
  float m00(m(0,0)), m01(m(0,1)), m10(m(1,0)), m11(m(1,1));

  float xRow = -xRadius*m00 - yRadius*m01 + m(0,2);
  float yRow = -xRadius*m10 - yRadius*m11 + m(1,2);
  for (int i = 0; i < oPatch.rows; ++i) {
    float x(xRow), y(yRow);
    OutType* outPtr = oPatch.ptr<OutType>(i);
    for (int j = 0; j < oPatch.cols; ++j, ++outPtr, x+= m00, y += m10) {
      int xInt(x), yInt(y);
      if ((xInt < 0) || (xInt >= iImage.cols-1) ||
          (yInt < 0) || (yInt >= iImage.rows-1)) continue;
      const InType* inPtr0 = iImage.ptr<InType>(yInt) + xInt;
      const InType* inPtr1 = iImage.ptr<InType>(yInt+1) + xInt;
      float xFrac(x-xInt), yFrac(y-yInt);
      float v00 = *inPtr0;
      float v10 = *(inPtr0+1);
      float v01 = *inPtr1;
      float v11 = *(inPtr1+1);
      *outPtr = (OutType)(xFrac*yFrac*(v00 + v11 - v10 - v01) +
                          xFrac*(v10 - v00) + yFrac*(v01 - v00) + v00);
    }
    xRow += m01;
    yRow += m11;
  }
  return true;
}

template<typename OutType>
static bool
interpolateDispatch(const cv::Mat& iImage, const Eigen::Affine2f& iTransform,
                    cv::Mat& oPatch) {
  switch(iImage.type()) {
  case CV_8UC1:
    return interpolateImpl<uint8_t, OutType>(iImage, iTransform, oPatch);
  case CV_32FC1:
    return interpolateImpl<float, OutType>(iImage, iTransform, oPatch);
  default: return false;
  }
}

bool PatchUtils::
interpolate(const cv::Mat& iImage, const Eigen::Affine2f& iTransform,
            cv::Mat& oPatch) {
  switch(oPatch.type()) {
  case CV_8UC1:
    return interpolateDispatch<uint8_t>(iImage, iTransform, oPatch);
  case CV_32FC1:
    return interpolateDispatch<float>(iImage, iTransform, oPatch);
  default: return false;
  }
}

// TODO: input image type does not have to be float
bool PatchUtils::
computeGradients(const cv::Mat& iImage, cv::Mat& oGx, cv::Mat& oGy) {
  if ((iImage.type() != CV_32FC1) ||
      (oGx.type() != CV_32FC1) || (oGy.type() != CV_32FC1)) return false;
  if ((iImage.rows < 2) || (iImage.cols < 2)) return false;

  // x gradient
  for (int i = 0; i < iImage.rows; ++i) {
    const float* inPtr = iImage.ptr<float>(i);
    float* gxPtr = oGx.ptr<float>(i);
    *gxPtr++ = *(inPtr+1) - *inPtr;
    ++inPtr;
    for (int j = 1; j < iImage.cols-1; ++j, ++inPtr, ++gxPtr) {
      *gxPtr = 0.5f * (*(inPtr+1) - *(inPtr-1));
    }
    *gxPtr = *inPtr - *(inPtr-1);
  }

  // y gradient
  float* gyPtr = oGy.ptr<float>(0);
  const float* inPtr0 = iImage.ptr<float>(0);
  const float* inPtr1 = iImage.ptr<float>(1);
  for (int j = 0; j < iImage.cols; ++j, ++inPtr0, ++inPtr1, ++gyPtr) {
    *gyPtr = *inPtr1 - *inPtr0;
  }
  for (int i = 1; i < iImage.rows-1; ++i) {
    inPtr0 = iImage.ptr<float>(i-1);
    inPtr1 = iImage.ptr<float>(i+1);
    gyPtr = oGy.ptr<float>(i);
    for (int j = 0; j < iImage.cols; ++j, ++inPtr0, ++inPtr1, ++gyPtr) {
      *gyPtr = 0.5f * (*inPtr1 - *inPtr0);
    }
  }
  inPtr0 = iImage.ptr<float>(iImage.rows-2);
  inPtr1 = iImage.ptr<float>(iImage.rows-1);
  for (int j = 0; j < iImage.cols; ++j, ++inPtr0, ++inPtr1, ++gyPtr) {
    *gyPtr = *inPtr1 - *inPtr0;
  }

  return true;
}

bool PatchUtils::
save(const cv::Mat& iImage, const std::string& iFileName) {
  std::ofstream ofs(iFileName.c_str());
  ofs << iImage.cols << " " << iImage.rows << std::endl;
  for (int i = 0; i < iImage.rows; ++i) {
    for (int j = 0; j < iImage.cols; ++j) {
      ofs << iImage.at<float>(i,j) << " ";
    }
    ofs << std::endl;
  }
  return true;
}

