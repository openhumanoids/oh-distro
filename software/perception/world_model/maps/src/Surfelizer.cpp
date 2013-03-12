#include "Surfelizer.hpp"

#include "Utils.hpp"

#include <boost/circular_buffer.hpp>

using namespace maps;

struct Surfelizer::Helper {
  struct ScanData {
    std::vector<Eigen::Vector3f> mPoints;
    std::vector<float> mRanges;
    std::vector<bool> mValid;
    Eigen::Isometry3f mPose;
    float mIntraScanAngle;
  };

  int mScanRadius;
  int mPointRadius;
  SizeMethod mSizeMethod;
  float mNominalSize;
  int mDecimation;
  boost::circular_buffer<ScanData> mDataBuffer;

  std::vector<Surfel> addScan(const maps::PointSet& iScan) {
    // cache some basic information
    ScanData data;
    data.mPose = Utils::getPose(*iScan.mCloud);
    data.mRanges.resize(iScan.mCloud->size());
    data.mValid.resize(iScan.mCloud->size());
    data.mPoints.resize(iScan.mCloud->size());
    data.mIntraScanAngle = 0;
    int counter = 0;
    for (int i = 0; i < iScan.mCloud->size(); ++i) {
      data.mPoints[i] = data.mPose*(*(iScan.mCloud))[i].getVector3fMap();
      Eigen::Vector3f ray = data.mPoints[i] - data.mPose.translation();
      data.mRanges[i] = ray.norm();
      data.mValid[i] = (data.mRanges[i] >= iScan.mMinRange) &&
        (data.mRanges[i] <= iScan.mMaxRange);
      if ((i>0) && data.mValid[i] && data.mValid[i-1]) {
        Eigen::Vector3f ray2 = data.mPoints[i-1] - data.mPose.translation();
        data.mIntraScanAngle += acos(ray.normalized().dot(ray2.normalized()));
        ++counter;
      }
    }
    data.mIntraScanAngle /= counter;
    mDataBuffer.push_back(data);

    // check to see whether we have enough scans in the buffer to proceed
    std::vector<Surfel> surfels;
    int r1 = mScanRadius;
    int r2 = mPointRadius;
    int w1 = 2*r1+1;
    int w2 = 2*r2+1;
    if (mDataBuffer.size() < w1) return surfels;

    const ScanData& curScan = mDataBuffer[r1];
    surfels.reserve(curScan.mPoints.size());
    std::vector<Eigen::Vector3f> points;
    points.reserve(w1*w2);

    float interScanAngle =
      acos(mDataBuffer[r1-1].mPose.linear().col(2).dot
           (mDataBuffer[r1+1].mPose.linear().col(2))) / 2;

    // loop over scan points
    int numPoints = curScan.mPoints.size();
    for (int i = r2; i < numPoints - r2; i += mDecimation) {
      if (!curScan.mValid[i]) continue;
      
      // initiate surfel
      Surfel surfel;
      surfel.mCenter = curScan.mPoints[i];

      // approximate surfel orientation
      points.clear();
      for (int j = 0; j < w1; ++j) {
        int kMax = std::min(i+r2, (int)mDataBuffer[j].mValid.size()-1);
        for (int k = i-r2; k <= kMax; ++k) {
          if (mDataBuffer[j].mValid[k]) {
            points.push_back(mDataBuffer[j].mPoints[k]);
          }
        }
      }
      if (points.size() < 3) continue;
      surfel.mOrientation = estimateOrientation(points,curScan.mPose.linear());

      // approximate surfel size
      surfel.mSize = Eigen::Vector2f(mNominalSize, mNominalSize);
      if (mSizeMethod == SizeMethodRange) {
        surfel.mSize *= curScan.mRanges[i];
      }
      else if (mSizeMethod == SizeMethodAngles) {
        float range = curScan.mRanges[i];
        Eigen::Vector3f xPt = curScan.mPose.translation() +
          curScan.mPose.linear().col(0)*range;
        float dist = (xPt-surfel.mCenter).norm();
        surfel.mSize = Eigen::Vector2f(interScanAngle*dist,
                                       curScan.mIntraScanAngle*range);
      }
      else if (mSizeMethod == SizeMethodNeighbors) {
        if (!mDataBuffer[r1+1].mValid[i] || !mDataBuffer[r1-1].mValid[i] ||
            !mDataBuffer[r1].mValid[i-1] || !mDataBuffer[r1].mValid[i+1]) {
          continue;
        }
        float d1 = (mDataBuffer[r1+1].mPoints[i] -
                    mDataBuffer[r1-1].mPoints[i]).norm();
        float d2 = (mDataBuffer[r1].mPoints[i-1] -
                    mDataBuffer[r1].mPoints[i+1]).norm();
        surfel.mSize = Eigen::Vector2f(d1/2,d2/2);
      }

      surfels.push_back(surfel);
    }
    return surfels;
  }

  Eigen::Matrix3f
  estimateOrientation(const std::vector<Eigen::Vector3f>& iPoints,
                      const Eigen::Matrix3f& iScanOrientation) {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    Eigen::Matrix3f meanSq = Eigen::Matrix3f::Zero();
    int n = iPoints.size();
    for (int i = 0; i < n; ++i) {
      mean += iPoints[i];
      meanSq += iPoints[i]*iPoints[i].transpose();
    }
    mean /= n;
    meanSq /= n;
    Eigen::Matrix3f cov = meanSq - mean*mean.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3f> svd;
    svd.compute(cov, Eigen::ComputeFullV);
    Eigen::Matrix3f v = svd.matrixV();

    Eigen::Matrix3f orientation;
    orientation.col(2) = v.col(2);
    orientation.col(1) = v.col(2).cross(iScanOrientation.col(2));
    orientation.col(0) = orientation.col(1).cross(v.col(2));
    for (int k = 0; k < 3; ++k) orientation.col(k).normalize();
    return orientation;
  }

};

Surfelizer::
Surfelizer() {
  mHelper.reset(new Helper());
  setScanRadius(1);
  setPointRadius(1);
  setSizeMethod(SizeMethodAngles);
  setNominalSize(1);
  setDecimation(1);
}

Surfelizer::
~Surfelizer() {
}

void Surfelizer::
setScanRadius(const int iRadius) {
  mHelper->mScanRadius = iRadius;
  mHelper->mDataBuffer.set_capacity(2*iRadius+1);
}

void Surfelizer::
setPointRadius(const int iRadius) {
  mHelper->mPointRadius = iRadius;
}

void Surfelizer::
setSizeMethod(const SizeMethod iMethod) {
  mHelper->mSizeMethod = iMethod;
}

void Surfelizer::
setNominalSize(const float iSize) {
  mHelper->mNominalSize = iSize;
}

void Surfelizer::
setDecimation(const int iDecimation) {
  mHelper->mDecimation = iDecimation;
}

std::vector<Surfelizer::Surfel> Surfelizer::
addScan(const maps::PointSet& iScan) {
  return mHelper->addScan(iScan);
}
