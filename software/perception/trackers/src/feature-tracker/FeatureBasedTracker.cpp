#include "FeatureBasedTracker.hpp"

#include <unordered_map>
#include "StereoCamera.hpp"
#include "KeyFrame.hpp"
#include "PointMatcher.hpp"

using namespace tracking;

struct FeatureBasedTracker::Helper {
  int mPatchRadiusX;
  int mPatchRadiusY;
  float mMinMatchScore;

  StereoCamera mCamera;

  typedef std::unordered_map<int,TrackedObject::Ptr> ObjectMap;
  ObjectMap mTrackedObjects;

  typedef std::unordered_map<int64_t,KeyFrame::Ptr> KeyFrameMap;
  KeyFrameMap mKeyFrames;


  /* TODO: idea:
     solve for 3d point X by minimizing patch differences in left and right
     patch_orig_left - patch_pred_left and either
        1. patch_orig_right - patch_pred_right, or
        2. patch_pred_left - patch_pred_right
     warp original patches to current viewpoint
     optimize sum of two error functions
     at each iteration, reproject 3d point and compute new errors
   */

  bool update(TrackedObject::Ptr& iObject, const KeyFrame::Ptr& iKeyFrame) {
    StereoCamera camera = mCamera;
    camera.applyPose(iKeyFrame->getPose());
    const CameraModel& leftCamera = camera.getLeftCamera();
    const CameraModel& rightCamera = camera.getRightCamera();
    for (size_t i = 0; i < iObject->mLandmarks.size(); ++i) {
      // project model point into images
      Eigen::Vector2f predLeft =
        leftCamera.pointToPixel(iObject->mLandmarks[i].mPos3d).head<2>();
      Eigen::Vector2f predRight =
        rightCamera.pointToPixel(iObject->mLandmarks[i].mPos3d).head<2>();

      // predict patches
      
      // TODO

      // relocalize predicted point in left image

      // find corresponding point in right image

      // intersect rays
    }

    // align subset of 3d points to original 3d points using ransac

    // update current track state


    return true;
  }
};


FeatureBasedTracker::
FeatureBasedTracker() {
  mHelper.reset(new Helper());
  setPatchRadius(7,7);
  setMinMatchScore(0.8);
}

void FeatureBasedTracker::
clear() {
  mHelper->mTrackedObjects.clear();
  mHelper->mKeyFrames.clear();
}

void FeatureBasedTracker::
setCamera(const StereoCamera& iCamera) {
  mHelper->mCamera = iCamera;
}

void FeatureBasedTracker::
setPatchRadius(const int iRadiusX, const int iRadiusY) {
  mHelper->mPatchRadiusX = iRadiusX;
  mHelper->mPatchRadiusY = iRadiusY;
}

void FeatureBasedTracker::
setMinMatchScore(const float iScore) {
  mHelper->mMinMatchScore = iScore;
}

bool FeatureBasedTracker::
initialize(const int64_t iTime, const int iId, const cv::Mat& iMask,
           const cv::Mat& iLeftImage, const cv::Mat& iRightImage,
           const cv::Mat& iDisparity, const Eigen::Isometry3f& iObjectPose,
           const Eigen::Isometry3f& iSensorPose) {

  // set up frame info
  KeyFrame::Ptr keyFrame(new KeyFrame());
  keyFrame->setId(iTime);
  keyFrame->setNumPyramidLevels(4);  // TODO: make this a param
  keyFrame->setFastThreshold(5);  // TODO: make this a param
  keyFrame->setSmoothingSigma(0.5);  // TODO: make this a param
  keyFrame->setShouldExtractFeatures(true);
  keyFrame->setPose(iSensorPose);
  keyFrame->setData(iLeftImage, iRightImage, iDisparity);

  // set up temporary pyramids
  std::vector<cv::Mat> leftPyramid(keyFrame->getNumPyramidLevels());
  std::vector<cv::Mat> rightPyramid(keyFrame->getNumPyramidLevels());
  for (int i = 0; i < keyFrame->getNumPyramidLevels(); ++i) {
    leftPyramid[i] = keyFrame->getPyramidLevel(i)->mLeftImage;
    rightPyramid[i] = keyFrame->getPyramidLevel(i)->mRightImage;
  }

  // mask out features and find matches across stereo pair
  // TODO: for now, just base pyramid level
  KeyFrame::PyramidLevel::Ptr pyrLevel = keyFrame->getPyramidLevel(0);
  std::vector<cv::KeyPoint>& keyPoints = pyrLevel->mLeftKeyPoints;
  PointMatcher matcher;
  Eigen::Affine2f refTransform = Eigen::Affine2f::Identity();
  Eigen::Vector2f xDir = Eigen::Vector2f::UnitX();
  std::vector<PointMatch> matches;
  matches.reserve(keyPoints.size());
  for (size_t i = 0; i < keyPoints.size(); ++i) {
    // check whether point falls inside object mask in left image
    if (iMask.at<uint8_t>(keyPoints[i].pt) == 0) continue;

    // move point into right image via disparity
    Eigen::Vector2f curPos(keyPoints[i].pt.x, keyPoints[i].pt.y);
    curPos[0] -= pyrLevel->mDisparity.at<float>(keyPoints[i].pt);

    // try to find matching point to subpixel accuracy using klt track
    refTransform.translation() = Eigen::Vector2f(keyPoints[i].pt.x,
                                                 keyPoints[i].pt.y);

    PointMatch match = matcher.refine(curPos, rightPyramid, refTransform,
                                      leftPyramid, mHelper->mPatchRadiusX,
                                      mHelper->mPatchRadiusY);
    std::cout << match.mRefPos.transpose() << " " << curPos.transpose() << " " << match.mCurPos.transpose() << " " << match.mScore << std::endl;
    // add to list if good match
    if (match.mScore >= mHelper->mMinMatchScore) {
      matches.push_back(match);
    }
  }
  return true;

  // triangulate 3d points from 2d stereo pairs
  StereoCamera camera = mHelper->mCamera;
  camera.applyPose(iSensorPose);
  const CameraModel& leftCam = mHelper->mCamera.getLeftCamera();
  const CameraModel& rightCam = mHelper->mCamera.getRightCamera();
  const Eigen::Vector3f leftOrigin = leftCam.getPose().translation();
  const Eigen::Vector3f rightOrigin = rightCam.getPose().translation();
  std::vector<Landmark> landmarks(matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    landmarks[i].mId = i;
    landmarks[i].mPyramidLevel = 0;
    landmarks[i].mKeyFrameId = keyFrame->getId();
    landmarks[i].mPosLeft = matches[i].mRefPos;
    landmarks[i].mPosRight = matches[i].mCurPos;
    landmarks[i].mNormal = -iSensorPose.matrix().block<3,1>(0,2);

    Eigen::Vector3f leftRay = leftCam.pixelToRay(matches[i].mRefPos);
    Eigen::Vector3f rightRay = rightCam.pixelToRay(matches[i].mCurPos);
    Eigen::Matrix<float,3,2> lhs;
    Eigen::Vector3f rhs = rightOrigin - leftOrigin;
    lhs.block<3,1>(0,0) = leftRay;
    lhs.block<3,1>(0,0) = -rightRay;
    Eigen::Vector2f sol =
      lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);
    landmarks[i].mPos3d = leftOrigin + leftRay*sol[0];
    // TODO landmarks[i].mPosCov = xxx;
  }

  // find robust mean and stdev
  int n = landmarks.size();
  std::vector<float> xVals(n);
  std::vector<float> yVals(n);
  std::vector<float> zVals(n);
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3f& pt = landmarks[i].mPos3d;
    xVals[i] = pt[0];
    yVals[i] = pt[1];
    zVals[i] = pt[2];
  }
  std::sort(xVals.begin(), xVals.end());
  std::sort(yVals.begin(), yVals.end());
  std::sort(zVals.begin(), zVals.end());
  Eigen::Vector3f medianPoint(xVals[n/2], yVals[n/2], zVals[n/2]);
  std::vector<float> squaredDistances(n);
  for (int i = 0; i < n; ++i) {
    squaredDistances[i] = (landmarks[i].mPos3d - medianPoint).squaredNorm();
  }
  std::vector<float> sortedSquaredDistances = squaredDistances;
  std::sort(sortedSquaredDistances.begin(), sortedSquaredDistances.end());
  float distThresh = 3*1.5*sqrt(squaredDistances[n/2]);
  distThresh *= distThresh;

  TrackedObject::Ptr object(new TrackedObject());
  object->mId = iId;
  TrackedObject::State state;
  state.mTimestamp = iTime;
  state.mPose = iObjectPose;
  object->mOriginalState = state;
  object->mCurrentState = state;
  object->mLandmarks.reserve(landmarks.size());

  // add points that are close enough to center
  for (int i = 0; i < n; ++i) {
    if (squaredDistances[i] <= distThresh) {
      object->mLandmarks.push_back(landmarks[i]);
    }
  }

  // add this object to map
  mHelper->mTrackedObjects[object->mId] = object;

  // add this keyframe to map
  mHelper->mKeyFrames[iTime] = keyFrame;

  return false;
}

bool FeatureBasedTracker::
update(const int64_t iTime,
       const cv::Mat& iLeftImage, const cv::Mat& iRightImage,
       const cv::Mat& iDisparity, const Eigen::Isometry3f& iSensorPose) {

  // set up frame info
  KeyFrame::Ptr keyFrame(new KeyFrame());
  keyFrame->setId(iTime);
  keyFrame->setNumPyramidLevels(4);  // TODO: make this a param
  keyFrame->setSmoothingSigma(0.5);  // TODO: make this a param
  keyFrame->setShouldExtractFeatures(true);
  keyFrame->setPose(iSensorPose);
  keyFrame->setData(iLeftImage, iRightImage, iDisparity);

  Helper::ObjectMap::const_iterator iter = mHelper->mTrackedObjects.begin();
  for (; iter != mHelper->mTrackedObjects.end(); ++iter) {
    TrackedObject::Ptr obj = iter->second;
    mHelper->update(obj, keyFrame);
  }

  return true;
}

TrackedObject::State FeatureBasedTracker::
getCurrentState(const int iId) const {
  Helper::ObjectMap::const_iterator item = mHelper->mTrackedObjects.find(iId);
  if (item == mHelper->mTrackedObjects.end()) {
    TrackedObject::State state;
    state.mTimestamp = -1;
    state.mPose = Eigen::Isometry3f::Identity();
    return state;
  }
  return item->second->mCurrentState;
}
