#include "FeatureBasedTracker.hpp"

#include <unordered_map>
#include "StereoCamera.hpp"
#include "KeyFrame.hpp"
#include "PointMatcher.hpp"
#include "PatchUtils.hpp"
#include "PoseEstimator.hpp"

using namespace tracking;

struct FeatureBasedTracker::Helper {
  int mPatchRadiusX;
  int mPatchRadiusY;
  float mMinMatchScore;

  StereoCamera mCamera;
  KeyFrame::Ptr mCurrentKeyFrame;

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

     TODO: idea:
     track entire object's dofs by minimizing all patch errors in both images
     can also update structure by allowing some deformations
   */

  bool update(TrackedObject::Ptr& iObject) {
    // set up cameras for current frame
    KeyFrame::Ptr keyFrame = mCurrentKeyFrame;
    StereoCamera camera = mCamera;
    camera.applyPose(keyFrame->getPose());
    const CameraModel& leftCamera = camera.getLeftCamera();
    const CameraModel& rightCamera = camera.getRightCamera();
    const Eigen::Vector3f leftOrigin = leftCamera.getPose().translation();
    const Eigen::Vector3f rightOrigin = rightCamera.getPose().translation();

    PointMatcher matcher;

    // set up image pyramids for current frame
    std::vector<cv::Mat> leftPyramid(keyFrame->getNumPyramidLevels());
    std::vector<cv::Mat> rightPyramid(keyFrame->getNumPyramidLevels());
    for (int i = 0; i < keyFrame->getNumPyramidLevels(); ++i) {
      leftPyramid[i] = keyFrame->getPyramidLevel(i)->mLeftImage;
      rightPyramid[i] = keyFrame->getPyramidLevel(i)->mRightImage;
    }

    // try to find landmarks in current frame
    std::vector<Eigen::Vector3f> pointsRef, pointsCur;
    for (size_t i = 0; i < iObject->mLandmarks.size(); ++i) {
      Landmark& landmark = iObject->mLandmarks[i];

      // adjust model point by delta in object pose
      Eigen::Isometry3f origToCurrent =
        iObject->mCurrentState.mPose.inverse()*iObject->mOriginalState.mPose;
      Eigen::Vector3f pos3d = origToCurrent*landmark.mPos3d;
      Eigen::Vector3f normal = origToCurrent.linear()*landmark.mNormal;

      // project model point into images
      Eigen::Vector2f predLeft, predRight;
      predLeft = leftCamera.pointToPixel(pos3d).head<2>();
      predRight = rightCamera.pointToPixel(pos3d).head<2>();

      KeyFrame::Ptr keyFrameOrig = mKeyFrames[landmark.mKeyFrameId];

      // set up pyramid from original frame
      std::vector<cv::Mat> leftPyramidOrig(leftPyramid.size());
      for (size_t k = 0; k < leftPyramidOrig.size(); ++k) {
        leftPyramidOrig[k] = keyFrameOrig->getPyramidLevel(k)->mLeftImage;
      }

      // predict and locate patch in left image via current sensor pose
      Eigen::Vector4f plane;
      plane.head<3>() = normal;
      plane[3] = -plane.head<3>().dot(pos3d);
      StereoCamera cameraOrig = mCamera;
      cameraOrig.applyPose(keyFrameOrig->getPose());
      const CameraModel& leftCameraOrig = cameraOrig.getLeftCamera();
      Eigen::Affine2f patchTransform =
        PatchUtils::linearize(landmark.mPosLeft, leftCameraOrig,
                              leftCamera, plane);
      PointMatch match = matcher.refine
        (predLeft, leftPyramid, patchTransform, leftPyramidOrig,
         mPatchRadiusX, mPatchRadiusY);
      if (match.mScore < mMinMatchScore) continue;

      // find corresponding point in right image
      // TODO: replace epipolarLine with actual epipolar line
      Eigen::Vector3f epipolarLine(0,1,-match.mCurPos[1]);
      float t = -(epipolarLine[2] + epipolarLine.head<2>().dot(predRight));
      predRight += epipolarLine.head<2>()*t;
      Eigen::Vector2f epiDir(epipolarLine[2], -epipolarLine[1]);
      patchTransform = Eigen::Affine2f::Identity();
      patchTransform.translation() = match.mCurPos;
      match = matcher.refine(predRight, rightPyramid, patchTransform,
                             leftPyramid, mPatchRadiusX, mPatchRadiusY, epiDir);
      if (match.mScore < mMinMatchScore) continue;

      // intersect rays
      Eigen::Vector3f leftRay = leftCamera.pixelToRay(match.mRefPos);
      Eigen::Vector3f rightRay = rightCamera.pixelToRay(match.mCurPos);
      Eigen::Matrix<float,3,2> lhs;
      Eigen::Vector3f rhs = rightOrigin - leftOrigin;
      lhs.block<3,1>(0,0) = leftRay;
      lhs.block<3,1>(0,1) = -rightRay;
      Eigen::Vector2f sol =
        lhs.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(rhs);
      Eigen::Vector3f pos3dCur = leftOrigin + leftRay*sol[0];

      pointsRef.push_back(pos3d);
      pointsCur.push_back(pos3dCur);
    }

    // align subset of 3d points to original 3d points using ransac
    PoseEstimator estimator;
    estimator.setErrorThreshold(0.03);
    Eigen::Isometry3f poseChange;
    std::vector<int> inliers;
    if (!estimator.ransac(pointsRef, pointsCur, poseChange, inliers)) {
      return false;
    }

    std::cout << "Pose change\n" << poseChange.matrix() << std::endl;
    std::cout << "Inliers " << inliers.size() << std::endl;

    // update current track state
    iObject->mCurrentState.mTimestamp = keyFrame->getId();
    iObject->mCurrentState.mPose = iObject->mCurrentState.mPose*poseChange;

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

void FeatureBasedTracker::
setData(const int64_t iTime, const cv::Mat& iLeftImage,
        const cv::Mat& iRightImage, const cv::Mat& iDisparity,
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
  mHelper->mCurrentKeyFrame = keyFrame;
}



bool FeatureBasedTracker::
initialize(const int iId, const cv::Mat& iMask,
           const Eigen::Isometry3f& iObjectPose) {
  KeyFrame::Ptr keyFrame = mHelper->mCurrentKeyFrame;

  // set up temporary pyramids
  std::vector<cv::Mat> leftPyramid(keyFrame->getNumPyramidLevels());
  std::vector<cv::Mat> rightPyramid(keyFrame->getNumPyramidLevels());
  for (int i = 0; i < keyFrame->getNumPyramidLevels(); ++i) {
    leftPyramid[i] = keyFrame->getPyramidLevel(i)->mLeftImage;
    rightPyramid[i] = keyFrame->getPyramidLevel(i)->mRightImage;
  }

  // mask features and find matches across stereo pair
  // TODO: for now, just base pyramid level; later use all
  KeyFrame::PyramidLevel::Ptr pyrLevel = keyFrame->getPyramidLevel(0);
  std::vector<cv::KeyPoint>& keyPoints = pyrLevel->mLeftKeyPoints;
  PointMatcher matcher;
  Eigen::Affine2f refTransform = Eigen::Affine2f::Identity();
  Eigen::Vector2f epiDir = Eigen::Vector2f::UnitX();
  StereoCamera camera = mHelper->mCamera;
  camera.applyPose(keyFrame->getPose());
  const CameraModel& leftCam = camera.getLeftCamera();
  const CameraModel& rightCam = camera.getRightCamera();
  const Eigen::Vector3f leftOrigin = leftCam.getPose().translation();
  const Eigen::Vector3f rightOrigin = rightCam.getPose().translation();
  std::vector<Landmark> landmarks;
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
                                      mHelper->mPatchRadiusY, epiDir);
    if (match.mScore < mHelper->mMinMatchScore) continue;

    // create new landmark
    Landmark landmark;
    landmark.mId = i;
    landmark.mPyramidLevel = 0;
    landmark.mKeyFrameId = keyFrame->getId();
    landmark.mPosLeft = match.mRefPos;
    landmark.mPosRight = match.mCurPos;
    landmark.mNormal = leftCam.getPose().matrix().block<3,1>(0,2);

    // triangulate 3d landmark position
    Eigen::Vector3f leftRay = leftCam.pixelToRay(match.mRefPos);
    Eigen::Vector3f rightRay = rightCam.pixelToRay(match.mCurPos);
    Eigen::Matrix<float,3,2> lhs;
    Eigen::Vector3f rhs = rightOrigin - leftOrigin;
    lhs.block<3,1>(0,0) = leftRay;
    lhs.block<3,1>(0,1) = -rightRay;
    Eigen::Vector2f sol =
      lhs.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(rhs);
    landmark.mPos3d = leftOrigin + leftRay*sol[0];
    // TODO landmark.mPosCov = xxx;

    // add landmark to list
    landmarks.push_back(landmark);
  }

  if (landmarks.size() < 5) {  // TODO: make this a parameter
    return false;
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
  float distThresh = 3*1.5*sqrt(sortedSquaredDistances[n/2]);
  distThresh *= distThresh;

  TrackedObject::Ptr object(new TrackedObject());
  object->mId = iId;
  TrackedObject::State state;
  state.mTimestamp = keyFrame->getId();
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

  // add keyframe to map
  mHelper->mKeyFrames[keyFrame->getId()] = keyFrame;

  // add this object to map
  mHelper->mTrackedObjects[object->mId] = object;

  return false;
}

bool FeatureBasedTracker::
update() {
  Helper::ObjectMap::const_iterator iter = mHelper->mTrackedObjects.begin();
  for (; iter != mHelper->mTrackedObjects.end(); ++iter) {
    TrackedObject::Ptr obj = iter->second;
    mHelper->update(obj);
  }
  return true;
}

std::vector<int> FeatureBasedTracker::
getAllTrackIds() const {
  std::vector<int> ids;
  ids.reserve(mHelper->mTrackedObjects.size());
  Helper::ObjectMap::const_iterator iter = mHelper->mTrackedObjects.begin();
  for (; iter != mHelper->mTrackedObjects.end(); ++iter) {
    ids.push_back(iter->second->mId);
  }
  return ids;
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
