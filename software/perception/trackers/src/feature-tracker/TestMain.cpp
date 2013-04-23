#include <iostream>
#include <fstream>
#include <chrono>
#include <random>

#include <bot_param/param_client.h>
#include <opencv2/opencv.hpp>

#include "StereoCamera.hpp"
#include "BotUtils.hpp"
#include "FeatureBasedTracker.hpp"
#include "KeyFrame.hpp"
#include "PatchUtils.hpp"
#include "PointMatcher.hpp"
#include "PoseEstimator.hpp"

using namespace std;
using namespace tracking;

int main(const int iArgc, const char** iArgv) {

  //
  // test initialization of stereo camera
  //
  StereoCamera camera;
  char* homeDir = getenv("HOME");
  char path[256];
  sprintf(path, "%s/drc/software/config/drc_robot.cfg", homeDir);
  BotParam* botParam = bot_param_new_from_file(path);
  if (botParam == NULL) {
    std::cerr << "Could not get botparam!" << std::endl;
    return -1;
  }
  if (!BotUtils::configure(botParam, camera)) {
    std::cerr << "Could not configure camera" << std::endl;
    return -1;
  }
  std::cout << "Configured stereo camera" << std::endl;
  if (false) {
    std::cout << "LEFT\n" << camera.getLeftCamera().getCalibMatrix() << "\n";
    std::cout << "RIGHT\n" << camera.getRightCamera().getCalibMatrix() << "\n";
    std::cout << "LEFT\n" << camera.getLeftCamera().getPose().matrix() << "\n";
    std::cout << "RIGHT\n" << camera.getRightCamera().getPose().matrix() << "\n";
  }
  bot_param_destroy(botParam);


  //
  // test keyframe
  //
  cv::Mat leftImage = cv::imread(std::string(homeDir) + "/left_orig.png");
  cv::Mat rightImage = cv::imread(std::string(homeDir) + "/right_orig.png");
  cv::Mat disparity = cv::imread(std::string(homeDir) + "/disparity.pgm",
                                 CV_LOAD_IMAGE_UNCHANGED);
  cv::Mat mask = cv::imread(std::string(homeDir) + "/mask.png",
                            CV_LOAD_IMAGE_UNCHANGED);
  disparity.convertTo(disparity, CV_32F);
  disparity /= 16;
  int64_t timestamp;
  int id;
  {
    std::ifstream ifs(std::string(homeDir) + "/metadata.txt");
    ifs >> id >> timestamp;
  }
  Eigen::Isometry3f sensorPose;
  {
    Eigen::Isometry3f pose;
    std::ifstream ifs(std::string(homeDir) + "/sensor_pose.txt");
    ifs >> pose(0,0) >> pose(0,1) >> pose(0,2) >> pose(0,3);
    ifs >> pose(1,0) >> pose(1,1) >> pose(1,2) >> pose(1,3);
    ifs >> pose(2,0) >> pose(2,1) >> pose(2,2) >> pose(2,3);
    ifs >> pose(3,0) >> pose(3,1) >> pose(3,2) >> pose(3,3);
    sensorPose = pose;
  }
  Eigen::Isometry3f objectPose;
  {
    Eigen::Isometry3f pose;
    std::ifstream ifs(std::string(homeDir) + "/object_pose.txt");
    ifs >> pose(0,0) >> pose(0,1) >> pose(0,2) >> pose(0,3);
    ifs >> pose(1,0) >> pose(1,1) >> pose(1,2) >> pose(1,3);
    ifs >> pose(2,0) >> pose(2,1) >> pose(2,2) >> pose(2,3);
    ifs >> pose(3,0) >> pose(3,1) >> pose(3,2) >> pose(3,3);
    objectPose = pose;
  }
  KeyFrame::Ptr keyFrame(new KeyFrame());
  keyFrame->setId(timestamp);
  keyFrame->setNumPyramidLevels(4);
  keyFrame->setFastThreshold(5);
  keyFrame->setSmoothingSigma(0.5);
  keyFrame->setShouldExtractFeatures(true);
  keyFrame->setPose(sensorPose);
  auto startTime = std::chrono::high_resolution_clock::now();
  keyFrame->setData(leftImage, rightImage, disparity);
  auto endTime = std::chrono::high_resolution_clock::now();
  auto timeDiff =
    std::chrono::duration_cast<std::chrono::microseconds>(endTime-startTime);
  std::cout << "Time " << timeDiff.count()/1e6 << std::endl;
  cv::imwrite(std::string(homeDir) + "/mask_out.pgm", mask);
  for (int i = 0; i < keyFrame->getNumPyramidLevels(); ++i) {
    KeyFrame::PyramidLevel::Ptr level = keyFrame->getPyramidLevel(i);
    cv::imwrite(std::string(homeDir) + "/left_level_" + char(i+'0') + ".png",
                level->mLeftImage);
    cv::imwrite(std::string(homeDir) + "/right_level_" + char(i+'0') + ".png",
                level->mRightImage);
    cv::Mat foo = level->mDisparity*16;
    foo.convertTo(foo, CV_16U);
    cv::imwrite(std::string(homeDir) + "/disp_level_" + char(i+'0') + ".pgm",
                foo);
    ofstream ofs(std::string(homeDir) + "/left_fast_" + char(i+'0') + ".txt");
    for (size_t k = 0; k < level->mLeftKeyPoints.size(); ++k) {
      ofs << level->mLeftKeyPoints[k].pt.x << " " << level->mLeftKeyPoints[k].pt.y << std::endl;
    }
  }

  //
  // test patch tracking
  //
  cv::Mat rightImageFake;
  cv::Mat_<float> warper(2,3);
  warper(0,0) = 1; warper(0,1) = 0; warper(0,2) = 3;
  warper(1,0) = 0; warper(1,1) = 1; warper(1,2) = 0;
  cv::warpAffine(leftImage, rightImageFake, warper, leftImage.size());
  KeyFrame::Ptr keyFrame1(new KeyFrame());
  keyFrame1->setId(0);
  keyFrame1->setNumPyramidLevels(4);
  keyFrame1->setFastThreshold(5);
  keyFrame1->setSmoothingSigma(0.5);
  keyFrame1->setShouldExtractFeatures(true);
  keyFrame1->setPose(sensorPose);
  keyFrame1->setData(leftImage, leftImage, disparity);
  std::vector<cv::Mat> leftPyramid(keyFrame1->getNumPyramidLevels());
  std::vector<cv::Mat> rightPyramid(keyFrame1->getNumPyramidLevels());
  for (int i = 0; i < keyFrame1->getNumPyramidLevels(); ++i) {
    leftPyramid[i] = keyFrame1->getPyramidLevel(i)->mLeftImage;
    rightPyramid[i] = keyFrame1->getPyramidLevel(i)->mRightImage;
  }
  KeyFrame::PyramidLevel::Ptr pyrLevel = keyFrame1->getPyramidLevel(0);
  PointMatcher matcher;
  Eigen::Affine2f refTransform = Eigen::Affine2f::Identity();
  Eigen::Vector2f xDir = Eigen::Vector2f::UnitX();
  Eigen::Vector2f curPos(408,376);
  //Eigen::Vector2f curPos(797,10);
  refTransform.translation() = curPos + Eigen::Vector2f(10,5);
  PointMatch match = matcher.refine(curPos, rightPyramid, refTransform,
                                    leftPyramid, 20,20);
  std::cout << match.mRefPos.transpose() << " " <<
    curPos.transpose() << " " << match.mCurPos.transpose() << " " <<
    match.mScore << std::endl;

  //
  // test 3d pose estimation
  //
  std::vector<Eigen::Vector3f> ptsOrig(100);
  std::default_random_engine gen;
  std::uniform_real_distribution<float> dist(-10,10);
  for (size_t i = 0; i < ptsOrig.size(); ++i) {
    ptsOrig[i] = Eigen::Vector3f(dist(gen), dist(gen), dist(gen));
  }
  Eigen::Isometry3f xform = Eigen::Isometry3f::Identity();
  xform.translation() = Eigen::Vector3f(1,2,3);
  Eigen::Matrix3f rot;
  rot = Eigen::AngleAxisf(0.1, Eigen::Vector3f(1,2,3).normalized());
  xform.linear() = rot;
  auto ptsNew = ptsOrig;
  for (size_t i = 0; i < ptsNew.size(); ++i) {
    ptsNew[i] = xform*ptsOrig[i];
  }
  PoseEstimator est;
  Eigen::Isometry3f estPose;
  if (est.leastSquares(ptsOrig, ptsNew, estPose)) {
    std::cout << "est pose\n" << estPose.matrix() << std::endl;
  } else {
    std::cout << "POSE ESTIMATION FAILED" << std::endl;
  }
  std::vector<int> inliers;
  for (int i = 0; i < 10; ++i) {
    ptsOrig.push_back(Eigen::Vector3f(dist(gen), dist(gen), dist(gen)));
    ptsNew.push_back(Eigen::Vector3f(dist(gen), dist(gen), dist(gen)));
  }
  est.setErrorThreshold(0.01);
  if (est.ransac(ptsOrig, ptsNew, estPose, inliers)) {
    std::cout << "ransac pose\n" << estPose.matrix() << std::endl;
    std::cout << "inliers: " << inliers.size() << std::endl;
  } else {
    std::cout << "POSE RANSAC FAILED" << std::endl;
  }

  //
  // test object initialization
  //
  startTime = std::chrono::high_resolution_clock::now();
  FeatureBasedTracker tracker;
  tracker.setCamera(camera);
  tracker.initialize(timestamp, id, mask, leftImage, rightImage, disparity,
                     objectPose, sensorPose);
  endTime = std::chrono::high_resolution_clock::now();
  timeDiff =
    std::chrono::duration_cast<std::chrono::microseconds>(endTime-startTime);
  std::cout << "Time for init " << timeDiff.count()/1e6 << std::endl;

  //
  // test object tracking
  //
  startTime = std::chrono::high_resolution_clock::now();
  tracker.update(timestamp, leftImage, rightImage, disparity, sensorPose);
  endTime = std::chrono::high_resolution_clock::now();
  timeDiff =
    std::chrono::duration_cast<std::chrono::microseconds>(endTime-startTime);
  std::cout << "Time for update " << timeDiff.count()/1e6 << std::endl;

  return 0;
}
