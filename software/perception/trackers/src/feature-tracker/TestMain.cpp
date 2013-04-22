#include <iostream>
#include <fstream>
#include <chrono>

#include "StereoCamera.hpp"
#include "BotUtils.hpp"
#include "FeatureBasedTracker.hpp"
#include <bot_param/param_client.h>
#include <opencv2/opencv.hpp>
#include "KeyFrame.hpp"

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
  // test object initialization
  //
  FeatureBasedTracker tracker;
  tracker.initialize(timestamp, id, mask, leftImage, rightImage, disparity,
                     objectPose, sensorPose);

  return 0;
}
