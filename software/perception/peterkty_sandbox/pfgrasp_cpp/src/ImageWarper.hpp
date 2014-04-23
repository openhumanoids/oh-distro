/*
 * ImageWarper.hpp
 *
 *  Created on: Apr 21, 2014
 *      Author: peterkty
 */

#ifndef IMAGEWARPER_HPP_
#define IMAGEWARPER_HPP_

#include <opencv2/opencv.hpp>
#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <bot_core/camtrans.h>

struct ImageWarper {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::string mInputChannel;
  std::string mOutputChannel;
  BotParam* mBotParam;
  lcm::Subscription* mSubscription;
  std::vector<int> mWarpFieldIntX;
  std::vector<int> mWarpFieldIntY;
  std::vector<float> mWarpFieldFrac00;
  std::vector<float> mWarpFieldFrac01;
  std::vector<float> mWarpFieldFrac10;
  std::vector<float> mWarpFieldFrac11;
  int mOutputWidth;
  int mOutputHeight;
  int mJpegQuality;

  ImageWarper(const std::string& iInputChannel, BotCamTrans** pOutputCamTrans = NULL) ;

  void interpolateGray(const cv::Mat& iImage, const int iIndex,
		               uint8_t* oBytes) ;

  void interpolateColor(const cv::Mat& iImage, const int iIndex,
                        uint8_t* oBytes) ;

  void unWarp(const cv::Mat& in, cv::Mat& out) ;

  bool setChannels(const std::string& iInputChannel, BotCamTrans** pOutputCamTrans = NULL) ;

};



#endif /* IMAGEWARPER_HPP_ */
