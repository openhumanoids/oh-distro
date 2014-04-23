
#include "ImageWarper.hpp"
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_core/camtrans.h>

#include <lcmtypes/bot_core/image_t.hpp>

#include <eigen3/Eigen/Eigen>




  ImageWarper::ImageWarper(const std::string& iInputChannel, BotCamTrans** pOutputCamTrans) {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotWrapper.reset(new drc::BotWrapper(mLcm));
    mSubscription = NULL;
    setChannels(iInputChannel, pOutputCamTrans);
  }

  void ImageWarper::interpolateGray(const cv::Mat& iImage, const int iIndex,
		               uint8_t* oBytes) {
    int xInt(mWarpFieldIntX[iIndex]), yInt(mWarpFieldIntY[iIndex]);
    if (xInt < 0) return;
    float c00(mWarpFieldFrac00[iIndex]), c01(mWarpFieldFrac01[iIndex]);
    float c10(mWarpFieldFrac10[iIndex]), c11(mWarpFieldFrac11[iIndex]);
    int pos = yInt*iImage.step+xInt;
    uint8_t* data = iImage.data;
    float val00(data[pos]), val10(data[pos+1]);
    float val01(data[pos+iImage.step]), val11(data[pos+iImage.step+1]);
    oBytes[iIndex] = (uint8_t)(c00*val00 + c01*val01 + c10*val10 + c11*val11);
  }

  void ImageWarper::interpolateColor(const cv::Mat& iImage, const int iIndex,
                        uint8_t* oBytes) {
    int xInt(mWarpFieldIntX[iIndex]), yInt(mWarpFieldIntY[iIndex]);
    if (xInt < 0) return;
    float c00(mWarpFieldFrac00[iIndex]), c01(mWarpFieldFrac01[iIndex]);
    float c10(mWarpFieldFrac10[iIndex]), c11(mWarpFieldFrac11[iIndex]);
    uint8_t* data = iImage.data;
    int pos = yInt*iImage.step+xInt*3;
    for (int k = 0; k < 3; ++k, ++pos) {
      float val00(data[pos]), val10(data[pos+3]);
      float val01(data[pos+iImage.step]), val11(data[pos+iImage.step+3]);
      oBytes[iIndex*3+k] =
        (uint8_t)(c00*val00 + c01*val01 + c10*val10 + c11*val11);
    }
  }


  void ImageWarper::unWarp(const cv::Mat& in, cv::Mat& out) {
    int imgType = in.type();

    // warp
    int rowStride = mOutputWidth*in.channels();

    uint8_t* bytes = out.data;
    std::fill(bytes, bytes + mOutputHeight*rowStride, 0);
    if (in.channels() == 1) {
      for (size_t i = 0; i < mWarpFieldIntX.size(); ++i) {
        interpolateGray(in, i, bytes);
      }
    }
    else {
      for (size_t i = 0; i < mWarpFieldIntX.size(); ++i) {
        interpolateColor(in, i, bytes);
      }
    }
  }

  bool ImageWarper::setChannels(const std::string& iInputChannel, BotCamTrans** pOutputCamTrans) {
                     
    mInputChannel = iInputChannel;
    BotCamTrans* inputCamTrans =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 mInputChannel.c_str());
    if (inputCamTrans == NULL) {
      std::cout << "error: cannot find camera " << mInputChannel << std::endl;
      return false;
    }

    BotCamTrans* outputCamTrans = bot_camtrans_new((mInputChannel+"_WARP").c_str(),
    		bot_camtrans_get_width(inputCamTrans), bot_camtrans_get_height(inputCamTrans),
    		bot_camtrans_get_focal_length_x(inputCamTrans), bot_camtrans_get_focal_length_y(inputCamTrans),
    		bot_camtrans_get_principal_x(inputCamTrans), bot_camtrans_get_principal_y(inputCamTrans),
    		bot_camtrans_get_skew(inputCamTrans), bot_null_distortion_create());

    if(pOutputCamTrans != NULL) *pOutputCamTrans = outputCamTrans;

    int inputWidth = bot_camtrans_get_width(inputCamTrans);
    int inputHeight = bot_camtrans_get_height(inputCamTrans);
    mOutputWidth = inputWidth;
    mOutputHeight = inputHeight;

    // precompute warp field
    mWarpFieldIntX.resize(mOutputWidth*mOutputHeight);
    mWarpFieldIntY.resize(mWarpFieldIntX.size());
    mWarpFieldFrac00.resize(mWarpFieldIntX.size());
    mWarpFieldFrac01.resize(mWarpFieldIntX.size());
    mWarpFieldFrac10.resize(mWarpFieldIntX.size());
    mWarpFieldFrac11.resize(mWarpFieldIntX.size());
    Eigen::Isometry3d outputToInput;

    outputToInput.setIdentity();   // no transform
    Eigen::Matrix3d rotation = outputToInput.rotation();

    Eigen::Vector3d ray;
    for (int i = 0, pos = 0; i < mOutputHeight; ++i) {
      for (int j = 0; j < mOutputWidth; ++j, ++pos) {
        mWarpFieldIntX[pos] = -1;
        if (0 != bot_camtrans_unproject_pixel(outputCamTrans, j, i,
                                              ray.data())) {
          continue;
        }
        ray = rotation*ray;
        double pix[3];
        if (0 != bot_camtrans_project_point(inputCamTrans, ray.data(), pix)) {
          continue;
        }
        if ((pix[2] < 0) ||
            (pix[0] < 0) || (pix[0] >= inputWidth-1) ||
            (pix[1] < 0) || (pix[1] >= inputHeight-1)) {
          continue;
        }
        mWarpFieldIntX[pos] = (int)pix[0];
        mWarpFieldIntY[pos] = (int)pix[1];
        double fracX = pix[0] - mWarpFieldIntX[pos];
        double fracY = pix[1] - mWarpFieldIntY[pos];
        mWarpFieldFrac00[pos] = (1-fracX)*(1-fracY);
        mWarpFieldFrac01[pos] = (1-fracX)*fracY;
        mWarpFieldFrac10[pos] = fracX*(1-fracY);
        mWarpFieldFrac11[pos] = fracX*fracY;
      }
    }

    return true;
  }



