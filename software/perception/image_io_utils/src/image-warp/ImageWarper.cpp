#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_core/camtrans.h>

#include <lcmtypes/bot_core/image_t.hpp>

#include <opencv2/opencv.hpp>

#include <ConciseArgs>

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

  ImageWarper() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotWrapper.reset(new drc::BotWrapper(mLcm));
    mSubscription = NULL;
  }

  void interpolateGray(const cv::Mat& iImage, const int iIndex,
                       std::vector<uint8_t>& oBytes) {
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

  void interpolateColor(const cv::Mat& iImage, const int iIndex,
                        std::vector<uint8_t>& oBytes) {
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



  void onImage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
               const bot_core::image_t* iMessage) {

    // uncompress or copy
    cv::Mat in;
    int imgType = CV_8UC3;
    switch (iMessage->pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      in = cv::imdecode(cv::Mat(iMessage->data), -1);
      if (in.channels() == 3) cv::cvtColor(in, in, CV_RGB2BGR);
      break;
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      imgType = CV_8UC1;
      in = cv::Mat(iMessage->height, iMessage->width, imgType,
                   (void*)iMessage->data.data(), iMessage->row_stride);
      break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      imgType = CV_8UC3;
      in = cv::Mat(iMessage->height, iMessage->width, imgType,
                   (void*)iMessage->data.data(), iMessage->row_stride);
      break;
    default: std::cout << "error: pixel format not supported\n"; return;
    } 

    // warp
    int rowStride = mOutputWidth*in.channels();
    std::vector<uint8_t> bytes(mOutputHeight*rowStride);
    std::fill(bytes.begin(),bytes.end(),0);
    cv::Mat out(mOutputHeight, mOutputWidth, imgType,
                (void*)bytes.data(), rowStride);
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

    // compress
    bot_core::image_t msg;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, mJpegQuality };
    if (in.channels() == 3) cv::cvtColor(out, out, CV_RGB2BGR);
    if (!cv::imencode(".jpg", out, msg.data, params)) {
      std::cout << "error encoding jpeg image" << std::endl;
    }
    msg.size = msg.data.size();
    
    // transmit
    msg.utime = iMessage->utime;
    msg.width = mOutputWidth;
    msg.height = mOutputHeight;
    msg.row_stride = rowStride;
    msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    msg.nmetadata = 0;
    mLcm->publish(mOutputChannel, &msg);
    std::cout << "re-transmitted image on " << mOutputChannel << std::endl;
  }

  void setJpegQuality(const int iQuality) {
    mJpegQuality = iQuality;
  }

  bool setChannels(const std::string& iInputChannel,
                   const std::string& iOutputChannel) {
    if (mSubscription != NULL) {
      mLcm->unsubscribe(mSubscription);
    }

    mInputChannel = iInputChannel;
    mOutputChannel = iOutputChannel;
    BotCamTrans* inputCamTrans =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 mInputChannel.c_str());
    if (inputCamTrans == NULL) {
      std::cout << "error: cannot find camera " << mInputChannel << std::endl;
      return false;
    }
    BotCamTrans* outputCamTrans =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 mOutputChannel.c_str());
    if (outputCamTrans == NULL) {
      std::cout << "error: cannot find camera " << mOutputChannel << std::endl;
      return false;
    }

    int inputWidth = bot_camtrans_get_width(inputCamTrans);
    int inputHeight = bot_camtrans_get_height(inputCamTrans);
    mOutputWidth = bot_camtrans_get_width(outputCamTrans);
    mOutputHeight = bot_camtrans_get_height(outputCamTrans);

    // precompute warp field
    mWarpFieldIntX.resize(mOutputWidth*mOutputHeight);
    mWarpFieldIntY.resize(mWarpFieldIntX.size());
    mWarpFieldFrac00.resize(mWarpFieldIntX.size());
    mWarpFieldFrac01.resize(mWarpFieldIntX.size());
    mWarpFieldFrac10.resize(mWarpFieldIntX.size());
    mWarpFieldFrac11.resize(mWarpFieldIntX.size());
    Eigen::Isometry3d outputToInput;
    if (!mBotWrapper->getTransform(mOutputChannel, mInputChannel,
                                   outputToInput)) {
      std::cout << "error: cannot get transform from " << mOutputChannel <<
        " to " << mInputChannel << std::endl;
      return false;
    }
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

    mLcm->subscribe(mInputChannel, &ImageWarper::onImage, this);
    return true;
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }
};

int main(const int iArgc, const char** iArgv) {
  int jpegQuality = 90;
  std::string inputChannel, outputChannel;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(inputChannel, "i", "input_channel",
          "incoming camera channel to warp");
  opt.add(outputChannel, "o", "output_channel",
          "camera for output warp and publish");
  opt.add(jpegQuality, "j", "jpeg_quality",
          "jpeg quality (1-100)");
  opt.parse();

  ImageWarper obj;
  obj.setJpegQuality(jpegQuality);
  if (!obj.setChannels(inputChannel, outputChannel)) {
    return -1;
  }
  obj.start();
  return 1;
}
