#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <opencv2/opencv.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <drc_utils/Clock.hpp>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/bot_core/raw_t.hpp>

extern "C" {
#include <x264.h>
#include <libswscale/swscale.h>
}

struct State {
  drc::LcmWrapper::Ptr mLcmWrapper;
  drc::BotWrapper::Ptr mBotWrapper;

  int mScaleFactor;
  int mBitRate;
  double mKeyframeInterval;
  double mPublishPeriod;
  std::string mCameraChannel;
  std::string mVideoChannel;

  x264_param_t mParams;
  x264_t* mEncoder;
  x264_picture_t mPictureIn;
  x264_picture_t mPictureOut;
  struct SwsContext* mConverterContext;
  int mCurrentFrameNum;

  std::deque<int> mPublishBytes;
  std::deque<int64_t> mPublishTimes;
  int mTimeWindowSize;

  bool setup() {
    auto lcm = mLcmWrapper->get();

    std::string keyBase = "cameras." + mCameraChannel + ".intrinsic_cal";
    if (!mBotWrapper->hasKey(keyBase)) {
      keyBase = "cameras." + mCameraChannel + "_LEFT.intrinsic_cal";
      if (!mBotWrapper->hasKey(keyBase)) {
        std::cout << "error: no such camera " << mCameraChannel << std::endl;
        return false;
      }
      lcm->subscribe(mCameraChannel, &State::onCameras, this);
    }
    else {
      lcm->subscribe(mCameraChannel, &State::onCamera, this);
    }
    int nativeWidth = mBotWrapper->getInt(keyBase + ".width");
    int nativeHeight = mBotWrapper->getInt(keyBase + ".height");

    // x264 params
    int width = nativeWidth/mScaleFactor;
    int height = nativeHeight/mScaleFactor;
    x264_param_default_preset(&mParams, "veryfast", "zerolatency");
    mParams.i_threads = 1;
    mParams.i_width = width;
    mParams.i_height = height;
    mParams.i_fps_num = 1000/mPublishPeriod;
    mParams.i_fps_den = 1000;
    mParams.i_nal_hrd = X264_NAL_HRD_CBR;

    // spacing between iframes and bframes
    mParams.i_keyint_max = mKeyframeInterval/mPublishPeriod;
    mParams.b_intra_refresh = 1;

    // rate control parameters
    mParams.rc.i_rc_method = X264_RC_ABR;
    mParams.rc.i_bitrate = mBitRate;
    mParams.rc.i_vbv_max_bitrate = mBitRate;
    mParams.rc.i_vbv_buffer_size = mBitRate;

    // settings for streaming
    mParams.b_repeat_headers = 1;
    mParams.b_annexb = 1;
    x264_param_apply_profile(&mParams, "baseline");

    // set up encoder
    mEncoder = x264_encoder_open(&mParams);
    if (mEncoder == NULL) {
      std::cout << "error: cannot open h264 encoder" << std::endl;
      return false;
    }

    x264_picture_alloc(&mPictureIn, X264_CSP_I420, width, height);

    // set up converter
    mConverterContext = sws_getContext(width, height, PIX_FMT_RGB24,
                                       width, height, PIX_FMT_YUV420P,
                                       SWS_FAST_BILINEAR, NULL, NULL, NULL);
    if (mConverterContext == NULL) {
      std::cout << "error: cannot create swscaler" << std::endl;
      return false;
    }

    mTimeWindowSize = 5.0/mPublishPeriod;
    mCurrentFrameNum = 0;

    return true;
  }


  void onCamera(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const bot_core::image_t* iMessage) {
    if (mPublishTimes.size() > 0) {
      int64_t dt = iMessage->utime - mPublishTimes.back();
      if ((dt > 0) && (dt < mPublishPeriod*1e6)) return;
    }

    // convert image to rgb
    const auto& img = *iMessage;
    int w = img.width;
    int h = img.height;
    cv::Mat cvImg;
    switch (img.pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      cvImg = cv::imdecode(cv::Mat(img.data), -1); break;
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      cvImg = cv::Mat(h, w, CV_8UC1, (void*)img.data.data()); break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      cvImg = cv::Mat(h, w, CV_8UC3, (void*)img.data.data()); break;
    default:
      std::cout << "error: unknown pixel format" << std::endl; break;
    }
    if (cvImg.channels() == 3) cv::cvtColor(cvImg, cvImg, CV_BGR2RGB);

    // scale down image
    cv::resize(cvImg,cvImg,cv::Size(),1.0f/mScaleFactor,1.0f/mScaleFactor,
               cv::INTER_AREA);

    // convert to yuv420
    w = cvImg.cols;
    h = cvImg.rows;
    int stride = w*3;
    sws_scale(mConverterContext, &cvImg.data, &stride, 0, h,
              mPictureIn.img.plane, mPictureIn.img.i_stride);
    mPictureIn.i_pts = mCurrentFrameNum;
    ++mCurrentFrameNum;

    // encode
    x264_nal_t* nals;
    int numNals;
    int numBytes = x264_encoder_encode(mEncoder, &nals, &numNals,
                                       &mPictureIn, &mPictureOut);

    // push payload bytes to message
    bot_core::raw_t rawMsg;
    rawMsg.utime = drc::Clock::instance()->getCurrentTime();
    rawMsg.data.resize(numBytes);
    int curIndex = 0;
    for (int i = 0; i < numNals; ++i) {
      std::copy(nals[i].p_payload, nals[i].p_payload + nals[i].i_payload,
                rawMsg.data.data() + curIndex);
      curIndex += nals[i].i_payload;
    }
    rawMsg.length = rawMsg.data.size();

    // publish bytes
    mLcmWrapper->get()->publish(mVideoChannel, &rawMsg);

    // compute bandwidth
    mPublishTimes.push_back(iMessage->utime);
    mPublishBytes.push_back(rawMsg.length);
    while ((int)mPublishTimes.size() > mTimeWindowSize) {
      mPublishTimes.pop_front();
      mPublishBytes.pop_front();
    }
    double dt = (mPublishTimes.back() - mPublishTimes.front())*1e-6;
    double bytes =
      std::accumulate(mPublishBytes.begin(), mPublishBytes.end(), 0);
    double kbps = bytes*8/dt/1024;
    std::cout << "cur bytes: " << rawMsg.length << ", bandwidth: " <<
      kbps << " kbps" << std::endl;
  }


  void onCameras(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const bot_core::images_t* iMessage) {
    for (int i = 0; i < iMessage->n_images; ++i) {
      const bot_core::image_t& img = iMessage->images[i];
      cv::Mat cvImg;
      auto imgType = iMessage->image_types[i];
      if (imgType == bot_core::images_t::LEFT) {
        onCamera(iBuf, iChannel + "_LEFT", &img);
        break;
      }
    }
  }
  
};

int main(const int iArgc, const char** iArgv) {

  int scaleFactor = 8;
  int bitRate = 4;
  double publishFrequency = 2;
  double keyframeInterval = 5;
  std::string cameraChannel = "CAMERA";
  std::string videoChannel = "CAMERA_STREAM";

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(scaleFactor, "s", "scale-factor", "downsample factor");
  opt.add(publishFrequency, "f", "publish-frequency", "publish frequency (Hz)");
  opt.add(bitRate, "b", "bitrate", "target bitrate (kbps)");
  opt.add(keyframeInterval, "k", "keyframe-interval",
          "max keyframe interval (s)");
  opt.add(cameraChannel, "c", "camera-channel", "camera channel");
  opt.add(videoChannel, "o", "output-channel", "output video channel");
  opt.parse();

  // create lcm wrapper
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  drc::LcmWrapper::Ptr lcmWrapper(new drc::LcmWrapper(lcm));

  // create bot wrapper
  drc::BotWrapper::Ptr botWrapper(new drc::BotWrapper(lcm));

  // set up state
  std::shared_ptr<State> state(new State());
  state->mLcmWrapper = lcmWrapper;
  state->mBotWrapper = botWrapper;
  state->mCameraChannel = cameraChannel;
  state->mVideoChannel = videoChannel;
  state->mScaleFactor = scaleFactor;
  state->mBitRate = bitRate;
  state->mKeyframeInterval = keyframeInterval;
  state->mPublishPeriod = 1/publishFrequency;

  if (!state->setup()) {
    return -1;
  }

  // run
  lcmWrapper->startHandleThread(true);

  // clean up
  return 0;
}
