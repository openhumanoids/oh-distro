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
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mOutputLcm;

  int mScaleFactor;
  double mPublishPeriod;
  std::string mCameraChannel;
  std::string mVideoChannel;

  x264_param_t mParams;
  x264_t* mEncoder;
  x264_picture_t mPictureIn;
  x264_picture_t mPictureOut;
  struct SwsContext* mConverterContext;
  int64_t mLastPublishTime;

  void setup() {
    auto lcm = mLcmWrapper->get();
    lcm->subscribe(mCameraChannel, &State::onCameras, this);

    // x264 params
    int width = 1024/mScaleFactor;  // TODO: get from config
    int height = 1024/mScaleFactor;
    x264_param_default_preset(&mParams, "veryfast", "zerolatency");
    mParams.i_threads = 1;
    mParams.i_width = width;
    mParams.i_height = height;
    mParams.i_fps_num = mPublishPeriod*1000;
    mParams.i_fps_den = 1000;
    // Intra refresh:
    mParams.i_keyint_max = 5/mPublishPeriod;
    mParams.b_intra_refresh = 1;
    //Rate control:
    mParams.rc.i_rc_method = X264_RC_CRF;
    //mParams.rc.f_rf_constant = 40;
    //mParams.rc.f_rf_constant_max = 50;
    mParams.rc.i_bitrate = 500;
    mParams.rc.i_vbv_buffer_size = 2000000;
    mParams.rc.i_vbv_max_bitrate = 1000;
    //For streaming:
    mParams.b_repeat_headers = 1;
    mParams.b_annexb = 1;
    x264_param_apply_profile(&mParams, "baseline");

    // set up encoder
    mEncoder = x264_encoder_open(&mParams);
    x264_picture_alloc(&mPictureIn, X264_CSP_I420, width, height);
    // TODO: need corresponding clean() call

    // TODO: other params? figure out bitrate stuff

    // set up converter
    mConverterContext = sws_getContext(width, height, PIX_FMT_RGB24,
                                       width, height, PIX_FMT_YUV420P,
                                       SWS_FAST_BILINEAR, NULL, NULL, NULL);

  }


  void onCamera(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const bot_core::image_t* iMessage) {
    if ((iMessage->utime - mLastPublishTime) < mPublishPeriod*1e6) return;

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
    cv::resize(cvImg,cvImg,cv::Size(),1.0f/mScaleFactor,1.0f/mScaleFactor,
               cv::INTER_AREA);

    // convert to yuv420
    w = cvImg.cols;
    h = cvImg.rows;
    int stride = w*3;
    sws_scale(mConverterContext, &cvImg.data, &stride, 0, h,
              mPictureIn.img.plane, mPictureIn.img.i_stride);

    // encode
    x264_nal_t* nals;
    int numNals;
    int numBytes = x264_encoder_encode(mEncoder, &nals, &numNals,
                                       &mPictureIn, &mPictureOut);
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
    mOutputLcm->publish(mVideoChannel, &rawMsg);
    mLastPublishTime = iMessage->utime;

    // TODO: error checking

    std::cout << "encoded " << numBytes << " " << curIndex << std::endl;
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

  int scaleFactor = 16;
  int bitRate = 1000;
  double publishFrequency = 2;
  std::string cameraChannel = "CAMERA";
  std::string videoChannel = "CAMERA_STREAM";

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(scaleFactor, "s", "scale-factor", "downsample factor");
  opt.add(publishFrequency, "f", "publish-frequency", "publish frequency (Hz)");
  opt.add(bitRate, "b", "bit-rate", "target bit rate (bps)");
  opt.add(cameraChannel, "c", "camera-channel", "camera channel");
  opt.add(videoChannel, "v", "video-channel", "video channel");
  opt.parse();

  // create lcm wrapper
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  std::shared_ptr<drc::LcmWrapper> lcmWrapper(new drc::LcmWrapper(lcm));

  std::shared_ptr<lcm::LCM> outLcm
    (new lcm::LCM("udpm://239.255.76.67:7667?ttl=0"));

  // set up state
  std::shared_ptr<State> state(new State());
  state->mLcmWrapper = lcmWrapper;
  state->mCameraChannel = cameraChannel;
  state->mVideoChannel = videoChannel;
  state->mScaleFactor = scaleFactor;
  state->mPublishPeriod = 1/publishFrequency;
  state->mOutputLcm = outLcm;
  state->setup();

  // run
  lcmWrapper->startHandleThread(true);

  // clean up
  return 0;
}
