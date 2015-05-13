#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <opencv2/opencv.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/raw_t.hpp>

extern "C" {
#include <x264.h>
#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
}

/*
 * see https://github.com/filippobrizzi/raw_rgb_straming
 */

struct State {
  drc::LcmWrapper::Ptr mLcmWrapper;
  drc::BotWrapper::Ptr mBotWrapper;

  int mScaleFactor;
  std::string mInputChannel;
  std::string mOutputChannel;
  std::string mCameraChannel;

  int mWidth;
  int mHeight;
  bool mInitialized;
  AVFrame* mFrameIn;
  AVFrame* mFrameOut;
  AVCodec* mCodec;
  AVCodecContext* mContext;
  struct SwsContext* mConverterContext;

  bool setup() {
    auto lcm = mLcmWrapper->get();
    lcm->subscribe(mInputChannel, &State::onVideoFrame, this);
    mInitialized = false;

    std::string keyBase = "cameras." + mCameraChannel + ".intrinsic_cal";
    if (!mBotWrapper->hasKey(keyBase)) {
      std::cout << "error: no such camera " << mCameraChannel << std::endl;
      return false;
    }
    int nativeWidth = mBotWrapper->getInt(keyBase + ".width");
    int nativeHeight = mBotWrapper->getInt(keyBase + ".height");
    mWidth = nativeWidth/mScaleFactor;
    mHeight = nativeHeight/mScaleFactor;

    // TODO: can we deduce this from the stream itself?
    if (!mInitialized) {
      if (!setupCodec()) return false;
      mInitialized = true;
    }

    return true;
  }

  AVFrame* allocatePicture(const int iFormat, const int iWidth,
                           const int iHeight) {
    AVFrame* picture;
    picture = avcodec_alloc_frame();
    int size = avpicture_get_size((PixelFormat)iFormat, iWidth, iHeight);
    uint8_t* buf = (uint8_t*)malloc(size);
    avpicture_fill((AVPicture*)picture, buf, (PixelFormat)iFormat,
                   iWidth, iHeight);
    return picture;
  }

  bool setupCodec() {
    int w = mWidth;
    int h = mHeight;

    avcodec_register_all();
    av_log_set_callback([](void*, int, const char*, va_list){});

    mCodec = avcodec_find_decoder(CODEC_ID_H264);
    if (mCodec == NULL) {
      std::cout << "error: cannot find H264 codec" << std::endl;
      return false;
    }

    mContext = avcodec_alloc_context3(mCodec);
    if (mContext == NULL) {
      std::cout << "error: cannot allocate context" << std::endl;
      return false;
    }

    mContext->width = w;
    mContext->height = h;
    mContext->pix_fmt = PIX_FMT_YUV420P;
    if (avcodec_open2(mContext, mCodec, NULL) < 0) {
      std::cout << "error: cannot open codec" << std::endl;
      return false;
    }

    // pictures
    mFrameIn = allocatePicture(PIX_FMT_YUV420P, w, h);
    mFrameOut = allocatePicture(PIX_FMT_RGB24, w, h);

    // set up converter
    mConverterContext = sws_getContext(w, h, PIX_FMT_YUV420P,
                                       w, h, PIX_FMT_RGB24,
                                       SWS_FAST_BILINEAR, NULL, NULL, NULL);
    if (mConverterContext == NULL) {
      std::cout << "error: cannot create swscaler" << std::endl;
      return false;
    }

    return true;
  }

  void onVideoFrame(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const bot_core::raw_t* iMessage) {
    int size = iMessage->length;
    int w = mWidth;
    int h = mHeight;

    AVPacket packet;
    packet.data = (uint8_t*)iMessage->data.data();
    packet.size = size;

    int finished = 0;
    int rgbSize = mWidth*mHeight*3;
    std::vector<uint8_t> buf(rgbSize);
    int ret = avcodec_decode_video2(mContext, mFrameIn, &finished, &packet);
    if ((ret < 0) || (finished == 0)) {
      std::cout << "error: cannot decode packet" << std::endl;
      return;
    }

    sws_scale(mConverterContext, mFrameIn->data, mFrameIn->linesize, 0, h,
              mFrameOut->data, mFrameOut->linesize);
    avpicture_layout((AVPicture*)mFrameOut, PIX_FMT_RGB24, w, h,
                     buf.data(), buf.size());

    // color convert and upsample
    cv::Mat cvImg(h, w, CV_8UC3, buf.data());
    cv::cvtColor(cvImg, cvImg, CV_BGR2RGB);
    cv::resize(cvImg, cvImg, cv::Size(), mScaleFactor, mScaleFactor);

    // prepare output message
    bot_core::image_t img;
    img.utime = iMessage->utime;
    img.width = cvImg.cols;
    img.height = cvImg.rows;
    img.row_stride = cvImg.step;
    img.nmetadata = 0;

    // convert to jpeg
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 95 };
    cv::imencode(".jpg", cvImg, img.data, params);
    img.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;

    img.size = img.data.size();
    mLcmWrapper->get()->publish(mOutputChannel, &img);
  }
  
};

int main(const int iArgc, const char** iArgv) {

  int scaleFactor = 8;
  std::string inputChannel = "CAMERA_STREAM";
  std::string outputChannel = "DECODED_IMAGE";
  std::string cameraChannel = "CAMERA_LEFT";

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(scaleFactor, "s", "scale-factor", "downsample factor");
  opt.add(inputChannel, "i", "input-channel", "input video stream channel");
  opt.add(outputChannel, "o", "output-channel", "output channel");
  opt.add(cameraChannel, "c", "camera-channel", "camera channel");
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
  state->mScaleFactor = scaleFactor;
  state->mInputChannel = inputChannel;
  state->mOutputChannel = outputChannel;
  state->mCameraChannel = cameraChannel;

  if (!state->setup()) return -1;

  // run
  lcmWrapper->startHandleThread(true);

  // clean up
  return 0;
}
