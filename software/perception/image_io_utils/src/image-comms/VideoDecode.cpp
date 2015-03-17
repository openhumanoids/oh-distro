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
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;

  int mScaleFactor;
  std::string mInputChannel;
  std::string mOutputChannel;

  int mWidth;
  int mHeight;
  bool mInitialized;
  AVFrame* mFrameIn;
  AVFrame* mFrameOut;
  AVCodec* mCodec;
  AVCodecContext* mContext;
  struct SwsContext* mConverterContext;

  void setup() {
    auto lcm = mLcmWrapper->get();
    lcm->subscribe(mInputChannel, &State::onVideoFrame, this);
    mInitialized = false;

    mWidth = 1024/mScaleFactor;
    mHeight = 1024/mScaleFactor; // TODO: TEMP; get from config

    // TODO: can we deduce this from the stream itself?
    if (!mInitialized) {
      setupCodec();
      mInitialized = true;
    }
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

  void setupCodec() {
    int w = mWidth;
    int h = mHeight;

    avcodec_register_all();
    mCodec = avcodec_find_decoder(CODEC_ID_H264);
    mContext = avcodec_alloc_context3(mCodec);
    mContext->width = w;
    mContext->height = h;
    mContext->pix_fmt = PIX_FMT_YUV420P;
    avcodec_open2(mContext, mCodec, NULL);

    // pictures
    mFrameIn = allocatePicture(PIX_FMT_YUV420P, w, h);
    mFrameOut = allocatePicture(PIX_FMT_RGB24, w, h);

    // set up converter
    mConverterContext = sws_getContext(w, h, PIX_FMT_YUV420P,
                                       w, h, PIX_FMT_RGB24,
                                       SWS_FAST_BILINEAR, NULL, NULL, NULL);
  }

  void onVideoFrame(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const bot_core::raw_t* iMessage) {
    int size = iMessage->length;
    int w = mWidth;
    int h = mHeight;

    AVPacket packet;
    av_new_packet(&packet, size);
    packet.data = (uint8_t*)iMessage->data.data();
    packet.size = size;

    int finished = 0;
    int rgbSize = mWidth*mHeight*3;
    std::vector<uint8_t> buf(rgbSize);
    int ret = avcodec_decode_video2(mContext, mFrameIn, &finished, &packet);
    if (ret < 0) {
      std::cout << "BAD!" << std::endl;
      return;
    }
    if (finished == 0) {
      std::cout << "BAD2!" << std::endl;
      // TODO: error handling
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

  int scaleFactor = 16;
  std::string inputChannel = "CAMERA_STREAM";
  std::string outputChannel = "DECODED_IMAGE";

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(scaleFactor, "s", "scale-factor", "downsample factor");
  opt.add(outputChannel, "o", "output-channel", "output channel");
  opt.parse();

  // create lcm wrapper
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  std::shared_ptr<drc::LcmWrapper> lcmWrapper(new drc::LcmWrapper(lcm));

  // set up state
  std::shared_ptr<State> state(new State());
  state->mLcmWrapper = lcmWrapper;
  state->mScaleFactor = scaleFactor;
  state->mInputChannel = inputChannel;
  state->mOutputChannel = outputChannel;
  state->setup();

  // run
  lcmWrapper->startHandleThread(true);

  // clean up
  return 0;
}
