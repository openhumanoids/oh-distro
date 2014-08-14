#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <lcmtypes/bot_core/image_t.hpp>

#include <opencv2/opencv.hpp>

#include <ConciseArgs>

struct ImageAnaglyphFuser {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::string mInputChannelLeft;
  std::string mInputChannelRight;
  std::string mOutputChannel;
  BotParam* mBotParam;
  lcm::Subscription* mSubscriptionLeft;
  lcm::Subscription* mSubscriptionRight;
  int mJpegQuality;
  unsigned int mMaxQueueSize;

  bool mIsRunning;
  std::list<std::shared_ptr<bot_core::image_t> > mLeftQueue;
  std::list<std::shared_ptr<bot_core::image_t> > mRightQueue;
  std::condition_variable mCondition;
  std::mutex mDataMutex;
  std::mutex mConditionMutex;
  std::thread mThread;

  ImageAnaglyphFuser() {
    mIsRunning = false;
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotWrapper.reset(new drc::BotWrapper(mLcm));
    mSubscriptionLeft = mSubscriptionRight = NULL;
    mMaxQueueSize = 10;
  }

  ~ImageAnaglyphFuser() {
    if (mThread.joinable()) mThread.join();
  }

  cv::Mat
  convertToGray(const std::shared_ptr<bot_core::image_t>& iMessage) const {
    cv::Mat out;
    int imgType = CV_8UC3;
    switch (iMessage->pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      out = cv::imdecode(cv::Mat(iMessage->data), -1);
      if (out.channels() == 3) cv::cvtColor(out, out, CV_RGB2GRAY);
      break;
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      imgType = CV_8UC1;
      out = cv::Mat(iMessage->height, iMessage->width, imgType,
                   (void*)iMessage->data.data(), iMessage->row_stride);
      break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      imgType = CV_8UC3;
      out = cv::Mat(iMessage->height, iMessage->width, imgType,
                   (void*)iMessage->data.data(), iMessage->row_stride);
      cv::cvtColor(out, out, CV_RGB2GRAY);
      break;
    default: std::cout << "error: pixel format not supported\n";
      break;
    }
    return out;
  }

  void publish(const std::shared_ptr<bot_core::image_t>& iLeft,
               const std::shared_ptr<bot_core::image_t>& iRight) const {
    cv::Mat left = convertToGray(iLeft);
    cv::Mat right = convertToGray(iRight);

    // compose into anaglyph
    cv::Mat anaglyph;
    std::vector<cv::Mat> channels = { right, right, left };
    cv::merge(channels, anaglyph);

    // compress
    bot_core::image_t msg;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, mJpegQuality };
    if (!cv::imencode(".jpg", anaglyph, msg.data, params)) {
      std::cout << "error encoding jpeg image" << std::endl;
    }
    msg.size = msg.data.size();
    
    // transmit
    msg.utime = iLeft->utime;
    msg.width = anaglyph.cols;
    msg.height = anaglyph.rows;
    msg.row_stride = anaglyph.step;
    msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    msg.nmetadata = 0;
    mLcm->publish(mOutputChannel, &msg);
    std::cout << "transmitted anaglyph on " << mOutputChannel << std::endl;
  }


  void operator()() {
    mIsRunning = true;
    while (mIsRunning) {
      std::unique_lock<std::mutex> lock(mConditionMutex);
      mCondition.wait_for(lock, std::chrono::milliseconds(100));
      typedef std::shared_ptr<bot_core::image_t> PCLImage;
      std::vector<std::pair<PCLImage,PCLImage> > pairs;
      {
        std::unique_lock<std::mutex> dataLock(mDataMutex);
        auto left = mLeftQueue.begin();
        while (left != mLeftQueue.end()) {
          bool found = false;
          auto right = mRightQueue.begin();
          while (right != mRightQueue.end()) {
            if ((*left)->utime == (*right)->utime) {
              pairs.push_back(std::make_pair(*left,*right));
              right = mRightQueue.erase(right);
              found = true;
              break;
            }
            ++right;
          }
          if (found) left = mLeftQueue.erase(left);
          else ++left;
        }
      }
      for (auto pair : pairs) {
        publish(pair.first, pair.second);
      }
    }
  }

  void onImageRight(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                    const bot_core::image_t* iMessage) {
    std::unique_lock<std::mutex> lock(mDataMutex);
    std::shared_ptr<bot_core::image_t> img(new bot_core::image_t(*iMessage));
    mRightQueue.push_back(img);
    while (mRightQueue.size() > mMaxQueueSize) mRightQueue.pop_front();
    mCondition.notify_one();
  }


  void onImageLeft(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                   const bot_core::image_t* iMessage) {
    std::unique_lock<std::mutex> lock(mDataMutex);
    std::shared_ptr<bot_core::image_t> img(new bot_core::image_t(*iMessage));
    mLeftQueue.push_back(img);
    while (mLeftQueue.size() > mMaxQueueSize) mLeftQueue.pop_front();
    mCondition.notify_one();
  }

  void setJpegQuality(const int iQuality) {
    mJpegQuality = iQuality;
  }

  bool setChannels(const std::string& iInputChannelLeft,
                   const std::string& iInputChannelRight,
                   const std::string& iOutputChannel) {
    if (mSubscriptionLeft != NULL) mLcm->unsubscribe(mSubscriptionLeft);
    if (mSubscriptionRight != NULL) mLcm->unsubscribe(mSubscriptionRight);

    mInputChannelLeft = iInputChannelLeft;
    mInputChannelRight = iInputChannelRight;
    mOutputChannel = iOutputChannel;

    mLcm->subscribe(mInputChannelLeft,&ImageAnaglyphFuser::onImageLeft,this);
    mLcm->subscribe(mInputChannelRight,&ImageAnaglyphFuser::onImageRight,this);
    return true;
  }

  void start() {
    mThread = std::thread(std::ref(*this));
    mLcmWrapper->startHandleThread(true);
  }
};

int main(const int iArgc, const char** iArgv) {
  int jpegQuality = 90;
  std::string inputChannelLeft = "CAMERA_LEFT";
  std::string inputChannelRight = "CAMERA_RIGHT";
  std::string outputChannel = "CAMERA_ANAGLYPH";
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(inputChannelLeft, "l", "left_channel",
          "incoming left camera channel");
  opt.add(inputChannelRight, "r", "right_channel",
          "incoming right camera channel");
  opt.add(outputChannel, "o", "output_channel",
          "camera for output warp and publish");
  opt.add(jpegQuality, "j", "jpeg_quality", "jpeg quality (1-100)");
  opt.parse();

  ImageAnaglyphFuser obj;
  obj.setJpegQuality(jpegQuality);
  if (!obj.setChannels(inputChannelLeft, inputChannelRight, outputChannel)) {
    return -1;
  }
  obj.start();
  return 1;
}
