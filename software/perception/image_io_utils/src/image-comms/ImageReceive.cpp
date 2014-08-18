#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_core/camtrans.h>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/subimage_response_t.hpp>

#include <opencv2/opencv.hpp>

#include <ConciseArgs>

struct ChannelData {
  typedef std::shared_ptr<ChannelData> Ptr;
  std::shared_ptr<lcm::LCM> mLcm;
  std::string mChannelBase;
  std::string mChannelReceive;
  std::string mChannelTransmit;
  int mWidth;
  int mHeight;
  int mJpegQuality;

  std::thread mThread;
  std::mutex mDataMutex;
  std::mutex mConditionMutex;
  std::condition_variable mCondition;
  std::list<std::shared_ptr<bot_core::image_t> > mDataQueue;
  std::shared_ptr<drc::subimage_response_t> mSubImage;
  bool mRunning;
  int mReceivedCount;
  int mProcessedCount;

  ChannelData() {
    mReceivedCount = mProcessedCount = 0;
    mRunning = false;
  }

  void handleImage(const std::shared_ptr<bot_core::image_t>& iImage) {
    // uncompress
    cv::Mat raw = cv::imdecode(cv::Mat(iImage->data), -1);
    if (raw.channels() == 3) cv::cvtColor(raw, raw, CV_BGR2RGB);

    // resize to normal
    cv::Mat img;
    cv::resize(raw, img, cv::Size(mWidth, mHeight));

    // insert subimage if appropriate
    if (mSubImage != NULL) {
      auto req = mSubImage->subimage_request;
      cv::Mat raw = cv::imdecode(cv::Mat(mSubImage->image.data), -1);
      if (raw.channels() == 3) cv::cvtColor(raw, raw, CV_BGR2RGB);
      cv::Mat roi(img, cv::Rect(req.x, req.y, req.w, req.h));
      cv::Mat subImg(roi.rows, roi.cols, roi.type(), raw.data,
                     mSubImage->image.row_stride);
      subImg.copyTo(roi);
      mSubImage = NULL;
      std::cout << "inserted subimage (" <<
        roi.rows << "x" << roi.cols << ")" << std::endl;
    }

    // form output message
    bot_core::image_t msg;
    msg.utime = iImage->utime;
    msg.width = img.cols;
    msg.height = img.rows;
    msg.row_stride = img.step;
    msg.nmetadata = 0;

    // re-compress if desired
    if (mJpegQuality < 100) {
      if (img.channels()==3) cv::cvtColor(img,img,CV_RGB2BGR);
      std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, mJpegQuality };
      if (!cv::imencode(".jpg", img, msg.data, params)) {
        std::cout << "error encoding jpeg image" << std::endl;
      }
      msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    }
    else {
      msg.data.resize(img.step*img.rows);
      std::copy(img.data, img.data + img.step*img.rows, msg.data.data());
      switch (img.channels()) {
      case 1: msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY; break;
      case 3: msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;  break;
      default:
        std::cout << "ImageReceive: invalid image type on channel " <<
          mChannelReceive << std::endl;
        return;
      }
    }
    msg.size = msg.data.size();

    // re-transmit
    mLcm->publish(mChannelTransmit, &msg);
    std::cout << "ImageReceive: re-transmitted image on " <<
      mChannelTransmit << std::endl;
  }

  void operator()() {
    mRunning = true;
    while (mRunning) {
      std::unique_lock<std::mutex> lock(mConditionMutex);
      mCondition.wait_for(lock, std::chrono::milliseconds(100));
      std::vector<std::shared_ptr<bot_core::image_t> > workQueue;
      {
        std::unique_lock<std::mutex> lock(mDataMutex);
        workQueue.reserve(mDataQueue.size());
        while (!mDataQueue.empty()) {
          workQueue.push_back(mDataQueue.front());
          mDataQueue.pop_front();
        }
      }
      for (auto img : workQueue) {
        handleImage(img);
        ++mProcessedCount;
        std::cout << mChannelTransmit << ": received " << mReceivedCount <<
          ", processed " << mProcessedCount << std::endl;
      }
    }
  }

  void onImage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
               const bot_core::image_t* iMessage) {
    std::unique_lock<std::mutex> lock(mDataMutex);
    std::shared_ptr<bot_core::image_t> img(new bot_core::image_t(*iMessage));
    mDataQueue.push_back(img);
    const int maxQueueSize = 10;
    ++mReceivedCount;
    while ((int)mDataQueue.size() > maxQueueSize) {
      mDataQueue.pop_front();
    }
    mCondition.notify_one();
  }

  void onSubImage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                  const drc::subimage_response_t* iMessage) {
    mSubImage.reset(new drc::subimage_response_t(*iMessage));
  }
};

struct ImageReceive {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  typedef std::unordered_map<std::string, ChannelData::Ptr> ChannelGroup;
  ChannelGroup mChannels;
  BotParam* mBotParam;
  int mJpegQuality;

  ImageReceive() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotParam = bot_param_get_global(mLcmWrapper->get()->getUnderlyingLCM(),0);
    mJpegQuality = 90;
  }

  void setJpegQuality(const int iQuality) {
    mJpegQuality = iQuality;
  }

  void addChannel(const std::string& iChannel) {
    ChannelData::Ptr data(new ChannelData());
    data->mChannelBase = iChannel;
    data->mChannelReceive = iChannel + "_TX";
    data->mChannelTransmit = iChannel;
    BotCamTrans* camTrans =
      bot_param_get_new_camtrans(mBotParam, iChannel.c_str());
    data->mWidth = bot_camtrans_get_width(camTrans);
    data->mHeight = bot_camtrans_get_height(camTrans);
    data->mLcm = mLcm;
    data->mJpegQuality = mJpegQuality;
    mChannels[data->mChannelReceive] = data;
    data->mThread = std::thread(std::ref(*data));
    mLcm->subscribe(data->mChannelReceive, &ChannelData::onImage, data.get());
    mLcm->subscribe(data->mChannelBase + "_SUB",
                    &ChannelData::onSubImage, data.get());
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }
};

int main(const int iArgc, const char** iArgv) {
  int jpegQuality = 90;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(jpegQuality, "j", "jpeg_quality",
          "jpeg quality (1-100), where 100 is uncompressed");
  opt.parse();
  
  ImageReceive obj;
  obj.setJpegQuality(jpegQuality);
  obj.addChannel("CAMERA_LEFT");
  obj.addChannel("CAMERA_RIGHT");
  obj.addChannel("CAMERACHEST_LEFT");
  obj.addChannel("CAMERACHEST_RIGHT");
  obj.start();
}
