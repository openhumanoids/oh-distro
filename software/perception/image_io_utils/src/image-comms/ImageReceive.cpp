#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_core/camtrans.h>

#include <lcmtypes/bot_core/image_t.hpp>

#include <opencv2/opencv.hpp>

struct ChannelData {
  typedef std::shared_ptr<ChannelData> Ptr;
  std::shared_ptr<lcm::LCM> mLcm;
  std::string mChannelBase;
  std::string mChannelReceive;
  std::string mChannelTransmit;
  int mWidth;
  int mHeight;

  std::thread mThread;
  std::mutex mDataMutex;
  std::mutex mConditionMutex;
  std::condition_variable mCondition;
  std::list<std::shared_ptr<bot_core::image_t> > mDataQueue;
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
    if (raw.channels() == 3) cv::cvtColor(raw, raw, CV_RGB2BGR);

    // resize to normal
    cv::Mat img;
    cv::resize(raw, img, cv::Size(mWidth, mHeight));

    int pixelFormat;
    switch (img.channels()) {
    case 1:
      pixelFormat = bot_core::image_t::PIXEL_FORMAT_GRAY;
      break;
    case 3:
      pixelFormat = bot_core::image_t::PIXEL_FORMAT_RGB;
      break;
    default:
      std::cout << "ImageReceive: invalid image type on channel " <<
        mChannelReceive << std::endl;
      return;
    }

    // copy final data
    std::vector<uint8_t> buf(img.data, img.data + img.step*img.rows);

    // re-transmit
    bot_core::image_t msg;
    msg.utime = iImage->utime;
    msg.width = img.cols;
    msg.height = img.rows;
    msg.row_stride = img.step;
    msg.pixelformat = pixelFormat;
    msg.size = buf.size();
    msg.data = buf;
    msg.nmetadata = 0;
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

};

struct ImageReceive {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  typedef std::unordered_map<std::string, ChannelData::Ptr> ChannelGroup;
  ChannelGroup mChannels;
  BotParam* mBotParam;

  ImageReceive() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotParam = bot_param_get_global(mLcmWrapper->get()->getUnderlyingLCM(),0);
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
    mChannels[data->mChannelReceive] = data;
    data->mThread = std::thread(std::ref(*data));
    mLcm->subscribe(data->mChannelReceive, &ChannelData::onImage, data.get());
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }
};

int main() {
  ImageReceive obj;

  obj.addChannel("CAMERA_LEFT");
  obj.addChannel("CAMERA_RIGHT");
  obj.addChannel("CAMERACHEST_LEFT");
  obj.addChannel("CAMERACHEST_RIGHT");
  obj.start();
}
