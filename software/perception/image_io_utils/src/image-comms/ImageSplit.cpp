#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>

struct ChannelData {
  typedef std::shared_ptr<ChannelData> Ptr;
  std::string mChannelBase;
  std::string mChannelTransmit;
  bot_core::image_t mLatestImage;
  int mRequestType;
  std::mutex mMutex;

  ChannelData() {
    mLatestImage.size = 0;
  }

  void onImage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
               const bot_core::image_t* iMessage) {
    std::lock_guard<std::mutex> lock(mMutex);
    mLatestImage = *iMessage;
  }
};

struct ImageSplit {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  typedef std::unordered_map<int, ChannelData::Ptr> ChannelGroup;
  ChannelGroup mChannels;

  int mJpegQuality;
  int mDownSampleFactor;

  ImageSplit() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mLcm->subscribe("TRIGGER_CAMERA", &ImageSplit::onTrigger, this);
  }

  void addChannel(const std::string& iChannel, const int iRequestType) {
    ChannelData::Ptr data(new ChannelData());
    data->mChannelBase = iChannel;
    data->mChannelTransmit = iChannel + "LEFT";
    data->mRequestType = iRequestType;
    mLcm->subscribe(data->mChannelBase, &ChannelData::onImage, data.get());
    mChannels[iRequestType] = data;
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }

  void onTrigger(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                 const drc::data_request_t* iMessage) {
    ChannelGroup::const_iterator item = mChannels.find(iMessage->type);
    if (item == mChannels.end()) return;
    const ChannelData::Ptr data = item->second;
    std::lock_guard<std::mutex> lock(data->mMutex);
    const bot_core::image_t& img = data->mLatestImage;
    if (img.size == 0) return;

    std::vector<uint8_t> buf(&img.data[0],
                             &img.data[0] + img.height*img.row_stride);
    
    // transmit
    bot_core::image_t msg;
    msg.utime = img.utime;
    msg.width = img.width;
    msg.height = img.height;
    msg.row_stride = img.row_stride;
    msg.pixelformat = img.pixelformat;
    msg.size = buf.size();
    msg.data = buf;
    msg.nmetadata = 0;
    mLcm->publish(data->mChannelTransmit, &msg);
    std::cout << "sent image (" << msg.size << " bytes) on " <<
      data->mChannelTransmit << std::endl;
  }

};

int main() {
  ImageSplit obj;
  obj.addChannel("CAMERA_LHAND", drc::data_request_t::CAMERA_IMAGE_LHAND);
  obj.addChannel("CAMERA_RHAND", drc::data_request_t::CAMERA_IMAGE_RHAND);
  obj.start();
}
