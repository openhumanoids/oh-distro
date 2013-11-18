#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/sensor_request_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>
#include <lcmtypes/multisense/images_t.hpp>

#include <opencv2/opencv.hpp>

#include <ConciseArgs>

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

  void onImages(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                const multisense::images_t* iMessage) {
    for (int i = 0; i < iMessage->n_images; ++i) {
      if (iMessage->image_types[i] == multisense::images_t::LEFT) {
        std::lock_guard<std::mutex> lock(mMutex);
        mLatestImage = iMessage->images[i];
        break;
      }
    }
  }
};

struct ImageTransmit {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  typedef std::unordered_map<int, ChannelData::Ptr> ChannelGroup;
  ChannelGroup mChannels;

  int mJpegQuality;
  int mDownSampleFactor;

  ImageTransmit() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mLcm->subscribe("SENSOR_REQUEST", &ImageTransmit::onSensorRequest, this);
    mLcm->subscribe("TRIGGER_CAMERA", &ImageTransmit::onTrigger, this);
    setQuality(drc::sensor_request_t::QUALITY_LOW);
  }

  void addChannel(const int iRequestType, const std::string& iChannel) {
    ChannelData::Ptr data(new ChannelData());
    data->mChannelBase = iChannel;
    data->mRequestType = iRequestType;
    data->mChannelTransmit = data->mChannelBase + "_TX";
    mLcm->subscribe(data->mChannelBase, &ChannelData::onImage, data.get());
    mChannels[iRequestType] = data;
  }

  void addMultiChannel(const int iRequestType, const std::string& iChannel, 
                       const std::string& iSuffix) {
    ChannelData::Ptr data(new ChannelData());
    data->mChannelBase = iChannel;
    data->mRequestType = iRequestType;
    data->mChannelTransmit = data->mChannelBase + iSuffix + "_TX";
    mLcm->subscribe(data->mChannelBase, &ChannelData::onImages, data.get());
    mChannels[iRequestType] = data;
  }

  void setQuality(const int iQuality) {
    switch (iQuality) {
    case drc::sensor_request_t::QUALITY_LOW:
      mJpegQuality = 50;
      mDownSampleFactor = 4;
      break;
    case drc::sensor_request_t::QUALITY_MED:
      mJpegQuality = 70;
      mDownSampleFactor = 4;
      break;
    case drc::sensor_request_t::QUALITY_HIGH:
      mJpegQuality = 50;
      mDownSampleFactor = 2;
      break;
    default:
      break;
    }
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }

  void onSensorRequest(const lcm::ReceiveBuffer* iBuf,
                       const std::string& iChannel,
                       const drc::sensor_request_t* iMessage) {
    if (iMessage->camera_compression >= 0) {
      setQuality(iMessage->camera_compression);
    }
  }

  void onTrigger(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                 const drc::data_request_t* iMessage) {
    ChannelGroup::const_iterator item = mChannels.find(iMessage->type);
    if (item == mChannels.end()) return;
    const ChannelData::Ptr data = item->second;
    std::lock_guard<std::mutex> lock(data->mMutex);
    bot_core::image_t& img = data->mLatestImage;
    if (img.size == 0) return;

    // uncompress
    if (img.pixelformat == bot_core::image_t::PIXEL_FORMAT_MJPEG) {
      cv::Mat uncomp = cv::imdecode(cv::Mat(img.data), -1);
      if (uncomp.channels() == 1) {
        img.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
      }
      else if (uncomp.channels() == 3) {
        cv::cvtColor(uncomp, uncomp, CV_BGR2RGB);
        img.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
      }
      else {
        std::cout << "ImageTransmit: unrecognized compressed data" << std::endl;
        return;
      }
      img.data.resize(uncomp.step*uncomp.rows);
      std::copy(uncomp.data, uncomp.data + uncomp.step*uncomp.rows,
                img.data.begin());
      img.size = img.data.size();
    }

    // determine parameters from pixel format
    int numChannels;
    int imgType;
    switch (img.pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      numChannels = 1;
      imgType = CV_8UC1;
      break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      numChannels = 3;
      imgType = CV_8UC3;
      break;
    default:
      std::cout << "ImageTransmit: invalid pixel format on channel " <<
        iChannel << std::endl;
      return;
    }

    void* bytes = const_cast<void*>(static_cast<const void*>(img.data.data()));
    cv::Mat imgOrig(img.height, img.width, imgType, bytes, img.row_stride);

    // downsample
    cv::Mat outImage;
    cv::Size newSize(img.width/mDownSampleFactor, img.height/mDownSampleFactor);
    cv::resize(imgOrig, outImage, newSize, 0, 0, cv::INTER_AREA);

    // form new image message
    bot_core::image_t msg;
    msg.utime = img.utime;
    msg.width = newSize.width;
    msg.height = newSize.height;
    msg.row_stride = numChannels*newSize.width;
    msg.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    msg.nmetadata = 0;

    // jpeg compress
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, mJpegQuality };
    if (numChannels == 3) cv::cvtColor(outImage, outImage, CV_RGB2BGR);
    if (!cv::imencode(".jpg", outImage, msg.data, params)) {
      std::cout << "Error encoding jpeg image!" << std::endl;
    }
    msg.size = msg.data.size();
    
    // transmit
    mLcm->publish(data->mChannelTransmit, &msg);
    std::cout << "sent image (" << msg.size << " bytes) on " <<
      data->mChannelTransmit << " with quality=" << mJpegQuality <<
      " and downsample=" << mDownSampleFactor << std::endl;
  }

};

int main(const int iArgc, const char** iArgv) {
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.parse();

  ImageTransmit obj;
  obj.addMultiChannel(drc::data_request_t::CAMERA_IMAGE_HEAD_LEFT, "CAMERA", "_LEFT");
  obj.addMultiChannel(drc::data_request_t::CAMERA_IMAGE_HEAD_RIGHT, "CAMERA", "_RIGHT");
  obj.addChannel(drc::data_request_t::CAMERA_IMAGE_LCHEST, "CAMERACHEST_LEFT");
  obj.addChannel(drc::data_request_t::CAMERA_IMAGE_RCHEST, "CAMERACHEST_RIGHT");
  obj.start();
}
