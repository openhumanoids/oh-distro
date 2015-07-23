#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <mutex>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
// #include <lcmtypes/drc/camera_settings_t.hpp>
#include <lcmtypes/drc/data_request_t.hpp>
#include <lcmtypes/drc/subimage_request_t.hpp>
#include <lcmtypes/drc/subimage_response_t.hpp>

#include <opencv2/opencv.hpp>

#include <ConciseArgs>

#define QUALITY_LOW   1
#define QUALITY_MED   2
#define QUALITY_HIGH  3
#define QUALITY_SUPER 4

struct ChannelData {
  typedef std::shared_ptr<ChannelData> Ptr;
  std::shared_ptr<lcm::LCM> mLcm;
  std::string mChannelBase;
  std::string mChannelTransmit;
  std::string mChannelSubImage;
  bot_core::image_t mLatestImage;
  int mRequestType;
  int mMultisenseType;
  std::mutex mDataMutex;
  std::shared_ptr<drc::subimage_request_t> mSubImageRequest;
  int mJpegQuality;
  int mDownSampleFactor;

  ChannelData() {
    mLatestImage.size = 0;
    setQuality(QUALITY_LOW);
  }

  void setQuality(const int iQuality) {
    switch (iQuality) {
    case QUALITY_LOW:
      mJpegQuality = 50;
      mDownSampleFactor = 4;
      break;
    case QUALITY_MED:
      mJpegQuality = 70;
      mDownSampleFactor = 4;
      break;
    case QUALITY_HIGH:
      mJpegQuality = 50;
      mDownSampleFactor = 2;
      break;
    case QUALITY_SUPER:
      mJpegQuality = 90;
      mDownSampleFactor = 1;
    default:
      break;
    }
  }

  void onImage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
               const bot_core::image_t* iMessage) {
    std::lock_guard<std::mutex> lock(mDataMutex);
    mLatestImage = *iMessage;
  }

  void onImages(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                const bot_core::images_t* iMessage) {
    for (int i = 0; i < iMessage->n_images; ++i) {
      if (iMessage->image_types[i] == mMultisenseType) {
        std::lock_guard<std::mutex> lock(mDataMutex);
        mLatestImage = iMessage->images[i];
        break;
      }
    }
  }

  void publish() {
    bot_core::image_t img;
    {
      std::lock_guard<std::mutex> lock(mDataMutex);
      img = mLatestImage;
    }
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
        mChannelBase << std::endl;
      return;
    }

    void* bytes = const_cast<void*>(static_cast<const void*>(img.data.data()));
    cv::Mat imgOrig(img.height, img.width, imgType, bytes, img.row_stride);

    // grab and send subimage if applicable
    auto& req = mSubImageRequest;
    if (req != NULL) {
      if ((req->w > 0) && (req->h > 0)) {
        if (req->x < 0) req->x = 0;
        if (req->y < 0) req->y = 0;
        if (req->x+req->w > img.width) req->w = img.width-req->x;
        if (req->y+req->h > img.height) req->h = img.height-req->y;
        cv::Mat subImg;
        cv::Mat(imgOrig, cv::Rect(req->x,req->y,req->w,req->h)).copyTo(subImg);
        drc::subimage_response_t msg;
        msg.image.utime = img.utime;
        msg.image.width = subImg.cols;
        msg.image.height = subImg.rows;
        msg.image.row_stride = subImg.channels()*msg.image.width;
        msg.image.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
        msg.image.nmetadata = 0;
        std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 70 };
        if (subImg.channels() == 3) cv::cvtColor(subImg, subImg, CV_RGB2BGR);
        if (!cv::imencode(".jpg", subImg, msg.image.data, params)) {
          std::cout << "Error encoding jpeg subimage!" << std::endl;
        }
        msg.image.size = msg.image.data.size();
        msg.subimage_request = *mSubImageRequest;
        mLcm->publish(mChannelSubImage, &msg);
        std::cout << "sent subimage image (" << subImg.cols << "x" <<
          subImg.rows << "), " << msg.image.size << " bytes" << std::endl;
      }
    }

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
    mLcm->publish(mChannelTransmit, &msg);
    std::cout << "sent image (" << msg.size << " bytes) on " <<
      mChannelTransmit << " with quality=" << mJpegQuality <<
      " and downsample=" << mDownSampleFactor << std::endl;
  }
};

struct ImageTransmit {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  typedef std::unordered_map<int, ChannelData::Ptr> ChannelGroup;
  ChannelGroup mChannels;

  ImageTransmit() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    //mLcm->subscribe("CAMERA_SETTINGS", &ImageTransmit::onCameraSettings, this);
    mLcm->subscribe("TRIGGER_CAMERA", &ImageTransmit::onTrigger, this);
    mLcm->subscribe("SUBIMAGE_REQUEST", &ImageTransmit::onSubRequest, this);
  }

  void addChannel(const int iRequestType, const std::string& iChannel) {
    ChannelData::Ptr data(new ChannelData());
    data->mLcm = mLcm;
    data->mChannelBase = iChannel;
    data->mRequestType = iRequestType;
    data->mChannelTransmit = data->mChannelBase + "_TX";
    data->mChannelSubImage = data->mChannelBase + "_SUB";
    mLcm->subscribe(data->mChannelBase, &ChannelData::onImage, data.get());
    mChannels[iRequestType] = data;
  }

  void addMultiChannel(const int iRequestType, const std::string& iChannel, 
                       const int iMultisenseType) {
    ChannelData::Ptr data(new ChannelData());
    data->mLcm = mLcm;
    data->mChannelBase = iChannel;
    data->mRequestType = iRequestType;
    std::string suffix;
    switch (iMultisenseType) {
    case bot_core::images_t::LEFT: suffix = "_LEFT"; break;
    case bot_core::images_t::RIGHT: suffix = "_RIGHT"; break;
    default: break;
    }
    data->mChannelTransmit = data->mChannelBase + suffix + "_TX";
    data->mChannelSubImage = data->mChannelBase + suffix + "_SUB";
    data->mMultisenseType = iMultisenseType;
    mLcm->subscribe(data->mChannelBase, &ChannelData::onImages, data.get());
    mChannels[iRequestType] = data;
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }

  /*
  void onCameraSettings(const lcm::ReceiveBuffer* iBuf,
                        const std::string& iChannel,
                        const drc::camera_settings_t* iMessage) {
    auto item = mChannels.find(iMessage->data_request.type);
    if (item == mChannels.end()) return;
    item->second->setQuality(iMessage->quality);
  }
  */

  void onSubRequest(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const drc::subimage_request_t* iMessage) {
    ChannelGroup::const_iterator item =
      mChannels.find(iMessage->data_request.type);
    if (item == mChannels.end()) return;
    const ChannelData::Ptr data = item->second;
    data->mSubImageRequest.reset(new drc::subimage_request_t(*iMessage));
  }

  void onTrigger(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                 const drc::data_request_t* iMessage) {
    ChannelGroup::const_iterator item = mChannels.find(iMessage->type);
    if (item == mChannels.end()) return;
    const ChannelData::Ptr data = item->second;
    data->publish();
  }

};

int main(const int iArgc, const char** iArgv) {
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.parse();

  ImageTransmit obj;
  obj.addMultiChannel(drc::data_request_t::CAMERA_IMAGE_HEAD_LEFT, "CAMERA", bot_core::images_t::LEFT);
  obj.addMultiChannel(drc::data_request_t::CAMERA_IMAGE_HEAD_RIGHT, "CAMERA", bot_core::images_t::RIGHT);
  obj.addChannel(drc::data_request_t::CAMERA_IMAGE_LCHEST, "CAMERACHEST_LEFT");
  obj.addChannel(drc::data_request_t::CAMERA_IMAGE_RCHEST, "CAMERACHEST_RIGHT");
  obj.start();
}
