#include "Camera.hpp"

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <multisense-lib/MultiSenseChannel.hh>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/multisense/image_t.hpp>
#include <lcmtypes/multisense/images_t.hpp>

#include <opencv2/opencv.hpp>
#include <zlib.h>

struct Camera::Imp {

  typedef std::list<std::shared_ptr<multisense::image_t> > MessageQueue;

  struct ImageData {
    Imp* mImp;

    int mWidth;
    int mHeight;

    cv::Mat mCalibrationMapX;
    cv::Mat mCalibrationMapY;
    cv::Mat mYuv;
    cv::Mat mRgbRaw;
    cv::Mat mRgbRectified;
    cv::Mat mLuma;
    int mLumaFrameId;

    MessageQueue mMessageQueue;

    ImageData(Imp* iImp) {
      mImp = iImp;
    }

    void allocate(const int iWidth, const int iHeight,
                  crl::multisense::image::Calibration::Data& iData) {
      mWidth = iWidth;
      mHeight = iHeight;

      // allocate memory
      mLuma = cv::Mat(mHeight, mWidth, CV_8UC1);
      mYuv = cv::Mat(mHeight + mHeight/2, mWidth, CV_8UC1);
      mRgbRaw = cv::Mat(mHeight, mWidth, CV_8UC3);
      mRgbRectified = cv::Mat(mHeight, mWidth, CV_8UC3);
      mLumaFrameId = -1;

      // compute distortion rectification maps
      double factor = (double)mWidth / mImp->mCalibratedWidth;
      cv::Mat scale = (cv::Mat_<float>(3,3) << factor,0,0, 0,factor,0, 0,0,1);
      cv::Mat M(3, 3, CV_32F, &iData.M);
      cv::Mat D(1, 8, CV_32F, &iData.D);
      cv::Mat R(3, 3, CV_32F, &iData.R);
      cv::Mat P(3, 4, CV_32F, &iData.P);
      M = scale*M;
      P = scale*P;
      cv::initUndistortRectifyMap(M, D, R, P, cv::Size(mWidth, mHeight),
                                  CV_32FC1, mCalibrationMapX, mCalibrationMapY);
      cv::convertMaps(mCalibrationMapX, mCalibrationMapY,
                      mCalibrationMapX, mCalibrationMapY, CV_16SC2, false);
    }

    // c-style shims for c++ callbacks
    static void lumaShim(const crl::multisense::image::Header& iHeader,
                         void* iUserData) {
      reinterpret_cast<ImageData*>(iUserData)->lumaCallback(iHeader);
    }
    static void chromaShim(const crl::multisense::image::Header& iHeader,
                           void* iUserData) {
      reinterpret_cast<ImageData*>(iUserData)->chromaCallback(iHeader);
    }
    static void rectShim(const crl::multisense::image::Header& iHeader,
                         void* iUserData) {
      reinterpret_cast<ImageData*>(iUserData)->rectCallback(iHeader);
    }

    void lumaCallback(const crl::multisense::image::Header& iHeader) {
      const int numPixels = iHeader.width*iHeader.height;
      memcpy(mYuv.data, iHeader.imageDataP, numPixels);
      mLumaFrameId = iHeader.frameId;
    }

    void chromaCallback(const crl::multisense::image::Header& iHeader) {
      if (iHeader.frameId != mLumaFrameId) return;
      const int64_t utime = iHeader.timeSeconds*1e6 + iHeader.timeMicroSeconds;
      const int numLumaPixels = mWidth*mHeight;
      const int numChromaPixels = iHeader.width*iHeader.height;
      memcpy(mYuv.data + numLumaPixels, iHeader.imageDataP, numChromaPixels*2);
      cv::cvtColor(mYuv, mRgbRaw, CV_YUV420sp2RGB);
      cv::remap(mRgbRaw, mRgbRectified, mCalibrationMapX, mCalibrationMapY,
                CV_INTER_LINEAR);
      mImp->pushMessage(mRgbRectified, utime, mMessageQueue);
    }
    
    void rectCallback(const crl::multisense::image::Header& iHeader) {
      const int64_t utime = iHeader.timeSeconds*1e6 + iHeader.timeMicroSeconds;
      const int numPixels = iHeader.width*iHeader.height;
      memcpy(mLuma.data, iHeader.imageDataP, numPixels);
      mImp->pushMessage(mLuma, utime, mMessageQueue);
    }

  };

  // class to synchronize and publish images on its own thread
  struct Publisher {
    Imp* mImp;
    bool mIsRunning;
    std::thread mThread;
    multisense::images_t mOutputMessage;

    Publisher(Imp* iImp) {
      mImp = iImp;
    }

    // find image that matches a given timestamp
    bool findImage(MessageQueue& iQueue, const int64_t iTime,
                   MessageQueue::iterator& oIter) {
      for (oIter = iQueue.begin(); oIter != iQueue.end(); ++oIter) {
        if ((*oIter)->utime == iTime) return true;
      }
      return false;
    }

    void operator()() {
      mIsRunning = true;

      // determine which images to look for and publish
      int leftIdx(-1), rightIdx(-1), dispIdx(-1);
      switch (mImp->mOutputMode) {
      case OutputMode::Left:
        leftIdx = 0;  break;
      case OutputMode::LeftRight:
        leftIdx = 0;  rightIdx = 1;  break;
      case OutputMode::LeftRightDisparity:
        leftIdx = 0;  dispIdx = 1;  rightIdx = 2;  break;
      case OutputMode::LeftDisparity:
      default:
        leftIdx = 0;  dispIdx = 1;  break;
      }
      bool useLeft(leftIdx>=0), useRight(rightIdx>=0), useDisp(dispIdx>=0);

      // set up image types array
      auto& msg = mOutputMessage;
      msg.n_images = useLeft + useRight + useDisp;
      msg.images.resize(msg.n_images);
      msg.image_types.resize(msg.n_images);
      if (useLeft) msg.image_types[leftIdx] = multisense::images_t::LEFT;
      if (useRight) msg.image_types[rightIdx] = multisense::images_t::RIGHT;
      int dispType = multisense::images_t::DISPARITY_ZIPPED;
      if (mImp->mZlibCompression==0) dispType = multisense::images_t::DISPARITY;
      if (useDisp) msg.image_types[dispIdx] = dispType;

      // loop until stopped
      while (mIsRunning) {

        // wait for new data to be ready
        std::unique_lock<std::mutex> lock(mImp->mDataMutex);
        mImp->mDataCondition.wait_for(lock, std::chrono::milliseconds(100));

        // copy queues safely
        MessageQueue leftQueue, rightQueue, dispQueue;
        {
          std::unique_lock<std::mutex> lock(mImp->mQueueMutex);
          leftQueue = mImp->mLeftData->mMessageQueue;
          rightQueue = mImp->mRightData->mMessageQueue;
          dispQueue = mImp->mDisparityQueue;
        }

        // for each left image, find corresponding images and publish
        MessageQueue toBeDeleted;
        for (auto leftImage : leftQueue) {
          int64_t utime = leftImage->utime;
 
          // try to find disparity image and right image, if necessary
          MessageQueue::iterator dispIter, rightIter;
          if (useDisp && !findImage(dispQueue, utime, dispIter)) continue;
          if (useRight && !findImage(rightQueue, utime, rightIter)) continue;

          // create output message
          if (useLeft) {
            msg.images[leftIdx] = *leftImage;
            toBeDeleted.push_back(leftImage);
          }
          if (useDisp) {
            msg.images[dispIdx] = **dispIter;
            toBeDeleted.push_back(*dispIter);
            dispQueue.erase(dispIter);
          }
          if (useRight) {
            msg.images[rightIdx] = **rightIter;
            toBeDeleted.push_back(*rightIter);
            rightQueue.erase(rightIter);
          }
          msg.utime = utime;

          // publish message
          mImp->mCamera->mPublishLcm->publish
            (mImp->mCamera->mOutputChannel, &msg);
          std::cout << "published " << msg.n_images << " images at " <<
            msg.utime << std::endl;
        }

        // delete items that have been published
        {
          std::unique_lock<std::mutex> lock(mImp->mQueueMutex);
          for (auto image : toBeDeleted) {
            mImp->mLeftData->mMessageQueue.remove(image);
            mImp->mRightData->mMessageQueue.remove(image);
            mImp->mDisparityQueue.remove(image);
          }
        }
      }
    }
  };


  static const crl::multisense::DataSource kAllImageSources =
    (crl::multisense::Source_Luma_Left            |
     crl::multisense::Source_Luma_Right           |
     crl::multisense::Source_Luma_Rectified_Left  |
     crl::multisense::Source_Luma_Rectified_Right |
     crl::multisense::Source_Chroma_Left          |
     crl::multisense::Source_Chroma_Right         |
     crl::multisense::Source_Raw_Right            |
     crl::multisense::Source_Disparity);

  static const int kMaxQueueLength = 30;


  Camera* mCamera;

  // actual requested image size
  int mImageWidth;
  int mImageHeight;

  // size at which camera calibration is valid
  int mCalibratedWidth;
  int mCalibratedHeight;

  std::vector<uint8_t> mCompressedDispBuf;
  std::shared_ptr<ImageData> mLeftData;
  std::shared_ptr<ImageData> mRightData;
  cv::Mat mDisparityImage;
  MessageQueue mDisparityQueue;
  std::shared_ptr<Publisher> mPublisher; 
  std::mutex mQueueMutex;
  std::mutex mDataMutex;
  std::condition_variable mDataCondition;

  OutputMode mOutputMode;   // which images to output
  int mDesiredWidth;        // target image capture width in pixels
  int mZlibCompression;     // level of zlib compression (0-9)
  int mJpegQuality;         // jpeg compression quality (0-100)

  Imp(Camera* iCamera) {
    mCamera = iCamera;
    mImageWidth = mImageHeight = 0;
    mCalibratedWidth = mCalibratedHeight = 0;

    // set some defaults
    mOutputMode = OutputMode::LeftDisparity;
    mDesiredWidth = 1024;
    mZlibCompression = 1;
    mJpegQuality = 75;

    mLeftData.reset(new ImageData(this));
    mRightData.reset(new ImageData(this));

    mPublisher.reset(new Publisher(this));
  }

  bool getConfig(crl::multisense::image::Config& oConfig) {
    auto status = mCamera->mChannel->getImageConfig(oConfig);
    if (crl::multisense::Status_Ok != status) {
      std::cerr << "failed to query image config (" <<
        mCamera->mChannel->statusString(status) << ")" << std::endl;
      return false;
    }
    return true;
  }

  bool setConfig(const crl::multisense::image::Config& iConfig) {
    auto status = mCamera->mChannel->setImageConfig(iConfig);
    if (crl::multisense::Status_Ok != status) {
      std::cerr << "failed to set image config (" <<
        mCamera->mChannel->statusString(status) << ")" << std::endl;
      return false;
    }
    return true;
  }

  // search for desired image resolution mode
  bool setImageSize() {
    std::vector<crl::multisense::system::DeviceMode> modes;
    crl::multisense::system::DeviceMode bestMode;
    int bestWidthDelta = 1000000;
    int maxHeight = 0;
    int maxDisparities = 0;
    mCalibratedWidth = 0;
    auto channel = mCamera->mChannel;
    if (crl::multisense::Status_Ok != channel->getDeviceModes(modes)) {
      std::cerr << "failed to get device image modes" << std::endl;
      return false;
    }
    else {
      for (auto mode : modes) {
        int delta = std::abs((int)mode.width - mDesiredWidth);
        if (delta <= bestWidthDelta) {
          if (mode.height >= maxHeight) {
            if (mode.disparities > maxDisparities) {
              bestMode = mode;
              bestWidthDelta = delta;
              maxHeight = mode.height;
            }
          }
        }
        // this assumes that the largest image size was used for calibration
        if (mode.width > mCalibratedWidth) {
          mCalibratedWidth = mode.width;
          mCalibratedHeight = mode.height;
        }
      }
      mImageWidth = bestMode.width;
      mImageHeight = bestMode.height;
      std::cout << "selected image size " << mImageWidth << "x" <<
        mImageHeight << std::endl;
    }
    
    // set appropriate resolution
    crl::multisense::image::Config config;
    if (crl::multisense::Status_Ok != channel->getImageConfig(config)) {
      std::cerr << "failed to query sensor configuration" << std::endl;
      return false;
    }
    if ((config.width() != mImageWidth) || (config.height() != mImageHeight)) {
      config.setResolution(mImageWidth, mImageHeight);
      if (crl::multisense::Status_Ok != channel->setImageConfig(config)) {
        std::cerr << "failed to set sensor configuration" << std::endl;
        return false;
      }
    }

    return true;
  }

  bool allocateBuffers() {
    mCompressedDispBuf.resize(mImageWidth*mImageHeight*2);

    auto channel = mCamera->mChannel;
    crl::multisense::image::Calibration calib;
    if (crl::multisense::Status_Ok != channel->getImageCalibration(calib)) {
      std::cerr << "failed to query image calibration" << std::endl;
      return false;
    }
    mLeftData->allocate(mImageWidth, mImageHeight, calib.left);
    mRightData->allocate(mImageWidth, mImageHeight, calib.right);
    mDisparityImage = cv::Mat(mImageHeight, mImageWidth, CV_16UC1);

    return true;
  }

  // start multisense streams and register appropriate callbacks
  bool addCallbacks() {
    auto channel = mCamera->mChannel;
    switch (mOutputMode) {
    case OutputMode::LeftRight:
      channel->startStreams(crl::multisense::Source_Luma_Rectified_Left |
                            crl::multisense::Source_Luma_Rectified_Right);
      channel->addIsolatedCallback(&ImageData::rectShim,
                                   crl::multisense::Source_Luma_Rectified_Left,
                                   mLeftData.get());
      channel->addIsolatedCallback(&ImageData::rectShim,
                                   crl::multisense::Source_Luma_Rectified_Right,
                                   mRightData.get());
      break;
    case OutputMode::Left:
      channel->startStreams(crl::multisense::Source_Luma_Left |
                            crl::multisense::Source_Chroma_Left);
      channel->addIsolatedCallback(&ImageData::lumaShim,
                                   crl::multisense::Source_Luma_Left,
                                   mLeftData.get());
      channel->addIsolatedCallback(&ImageData::chromaShim,
                                   crl::multisense::Source_Chroma_Left,
                                   mLeftData.get());
      break;
    case OutputMode::LeftRightDisparity:
      channel->startStreams(crl::multisense::Source_Luma_Left |
                            crl::multisense::Source_Chroma_Left |
                            crl::multisense::Source_Luma_Rectified_Right |
                            crl::multisense::Source_Disparity);
      channel->addIsolatedCallback(&ImageData::lumaShim,
                                   crl::multisense::Source_Luma_Left,
                                   mLeftData.get());
      channel->addIsolatedCallback(&ImageData::chromaShim,
                                   crl::multisense::Source_Chroma_Left,
                                   mLeftData.get());
      channel->addIsolatedCallback(&ImageData::rectShim,
                                   crl::multisense::Source_Luma_Rectified_Right,
                                   mRightData.get());
      channel->addIsolatedCallback(&Imp::disparityShim,
                                   crl::multisense::Source_Disparity, this);
      break;
    case OutputMode::LeftDisparity:
    default:
      channel->startStreams(crl::multisense::Source_Luma_Left |
                            crl::multisense::Source_Chroma_Left |
                            crl::multisense::Source_Disparity);
      channel->addIsolatedCallback(&ImageData::lumaShim,
                                   crl::multisense::Source_Luma_Left,
                                   mLeftData.get());
      channel->addIsolatedCallback(&ImageData::chromaShim,
                                   crl::multisense::Source_Chroma_Left,
                                   mLeftData.get());
      channel->addIsolatedCallback(&Imp::disparityShim,
                                   crl::multisense::Source_Disparity, this);
      break;
    }

    return true;
  }

  // create lcm message from raw image input
  void pushMessage(const cv::Mat& iImage, const int64_t iTime,
                   MessageQueue& ioQueue) {
    std::shared_ptr<multisense::image_t> msg(new multisense::image_t());
    const int rowSizeBytes = iImage.cols*iImage.elemSize();
    const int imageSizeBytes = rowSizeBytes*iImage.rows;
    msg->width = iImage.cols;
    msg->height = iImage.rows;
    msg->nmetadata = 0;
    msg->utime = iTime;
    msg->row_stride = rowSizeBytes;
    msg->pixelformat = multisense::image_t::PIXEL_FORMAT_GRAY;

    bool doJpeg = (mJpegQuality<100) &&
      ((iImage.type() == CV_8UC1) || (iImage.type() == CV_8UC3));
    bool doZlib = (mZlibCompression>0) && (iImage.type() == CV_16UC1);

    // jpeg compress (for left and/or right images)
    if (doJpeg) {
      std::vector<int> params;
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(mJpegQuality);
      if (!cv::imencode(".jpg", iImage, msg->data, params)) {
        std::cerr << "Error encoding jpeg image!" << std::endl;
        return;
      }
      msg->pixelformat = multisense::image_t::PIXEL_FORMAT_MJPEG; 
    }

    // zlib compress (for depth images)
    else if (doZlib) {
      int uncompressedSize = imageSizeBytes;
      unsigned long compressedSize = mCompressedDispBuf.size();
      compress2(mCompressedDispBuf.data(), &compressedSize,
                iImage.data, uncompressedSize, Z_BEST_SPEED);
      msg->data.resize(compressedSize);
      memcpy(msg->data.data(), mCompressedDispBuf.data(), compressedSize);
    }

    // no compression, just copy data
    else {
      if (iImage.channels() == 3) {
        msg->pixelformat = multisense::image_t::PIXEL_FORMAT_RGB;
      }
      msg->data.resize(imageSizeBytes);
      memcpy(msg->data.data(), iImage.data, imageSizeBytes);
    }

    msg->size = msg->data.size();

    // push data onto work queue
    {
      std::unique_lock<std::mutex> lock(mQueueMutex);
      ioQueue.push_back(msg);
      while (ioQueue.size() > kMaxQueueLength) {
        ioQueue.pop_front();
        std::cout << "warning: dropped image" << std::endl;
      }
    }

    // notify waiting thread that new data is ready
    mDataCondition.notify_one();
  }

  // c-style shim for c++ callback
  static void disparityShim(const crl::multisense::image::Header& iHeader,
                            void* iUserData) {
    reinterpret_cast<Imp*>(iUserData)->disparityCallback(iHeader);
  }

  void disparityCallback(const crl::multisense::image::Header& iHeader) {
    const int64_t utime = iHeader.timeSeconds*1e6 + iHeader.timeMicroSeconds;
    const int sizeBytes = iHeader.width*iHeader.height*2;
    memcpy(mDisparityImage.data, iHeader.imageDataP, sizeBytes);
    pushMessage(mDisparityImage, utime, mDisparityQueue);
  }
  
};

Camera::
Camera(crl::multisense::Channel* iChannel) : Sensor(iChannel) {
  mImp.reset(new Imp(this));

  // set some defaults
  setFrameRate(5);
  setAutoExposure(true);
}

Camera::
~Camera() {
  stop();
}

bool Camera::
setOutputMode(const OutputMode iMode) {
  mImp->mOutputMode = iMode;
  return true;
}

bool Camera::
setDesiredWidth(const int iPixels) {
  mImp->mDesiredWidth = iPixels;
  return true;
}

bool Camera::
setJpegQuality(const int iPercent) {
  if ((iPercent < 0) || (iPercent > 100)) return false;
  mImp->mJpegQuality = iPercent;
  return true;
}

bool Camera::
setZlibCompression(const int iLevel) {
  mImp->mZlibCompression = iLevel;
  return true;
}

bool Camera::
setFrameRate(const float iFramesPerSecond) {
  crl::multisense::image::Config config;
  if (!mImp->getConfig(config)) return false;
  config.setFps(iFramesPerSecond);
  if (!mImp->setConfig(config)) return false;
  return true;
}

bool Camera::
setGainFactor(const float iFactor) {
  crl::multisense::image::Config config;
  if (!mImp->getConfig(config)) return false;
  config.setGain(iFactor);
  if (!mImp->setConfig(config)) return false;
  return true;
}

bool Camera::
setExposureTime(const int iMicroSeconds) { 
  crl::multisense::image::Config config;
  if (!mImp->getConfig(config)) return false;
  config.setExposure(iMicroSeconds);
  if (!mImp->setConfig(config)) return false;
  return true;
} 

bool Camera::
setAutoExposure(const bool iVal) {
  crl::multisense::image::Config config;
  if (!mImp->getConfig(config)) return false;
  config.setAutoExposure(iVal);
  config.setAutoWhiteBalance(iVal);
  if (!mImp->setConfig(config)) return false;
  return true;
}

bool Camera::
start() {
  // initialize buffers and data structures
  if (!mImp->setImageSize()) return false;
  if (!mImp->allocateBuffers()) return false;
  if (!mImp->addCallbacks()) return false;

  // start publish thread
  mImp->mPublisher->mThread = std::thread(std::ref(*mImp->mPublisher));

  std::cout << "started camera" << std::endl;
  return true;
}

bool Camera::
stop() {
  // stop publish thread
  mImp->mPublisher->mIsRunning = false;
  if (mImp->mPublisher->mThread.joinable()) mImp->mPublisher->mThread.join();

  // stop all callbacks
  mChannel->removeIsolatedCallback(&Imp::disparityShim);
  mChannel->removeIsolatedCallback(&Imp::ImageData::lumaShim);
  mChannel->removeIsolatedCallback(&Imp::ImageData::chromaShim);
  mChannel->removeIsolatedCallback(&Imp::ImageData::rectShim);

  // stop multisense streams
  auto status = mChannel->stopStreams(Imp::kAllImageSources);
  if (crl::multisense::Status_Ok != status) {
    std::cerr << "error: cannot stop image streams" << std::endl;
    return false;
  }

  return true;
}
