#include "FusedDepthHandler.hpp"

#include <maps/DepthImage.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/BotWrapper.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>

#include <mutex>

#include <zlib.h>
#include <opencv2/opencv.hpp>

using namespace maps;

struct FusedDepthHandler::Imp {
  std::shared_ptr<BotWrapper> mBotWrapper;
  std::string mDepthChannel;
  std::string mCameraChannel;
  int mMedianFilterSize;
  lcm::Subscription* mDepthSubscription;
  BotCamTrans* mCamTrans;
  bot_core::image_t mCurrentImage;
  int mCurrentFormat;
  Eigen::Isometry3d mCurrentPose;
  Eigen::Matrix3d mCalibMatrix;
  std::mutex mDataMutex;

  Imp() {
    mDepthSubscription = NULL;
    mCamTrans = NULL;
  }

  void start() {
    stop();
    mCurrentImage.utime = 0;
    mCamTrans = bot_param_get_new_camtrans
      (mBotWrapper->getBotParam(), mCameraChannel.c_str());
    if (mCamTrans == NULL) {
      std::cout << "could not start fused depth handler; " <<
        "no calibration data for " << mCameraChannel << std::endl;
      return;
    }

    double k00 = bot_camtrans_get_focal_length_x(mCamTrans);
    double k11 = bot_camtrans_get_focal_length_y(mCamTrans);
    double k01 = bot_camtrans_get_skew(mCamTrans);
    double k02 = bot_camtrans_get_principal_x(mCamTrans);
    double k12 = bot_camtrans_get_principal_y(mCamTrans);
    mCalibMatrix << k00,k01,k02, 0,k11,k12, 0,0,1;
    mDepthSubscription =
      mBotWrapper->getLcm()->subscribe(mDepthChannel, &Imp::onDepth, this);
  }

  void stop() {
    if (mDepthSubscription != NULL) {
      mBotWrapper->getLcm()->unsubscribe(mDepthSubscription);
      mDepthSubscription = NULL;
    }
    if (mCamTrans != NULL) {
      bot_camtrans_destroy(mCamTrans);
      mCamTrans = NULL;
    }
  }

  void onDepth(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const bot_core::images_t* iMessage) {
    for (int i = 0; i < iMessage->n_images; ++i) {
      if ((iMessage->image_types[i] == bot_core::images_t::DEPTH_MM) ||
          (iMessage->image_types[i] == bot_core::images_t::DEPTH_MM_ZIPPED)) {
        std::unique_lock<std::mutex> lock(mDataMutex);
        mCurrentFormat = iMessage->image_types[i];
        mCurrentImage = iMessage->images[i];
        mBotWrapper->getTransform(mCameraChannel, "local", mCurrentPose,
                                  mCurrentImage.utime);
        return;
      }
    }
  }

  DepthImageView::Ptr getLatest() {
    // copy data
    bot_core::image_t img;
    Eigen::Isometry3d pose;
    int format;
    {
      std::unique_lock<std::mutex> lock(mDataMutex);
      img = mCurrentImage;
      pose = mCurrentPose;
      format = mCurrentFormat;
    }

    // check whether data is valid
    if (img.utime == 0) return NULL;

    // get data bytes
    std::vector<uint8_t> bytes;
    if (format == bot_core::images_t::DEPTH_MM_ZIPPED) {
      unsigned long uncompressedSize = img.width*img.height*2;
      bytes.resize(uncompressedSize);
      uncompress(bytes.data(), &uncompressedSize,
                 img.data.data(), img.data.size());
    }
    else {
      bytes = img.data;
    }

    // create depth image
    DepthImage depthImage;
    depthImage.setSize(img.width, img.height);
    depthImage.setOrthographic(false);
    depthImage.setPose(pose.cast<float>());
    depthImage.setCalib(mCalibMatrix.cast<float>());

    // convert from uint16 to float and set data in depth image
    std::vector<float> depths(img.width*img.height);
    uint16_t* raw = (uint16_t*)bytes.data();
    for (int i = 0; i < (int)depths.size(); ++i) {
      depths[i] = (raw[i] == 0) ?
        depthImage.getInvalidValue(DepthImage::TypeDepth) : raw[i]/1e3f;
    }
    depthImage.setData(depths, DepthImage::TypeDepth);

    // create and return wrapper view
    DepthImageView::Ptr view(new DepthImageView());
    view->set(depthImage);
    view->setUpdateTime(img.utime);
    return view;
  }

  DepthImageView::Ptr getLatest(const drc::map_request_t& iRequest) {
    // get latest fused depth map as view
    DepthImageView::Ptr depthView = getLatest();
    if (depthView == NULL) return depthView;

    // convert to point cloud in local frame
    maps::PointCloud::Ptr cloud = depthView->getAsPointCloud();

    // set up transformation
    Eigen::Projective3f projector;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        projector(i,j) = iRequest.transform[i][j];
      }
    }
    Eigen::Isometry3f headToLocal, torsoToLocal;
    if (mBotWrapper->getTransform("utorso", "local", torsoToLocal) &&
        mBotWrapper->getTransform("head", "local", headToLocal)) {
      float theta = atan2(torsoToLocal(1,0), torsoToLocal(0,0));
      Eigen::Matrix3f rotation;
      rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
      headToLocal.linear() = rotation;
      projector = projector*headToLocal.inverse();
    }

    // reproject to depth map according to given specifications
    DepthImageView::Ptr view(new DepthImageView());
    view->setSize(iRequest.width, iRequest.height);
    view->getDepthImage()->setAccumulationMethod
      ((DepthImage::AccumulationMethod)iRequest.accum_type);
    view->setTransform(projector);
    view->set(cloud);

    // filter out bogus returns
    if (mMedianFilterSize > 0) {
      const auto& depthImage = view->getDepthImage();
      DepthImage::Type dataType = DepthImage::TypeDisparity;
      std::vector<float> data = depthImage->getData(dataType);
      cv::Mat depthMat(depthImage->getHeight(), depthImage->getWidth(),
                       CV_32FC1, data.data());
      cv::medianBlur(depthMat, depthMat, mMedianFilterSize);
      depthImage->setData(data, dataType);
    }

    return view;
  }
  
};

FusedDepthHandler::
FusedDepthHandler(const std::shared_ptr<BotWrapper>& iBotWrapper) {
  mImp.reset(new Imp());
  mImp->mBotWrapper = iBotWrapper;
  setMedianFilter(0);
}

FusedDepthHandler::
~FusedDepthHandler() {
  stop();
}

void FusedDepthHandler::
setDepthChannel(const std::string& iChannel) {
  mImp->mDepthChannel = iChannel;
}

void FusedDepthHandler::
setCameraChannel(const std::string& iChannel) {
  mImp->mCameraChannel = iChannel;
}

void FusedDepthHandler::
setMedianFilter(const int iKernelRadius) {
  mImp->mMedianFilterSize = iKernelRadius*2+1;
}

void FusedDepthHandler::
start() {
  return mImp->start();
}

void FusedDepthHandler::
stop() {
  return mImp->stop();
}

DepthImageView::Ptr FusedDepthHandler::
getLatest() const {
  return mImp->getLatest();
}

DepthImageView::Ptr FusedDepthHandler::
getLatest(const drc::map_request_t& iRequest) const {
  return mImp->getLatest(iRequest);
}
