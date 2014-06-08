#include "LidarAccumulator.hpp"

#include <thread>
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include <opencv2/opencv.hpp>
#include <zlib.h>

#include <drc_utils/PointerUtils.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <drc_utils/ThreadSafeQueue.hpp>

#include <drcvision/fovision.hpp>
#include <drcvision/voconfig.hpp>

#include <lcmtypes/multisense/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>

using namespace maps;

class LidarAccumulator::Helper {
private:
  struct Pose {
    int64_t mTimestamp;
    Eigen::Isometry3d mTransform;
    bool mGood;
    Pose() : mGood(true) {}
  };

  struct Scan {
    int64_t mTimestamp;
    double mThetaInit;
    double mThetaStep;
    double mSpindleAngle;
    std::vector<double> mRanges;

    // pose to local as measured by robot state estimate
    Pose mPoseBeginToLocal;
    Pose mPoseEndToLocal;

    // pose wrt camera
    Pose mPoseBeginToCamera;
    Pose mPoseEndToCamera;

    // pose to visual odometry reference frame (filled in by algo)
    Eigen::Isometry3d mPoseBeginToVoReference;
    Eigen::Isometry3d mPoseEndToVoReference;
    bool mVoPoseGood;

    typedef std::shared_ptr<Scan> Ptr;
  };

  typedef std::shared_ptr<multisense::images_t> ImagesPtr;

  static constexpr double kPi = acos(-1);
  static constexpr int kMaxVoPoses = 1000;
  static constexpr int kMaxScans = 5000;
  static constexpr int kMaxCameraQueue = 20;

public:
  Helper() {
    mCameraSubscription = NULL;
    mLidarSubscription = NULL;
    setCameraChannel("CAMERA");
    setLidarChannel("SCAN");
    setRangeLimits(-1,-1);
  }

  ~Helper() {
    stop();
  }

  bool setCameraChannel(const std::string& iChannel) {
    if (mIsRunning) return false;
    mCameraChannel = iChannel;
    mCameraLeftName = iChannel + "_LEFT";
    return true;
  }

  bool setLidarChannel(const std::string& iChannel) {
    if (mIsRunning) return false;
    mLidarChannel = iChannel;
    return true;
  }

  bool setBotWrapper(const std::shared_ptr<drc::BotWrapper>& iBotWrapper) {
    if (mIsRunning) return false;
    mBotWrapper = iBotWrapper;
    return true;
  }

  bool setRangeLimits(const float iRangeMin, const float iRangeMax) {
    // TODO: if we accumulate while processing, uncomment this
    // if (mIsRunning) return false;
    mRangeMin = iRangeMin;
    mRangeMax = iRangeMax;
    return true;
  }

  bool start() {
    if (mIsRunning) return false;

    // clear all queues
    mVoPoses.clear();
    mScans.clear();
    mCameraWorkQueue.clear();
    mScanWorkQueue.clear();
    mCameraWorkQueue.setMaxSize(kMaxCameraQueue);

    // set up fovis object
    auto boostLcm = drc::PointerUtils::boostPtr(mLcm);
    auto config =
      new voconfig::KmclConfiguration(mBotWrapper->getBotParam(), mCameraChannel);
    boost::shared_ptr<fovis::StereoCalibration>
      calib(config->load_stereo_calibration());
    mVo.reset(new FoVision(boostLcm, calib));

    // set up subscriptions
    mLcm = mBotWrapper->getLcm();
    mCameraSubscription =
      mLcm->subscribe(mCameraChannel, &Helper::onCamera, this);
    mLidarSubscription = mLcm->subscribe(mLidarChannel, &Helper::onScan, this);

    // get lidar config values
    if (!mBotWrapper->get("planar_lidars." + mLidarChannel + ".frequency",
                          mScanFrequency)) mScanFrequency = 40;
    if (mRangeMin < 0) {
      if (!mBotWrapper->get("planar_lidars." + mLidarChannel + ".min_range",
                            mRangeMin)) mRangeMin = 0.3;
    }
    if (mRangeMax < 0) {
      if (!mBotWrapper->get("planar_lidars." + mLidarChannel + ".max_range",
                            mRangeMax)) mRangeMax = 29.0;
    }

    // now running; start processing threads
    mIsRunning = true;
    mCameraThread = std::thread(&Helper::processCameraLoop, this);
    mScanThread = std::thread(&Helper::processScanLoop, this);

    return true;
  }

  bool stop() {
    if (!mIsRunning) return false;
    if (mLcm != NULL) {
      if (mCameraSubscription != NULL) mLcm->unsubscribe(mCameraSubscription);
      if (mLidarSubscription != NULL) mLcm->unsubscribe(mLidarSubscription);
    }
    mCameraSubscription = mLidarSubscription = NULL;
    mIsRunning = false;
    mCameraWorkQueue.unblock();
    if (mCameraThread.joinable()) mCameraThread.join();
    if (mScanThread.joinable()) mScanThread.join();
    return true;
  }

  bool isRunning() const {
    return mIsRunning;
  }

  bool getPointCloud(const double iRevolutions, const double iStartAngle,
                     std::vector<Eigen::Vector3f>& oCloud) {
    oCloud.clear();

    // create scan list
    int totalPoints = 0;
    std::list<Scan::Ptr> scans;
    bool foundAllRequestedRevolutions = false;

    bool started = (iStartAngle < 0);
    
    {
      std::unique_lock<std::mutex> lock(mScansMutex);
      if (mScans.size() == 0) return false;
      double prevAngle = mScans.back()->mSpindleAngle;
      int signInit = computeSign(prevAngle,iStartAngle);
      double targetAngle = iRevolutions*2*kPi;
      double totalAngle = 0;
      for (auto iter = mScans.rbegin(); iter != mScans.rend(); ++iter) {
        double angle = (*iter)->mSpindleAngle;

        // if not started yet, check for change of sign
        if (!started) {
          int signCur = computeSign(angle,iStartAngle);
          if (signCur != signInit) started = true;
        }

        // otherwise start accumulating
        else {
          if (!(*iter)->mVoPoseGood) return false;
          double deltaAngle = std::abs(angle-prevAngle);
          prevAngle = angle;
          if (deltaAngle > kPi) deltaAngle = 2*kPi - deltaAngle;
          totalAngle += deltaAngle;
          scans.push_back(*iter);
          totalPoints += (*iter)->mRanges.size();
          if (totalAngle >= targetAngle) {
            foundAllRequestedRevolutions = true;
            break;
          }
        }
      }
    }
    if (!foundAllRequestedRevolutions) return false;

    // compute transform from vo reference frame to current local frame
    Eigen::Isometry3d voToCurCam = mVoPoses.back().mTransform.inverse();
    Eigen::Isometry3d curCamToLocal;
    mBotWrapper->getTransform(mCameraLeftName, "local", curCamToLocal);
    Eigen::Isometry3d voReferenceToLocal = curCamToLocal*voToCurCam;

    // loop over scans
    for (auto scan : scans) {

      // transform poses to local frame
      auto pose1 = voReferenceToLocal*scan->mPoseBeginToVoReference;
      auto pose2 = voReferenceToLocal*scan->mPoseEndToVoReference;

      // interpolate returns in scan
      const int n = scan->mRanges.size();
      for (int j = 0; j < n; ++j) {
        double r = scan->mRanges[j];
        if ((r < mRangeMin) || (r > mRangeMax)) continue;
        double alpha = (double)j/(n-1);
        double theta = scan->mThetaInit + j*scan->mThetaStep;
        Eigen::Vector3d p(r*cos(theta), r*sin(theta), 0);
        Eigen::Vector3d pt;
        pt = applyInterpolated(p, pose1, pose2, alpha);
        oCloud.push_back(pt.cast<float>());
      }
    }

    // TODO: make this a pcl cloud?
    return true;
  }

  int computeSign(const double iAngle, const double iRefAngle) {
    double delta = iAngle - iRefAngle;
    while (std::abs(delta) > kPi) {
      if (delta < 0) delta += kPi;
      else delta -= kPi;
    }
    return (delta>0) ? 1 : ((delta<0) ? -1 : 0);
  }

  void onScan(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
              const bot_core::planar_lidar_t* iMessage) {
    if (!mIsRunning) return;

    const double dt = (iMessage->radstep/(2*kPi))/mScanFrequency;
    const int n = iMessage->nranges;

    // set basic scan data fields
    std::shared_ptr<Scan> scan(new Scan());
    scan->mTimestamp = iMessage->utime;
    scan->mThetaInit = iMessage->rad0;
    scan->mThetaStep = iMessage->radstep;
    scan->mRanges.reserve(iMessage->ranges.size());
    scan->mVoPoseGood = false;
    for (auto r : iMessage->ranges) {
      scan->mRanges.push_back(r);
    }

    // get scan pose with respect to local frame at end of scan
    scan->mPoseEndToLocal.mTimestamp = iMessage->utime;
    mBotWrapper->getTransform(iChannel,"local",
                              scan->mPoseEndToLocal.mTransform,
                              iMessage->utime);

    // get scan pose with respect to camera at end of scan
    scan->mPoseEndToCamera.mTimestamp = iMessage->utime;
    mBotWrapper->getTransform(iChannel, mCameraLeftName,
                              scan->mPoseEndToCamera.mTransform,
                              iMessage->utime);

    // get scan pose with respect to local frame at start of scan
    scan->mPoseBeginToLocal.mTimestamp =
      iMessage->utime - int64_t(dt*(n-1)*1e6 + 0.5);
    mBotWrapper->getTransform(iChannel,"local",
                              scan->mPoseBeginToLocal.mTransform,
                              iMessage->utime);

    // get scan pose with respect to camera at start of scan
    scan->mPoseBeginToCamera.mTimestamp =
      iMessage->utime - int64_t(dt*(n-1)*1e6 + 0.5);
    mBotWrapper->getTransform(iChannel, mCameraLeftName,
                              scan->mPoseBeginToCamera.mTransform,
                              iMessage->utime);

    // get spindle angle
    Eigen::Isometry3d spindleTransform;
    mBotWrapper->getTransform("POST_SPINDLE", "PRE_SPINDLE",spindleTransform,
                              iMessage->utime);
    scan->mSpindleAngle = std::atan2(spindleTransform(1,0),spindleTransform(0,0));

    // add to scan work queue
    {
      std::unique_lock<std::mutex> lock(mScanWorkQueueMutex);
      mScanWorkQueue.push_back(scan);
    }
  }

  void onCamera(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                const multisense::images_t* iMessage) {
    if (!mIsRunning) return;
    ImagesPtr images(new multisense::images_t(*iMessage));
    mCameraWorkQueue.push(images);
    if (mCameraWorkQueue.getSize() > kMaxCameraQueue) {
      std::cout << "**warning: camera work queue is full" << std::endl;
    }
  }

  void processCameraLoop() {
    while (mIsRunning) {
      ImagesPtr images;
      if (!mCameraWorkQueue.waitForData(images)) continue;
      processCamera(*images);
    }
  }

  void processScanLoop() {
    while (mIsRunning) {
      std::unique_lock<std::mutex> lock(mVoPoseReadyMutex);
      mVoPoseReadyCondition.wait_for(lock, std::chrono::milliseconds(500));
      
      // copy work queue
      std::list<Scan::Ptr> workQueue;
      {
        std::unique_lock<std::mutex> lock(mScanWorkQueueMutex);
        workQueue = mScanWorkQueue;
      }

      // iterate over work queue
      for (auto scan : workQueue) {
        if (!assignPoses(scan)) break;
        
        {  // remove this scan from the work queue
          std::unique_lock<std::mutex> lock(mScanWorkQueueMutex);
          mScanWorkQueue.remove(scan);
        }
        {  // add this scan to the valid scans list
          std::unique_lock<std::mutex> lock(mScansMutex);
          mScans.push_back(scan);
          while (mScans.size() > kMaxScans)  mScans.pop_front();
        }
      }
    }
  }

  void processCamera(const multisense::images_t& iMessage) {
    auto t1 = std::chrono::high_resolution_clock::now();
    // decode images
    cv::Mat leftImage;
    cv::Mat dispImage;
    for (int i = 0; i < iMessage.n_images; ++i) {
      const bot_core::image_t& img = iMessage.images[i];
      const int w = img.width;
      const int h = img.height;
      auto imgType = iMessage.image_types[i];

      // left image
      if (imgType == multisense::images_t::LEFT) {
        switch (img.pixelformat) {
        case bot_core::image_t::PIXEL_FORMAT_MJPEG:
          leftImage = cv::imdecode(cv::Mat(img.data), -1); break;
        case bot_core::image_t::PIXEL_FORMAT_GRAY:
          leftImage = cv::Mat(h, w, CV_8UC1, (void*)img.data.data()); break;
        case bot_core::image_t::PIXEL_FORMAT_RGB:
          leftImage = cv::Mat(h, w, CV_8UC3, (void*)img.data.data()); break;
        default:
          std::cout << "error: unknown pixel format" << std::endl; break;
        }
        if (leftImage.channels() == 3) {
          cv::cvtColor(leftImage, leftImage, CV_RGB2GRAY);
        }
      }

      // disparity image
      if ((imgType == multisense::images_t::DISPARITY) ||
          (imgType == multisense::images_t::DISPARITY_ZIPPED)) {
        if (img.data.size() != 2*w*h) {
          std::vector<uint8_t> buf(w*h*2);
          unsigned long len = buf.size();
          uncompress(buf.data(), &len, img.data.data(), img.data.size());
          cv::Mat(h, w, CV_16UC1, (void*)buf.data()).
            convertTo(dispImage, CV_32F, 1.0/16);
        }
      }
    }

    // do fovis
    Pose pose;
    pose.mTimestamp = iMessage.utime;
    mVo->doOdometry(leftImage.data, (float*)dispImage.data, iMessage.utime);
    auto status = mVo->getEstimateStatus();
    if ((status != fovis::SUCCESS) && (status != fovis::NO_DATA)) {
      std::cout << "fovis error: " << status;
      switch (status) {
      case fovis::INSUFFICIENT_INLIERS: std::cout << "too few inliers"; break;
      default: std::cout << "other"; break;
      }
      std::cout << std::endl;
      pose.mGood = false;
    }
    pose.mTransform = mVo->getPose();

    // store camera pose
    auto t2 = std::chrono::high_resolution_clock::now();
    auto num = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);
    {
      std::unique_lock<std::mutex> lock(mVoPosesMutex);
      mVoPoses.push_back(pose);
      while (mVoPoses.size() > kMaxVoPoses) mVoPoses.pop_front();
    }
    mVoPoseReadyCondition.notify_one();
    std::cout << "processed image " << num.count()/1e6 << " sec" << std::endl;
  }

  int findPose(const std::deque<Pose>& iPoses, const int64_t iTimestamp,
                const int iStartIdx) {
    int idx = iStartIdx;
    while (iPoses[idx].mTimestamp > iTimestamp) --idx;
    return idx;
  }

  bool assignPoses(const Scan::Ptr& ioScan) {
    int64_t time1 = ioScan->mPoseBeginToCamera.mTimestamp;
    int64_t time2 = ioScan->mPoseEndToCamera.mTimestamp;
    Eigen::Isometry3d voPose1, voPose2;

    std::vector<Pose> voPoses;
    bool good = true;
    {
      std::unique_lock<std::mutex> lock(mVoPosesMutex);
      if (mVoPoses.size() == 0) return false;
      if (mVoPoses.back().mTimestamp <= time2) return false;
      if (mVoPoses.front().mTimestamp > time1) return true;
      int idx = findPose(mVoPoses, time2, mVoPoses.size()-1);
      good = good && mVoPoses[idx].mGood && mVoPoses[idx+1].mGood;
      voPose2 = interpolate(mVoPoses[idx], mVoPoses[idx+1], time2);
      idx = findPose(mVoPoses, time1, idx+1);
      good = good && mVoPoses[idx].mGood && mVoPoses[idx+1].mGood;
      voPose1 = interpolate(mVoPoses[idx], mVoPoses[idx+1], time1);
    }
    ioScan->mPoseBeginToVoReference = voPose1*ioScan->mPoseBeginToCamera.mTransform;
    ioScan->mPoseEndToVoReference = voPose2*ioScan->mPoseEndToCamera.mTransform;
    ioScan->mVoPoseGood = good;
    return true;
  }

  Eigen::Isometry3d interpolate(const Pose& iPose1, const Pose& iPose2,
                                const int64_t iTimestamp) {
    double alpha = (double)(iTimestamp - iPose1.mTimestamp) /
      (iPose2.mTimestamp - iPose1.mTimestamp);
    Eigen::Quaterniond q1(iPose1.mTransform.rotation());
    Eigen::Quaterniond q2(iPose2.mTransform.rotation());
    Eigen::Quaterniond q = q1.slerp(alpha, q2);
    Eigen::Vector3d p1 = iPose1.mTransform.translation();
    Eigen::Vector3d p2 = iPose2.mTransform.translation();
    Eigen::Vector3d p = p1*(1-alpha) + p2*alpha;
    Eigen::Isometry3d xform = Eigen::Isometry3d::Identity();
    xform.linear() = q.matrix();
    xform.translation() = p;
    return xform;
  }

  Eigen::Vector3d applyInterpolated(const Eigen::Vector3d& iPoint,
                                    const Eigen::Isometry3d& iPose1,
                                    const Eigen::Isometry3d& iPose2,
                                    const double iAlpha) {
    Eigen::Vector3d pos1 = iPose1.translation();
    Eigen::Vector3d pos2 = iPose2.translation();
    Eigen::Quaterniond q1(iPose1.rotation());
    Eigen::Quaterniond q2(iPose2.rotation());
    Eigen::Vector3d pos = pos1*(1-iAlpha) + pos2*iAlpha;
    Eigen::Quaterniond q = q1.slerp(iAlpha, q2);
    return q*iPoint + pos;
  }
  

private:
  std::string mCameraChannel;
  std::string mCameraLeftName;
  std::string mLidarChannel;
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  double mRangeMin;
  double mRangeMax;
  double mScanFrequency;

  lcm::Subscription* mCameraSubscription;
  lcm::Subscription* mLidarSubscription;

  bool mIsRunning;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<FoVision> mVo;
  std::list<Scan::Ptr> mScans;
  std::deque<Pose> mVoPoses;
  std::list<Scan::Ptr> mScanWorkQueue;
  drc::ThreadSafeQueue<ImagesPtr> mCameraWorkQueue;
  std::thread mCameraThread;
  std::thread mScanThread;
  std::mutex mScanWorkQueueMutex;
  std::mutex mVoPosesMutex;
  std::mutex mScansMutex;
  std::mutex mVoPoseReadyMutex;
  std::condition_variable mVoPoseReadyCondition;
};

LidarAccumulator::
LidarAccumulator() {
  mHelper.reset(new Helper());
}

LidarAccumulator::
~LidarAccumulator() {
}

bool LidarAccumulator::
setCameraChannel(const std::string& iChannel) {
  return mHelper->setCameraChannel(iChannel);
}

bool LidarAccumulator::
setLidarChannel(const std::string& iChannel) {
  return mHelper->setLidarChannel(iChannel);
}

bool LidarAccumulator::
setBotWrapper(const std::shared_ptr<drc::BotWrapper>& iBotWrapper) {
  return mHelper->setBotWrapper(iBotWrapper);
}

bool LidarAccumulator::
setRangeLimits(const float iRangeMin, const float iRangeMax) {
  return mHelper->setRangeLimits(iRangeMin, iRangeMax);
}


bool LidarAccumulator::
start() {
  return mHelper->start();
}

bool LidarAccumulator::
stop() {
  return mHelper->stop();
}

bool LidarAccumulator::
isRunning() const {
  return mHelper->isRunning();
}

bool LidarAccumulator::
getPointCloud(const double iRevolutions, const double iStartAngle,
              std::vector<Eigen::Vector3f>& oCloud) {
  return mHelper->getPointCloud(iRevolutions, iStartAngle, oCloud);
}
