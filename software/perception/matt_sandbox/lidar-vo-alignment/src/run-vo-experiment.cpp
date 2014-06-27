#include <fstream>
#include <ConciseArgs>
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include <opencv2/opencv.hpp>
#include <zlib.h>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <drcvision/fovision.hpp>
#include <drcvision/voconfig.hpp>

#include <lcmtypes/multisense/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>


struct Pose {
  int64_t mTimestamp;
  Eigen::Isometry3d mTransform;
};

struct Scan {
  int64_t mTimestamp;
  double mThetaInit;
  double mThetaStep;
  std::vector<double> mRanges;
  Pose mPoseScan;
  Pose mPoseSpindleBegin;
  Pose mPoseSpindleEnd;
  typedef std::shared_ptr<Scan> Ptr;
};


struct State {
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  int64_t mStartTime;
  int64_t mEndTime;
  double mMinRange;
  double mMaxRange;

  std::vector<Scan::Ptr> mScans;

  std::vector<Pose> mPosesVo;
  std::vector<Pose> mPosesCam;
  std::vector<Eigen::Vector3d> mCloudBefore;
  std::vector<Eigen::Vector3d> mCloudAfter;

  std::shared_ptr<FoVision> mVo;
  Eigen::Isometry3d mWorldToCamera;

  static constexpr double kPi = acos(-1);
  static constexpr double kScanFrequency = 1.0/40;


  void setup() {
    auto lcm = mBotWrapper->getLcm();
    lcm->subscribe("CAMERA", &State::onCamera, this);
    lcm->subscribe("SCAN", &State::onScan, this);

    // set up fovis object
    auto boostLcm = mLcmWrapper->getBoost();
    auto config =
      new voconfig::KmclConfiguration(mBotWrapper->getBotParam(), "CAMERA");
    boost::shared_ptr<fovis::StereoCalibration>
      calib(config->load_stereo_calibration());
    mVo.reset(new FoVision(boostLcm, calib));
    mWorldToCamera = Eigen::Isometry3d::Identity();
  }

  void onCamera(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const multisense::images_t* iMessage) {
    if (iMessage->utime < mStartTime) return;
    if ((mEndTime > 0) && (iMessage->utime > mEndTime)) {
      mLcmWrapper->stopHandleThread();
      return;
    }

    // get pose wrt local frame
    Pose pose;
    pose.mTimestamp = iMessage->utime;
    mBotWrapper->getTransform(iChannel+"_LEFT","local",
                              pose.mTransform, iMessage->utime);
    mPosesCam.push_back(pose);

    // decode images
    cv::Mat leftImage;
    cv::Mat dispImage;
    for (int i = 0; i < iMessage->n_images; ++i) {
      const bot_core::image_t& img = iMessage->images[i];
      const int w = img.width;
      const int h = img.height;
      auto imgType = iMessage->image_types[i];

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
    mVo->doOdometry(leftImage.data, (float*)dispImage.data, iMessage->utime);
    auto status = mVo->getEstimateStatus();
    if ((status != fovis::SUCCESS) && (status != fovis::NO_DATA)) {
      std::cout << "fovis error: " << status;
      switch (status) {
      case fovis::INSUFFICIENT_INLIERS: std::cout << "too few inliers"; break;
      default: std::cout << "other"; break;
      }
      std::cout << std::endl;
    }
    /* TODO: temp?
    Eigen::Isometry3d delta;
    Eigen::MatrixXd cov;
    mVo->getMotion(delta, cov, status);
    mWorldToCamera = mWorldToCamera*delta;
    pose.mTransform = mWorldToCamera.inverse();
    */
    pose.mTransform = mVo->getPose();
    mPosesVo.push_back(pose);
  }

  void onScan(const lcm::ReceiveBuffer* iBuf,
              const std::string& iChannel,
              const bot_core::planar_lidar_t* iMessage) {
    if (iMessage->utime < mStartTime) return;
    if ((mEndTime > 0) && (iMessage->utime > mEndTime)) {
      mLcmWrapper->stopHandleThread();
      return;
    }

    // store pose for later
    const double dt = kScanFrequency*(iMessage->radstep/(2*kPi));
    const int n = iMessage->nranges;
    std::shared_ptr<Scan> scan(new Scan());
    scan->mTimestamp = iMessage->utime;
    mBotWrapper->getTransform(iChannel,"local",scan->mPoseScan.mTransform,
                              iMessage->utime);
    scan->mPoseSpindleEnd.mTimestamp = iMessage->utime;
    mBotWrapper->getTransform(iChannel,"CAMERA_LEFT",
                              scan->mPoseSpindleEnd.mTransform,
                              iMessage->utime);
    scan->mPoseSpindleBegin.mTimestamp =
      iMessage->utime - int64_t(dt*(n-1)*1e6 + 0.5);
    mBotWrapper->getTransform(iChannel,"CAMERA_LEFT",
                              scan->mPoseSpindleBegin.mTransform,
                              scan->mPoseSpindleBegin.mTimestamp);
    scan->mThetaInit = iMessage->rad0;
    scan->mThetaStep = iMessage->radstep;
    scan->mRanges.reserve(iMessage->ranges.size());
    for (auto r : iMessage->ranges) {
      scan->mRanges.push_back(r);
    }

    mScans.push_back(scan);
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

  void integrateScans() {
    // count points
    int totalPoints = 0;
    for (auto scan : mScans) {
      totalPoints += scan->mRanges.size();
    }
    mCloudBefore.reserve(totalPoints);
    mCloudAfter.reserve(totalPoints);

    // adjust vo poses
    Eigen::Isometry3d firstVoToCam =
      mPosesCam[0].mTransform*mPosesVo[0].mTransform.inverse();
    for (auto& pose : mPosesVo) {
      pose.mTransform = firstVoToCam*pose.mTransform;
    }

    // process scans
    int poseIdx = 0;
    for (int i = 0; i < mScans.size(); ++i) {
      auto scan = mScans[i];
      int n = scan->mRanges.size();
      int64_t time1 = scan->mPoseSpindleBegin.mTimestamp;
      int64_t time2 = scan->mPoseSpindleEnd.mTimestamp;

      // interpolate camera poses at the two times
      Eigen::Isometry3d voPose1, voPose2, camPose1, camPose2;
      int poseIdxNew = findPose(mPosesVo, time1, poseIdx);
      if (poseIdxNew == -1) continue;
      if (poseIdxNew == -2) break;
      poseIdx = poseIdxNew;
      voPose1 = interpolate(mPosesVo[poseIdx], mPosesVo[poseIdx+1], time1);
      voPose1 = voPose1*scan->mPoseSpindleBegin.mTransform;
      camPose1 = interpolate(mPosesCam[poseIdx], mPosesCam[poseIdx+1], time1);
      camPose1 = camPose1*scan->mPoseSpindleBegin.mTransform;
      poseIdxNew = findPose(mPosesVo, time2, poseIdx);
      if (poseIdxNew == -1) continue;
      if (poseIdxNew == -2) break;
      poseIdx = poseIdxNew;
      voPose2 = interpolate(mPosesVo[poseIdx], mPosesVo[poseIdx+1], time2);
      voPose2 = voPose2*scan->mPoseSpindleEnd.mTransform;
      camPose2 = interpolate(mPosesCam[poseIdx], mPosesCam[poseIdx+1], time2);
      camPose2 = camPose2*scan->mPoseSpindleEnd.mTransform;

      for (int j = 0; j < n; ++j) {

        // check whether lidar return is valid
        double r = scan->mRanges[j];
        if ((r < mMinRange) || (r > mMaxRange)) continue;

        // interpolate between two camera poses
        double alpha = (double)j/(n-1);
        double theta = scan->mThetaInit + j*scan->mThetaStep;
        Eigen::Vector3d p(r*cos(theta), r*sin(theta), 0);
        Eigen::Vector3d pt;
        pt = applyInterpolated(p, camPose1, camPose2, alpha);
        mCloudBefore.push_back(pt);
        pt = applyInterpolated(p, voPose1, voPose2, alpha);
        mCloudAfter.push_back(pt);
      }
    }
  }

  int findPose(const std::vector<Pose>& iPoses, const int64_t iTimestamp,
               const int iStartIndex) {
    int idx = iStartIndex;
    if (idx >= iPoses.size()-1) return -2;
    if (iTimestamp < iPoses[idx].mTimestamp) return -1;
    while (iTimestamp > iPoses[idx+1].mTimestamp) {
      ++idx;
      if (idx >= iPoses.size()-1) return -2;
    }
    return idx;
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

  void writeCloud(const std::vector<Eigen::Vector3d>& iPoints,
                   const std::string& iFileName) {
    std::ofstream ofs(iFileName);
    for (auto p : iPoints) {
      ofs << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }
    ofs.close();
  }
};



int main(const int iArgc, const char** iArgv) {
  std::string logFileName;
  std::string botParamFileName;
  std::string outputFileBase;
  int64_t startTime = 0;
  int64_t endTime = -1;
  double minRange = 0.1;
  double maxRange = 10;
  /* TODO:

     
     incorporate message to control start, stop, integrate request
     send integrated point cloud as maps cloud or as list of poses
     listen for walking translator output (number of steps remaining in plan)
     make sure to use lidar min and max
     
     make an object for this functionality
       maintain a rolling window of scans, say 2000 long
       each scan also has a point cloud associated with it
       point cloud is only populated when vo can be interpolated
         . so we need a processing queue
         . maybe move the work queue to drc_utils
     . create a branch
     better command line parsing:
       

     integration start, integration stop, vo stop
       lidar scans collected from int start to int stop;
       then all integrated with respect to vo time 1;
       then all transformed from camera at time "vo stop" to local at time "vo stop"
     online operation: can transform scans as we go
       maintain a small queue of scans
       process a scan when its vo cameras are available
       reference everything to vo frame (camera) 
       then at any time can transform a subset of scans directly to local

     integrate scans at whatever time range
     but then keep running until the robot settles
     then propagate the point cloud forward just after settling
       using vo and using raw state
     compare this to the point cloud while standing
     what is the transform we need?
       vo_end to vo_lastscan (or vo_firstscan)
       then cam to local at vo_end time
   */

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv, "<logfile> <configfile> <outdir>");
  opt.add(startTime, "s", "start_time", "log start time");
  opt.add(endTime, "e", "end_time", "log end time");
  opt.add(minRange, "m", "min_range", "min lidar range to include");
  opt.add(maxRange, "x", "max_range", "max lidar range to include");
  opt.parse(logFileName, botParamFileName, outputFileBase);

  // get start time and time range
  std::shared_ptr<lcm::LogFile> logFile(new lcm::LogFile(logFileName, "r"));
  if (!logFile->good()) {
    std::cerr << "cannot open log file " << logFileName << std::endl;
    return -1;
  }
  const lcm::LogEvent* event = logFile->readNextEvent();
  int64_t firstTimestamp = event->timestamp;
  logFile.reset();
  int64_t startTimestamp = firstTimestamp + int64_t(1e6*startTime + 0.5);
  int64_t endTimestamp = (endTime<0) ? -1 :
    firstTimestamp + int64_t(1e6*endTime + 0.5);
  std::cout << "time range " <<
    startTimestamp << " " << endTimestamp << std::endl;

  // open lcm
  std::ostringstream oss;
  int64_t logStartTime = startTimestamp - 1000000;  // for bot frames
  if (logStartTime < firstTimestamp) logStartTime = firstTimestamp;
  oss << "file://" << logFileName << "?speed=0&mode=r&start_timestamp=" <<
    logStartTime;
  std::string url = oss.str();
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM(url));
  if (!lcm->good()) {
    std::cerr << "cannot open log file " << url << std::endl;
    return -1;
  }
  std::cout << "opened url " << url << std::endl;

  // set up bot objects
  BotParam* param = bot_param_new_from_file(botParamFileName.c_str());
  std::shared_ptr<drc::BotWrapper> botWrapper(new drc::BotWrapper(lcm,param));

  // create lcm wrapper
  std::shared_ptr<drc::LcmWrapper> lcmWrapper(new drc::LcmWrapper(lcm));

  // set up state
  std::shared_ptr<State> state(new State());
  state->mBotWrapper = botWrapper;
  state->mLcmWrapper = lcmWrapper;
  state->mStartTime = startTimestamp;
  state->mEndTime = endTimestamp;
  state->mMinRange = minRange;
  state->mMaxRange = maxRange;
  state->setup();

  // run to gather all data
  lcmWrapper->startHandleThread(true);
  std::cout << "finished reading data" << std::endl;
  std::cout << state->mScans.size() << " scans, " <<
    state->mPosesVo.size() << " vo cams" << std::endl;         

  // post-process lidar scans
  state->integrateScans();
  std::cout << "finished integrating scans" << std::endl;

  // write scans
  state->writeCloud(state->mCloudBefore, outputFileBase + "cloud_before.txt");
  state->writeCloud(state->mCloudAfter, outputFileBase + "cloud_after.txt");
  std::cout << "wrote point clouds" << std::endl;

  // clean up
  bot_param_destroy(param);
  return 0;
}
