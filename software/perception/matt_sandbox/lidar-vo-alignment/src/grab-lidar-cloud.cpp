#include <fstream>
#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <bot_param/param_client.h>

struct State {
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  int64_t mStartTime;
  int64_t mEndTime;
  double mMinRange;
  double mMaxRange;
  std::string mFinalCoordFrame;
  int64_t mFinalCoordTime;  
  std::list<Eigen::Vector3d> mCloud;

  static constexpr double kPi = acos(-1);
  static constexpr double kScanFrequency = 1.0/40;
  static constexpr double kAngleThresh = 30*kPi/180;

  void setup() {
    mCloud.clear();
    auto lcm = mBotWrapper->getLcm();
    lcm->subscribe("SCAN", &State::onScan, this);
  }

  void onScan(const lcm::ReceiveBuffer* iBuf,
              const std::string& iChannel,
              const bot_core::planar_lidar_t* iMessage) {
    if (iMessage->utime < mStartTime) return;
    if ((mEndTime > 0) && (iMessage->utime > mEndTime)) {
      mLcmWrapper->stopHandleThread();
      return;
    }

    // filter points
    auto msg = *iMessage;
    const int n = msg.nranges;
    for (int i = 1; i < n-1; ++i) {
      double theta1 = msg.rad0 + (i-1)*msg.radstep;
      double theta2 = msg.rad0 + i*msg.radstep;
      double theta3 = msg.rad0 + (i+1)*msg.radstep;
      double r1 = msg.ranges[i-1];
      double r2 = msg.ranges[i];
      double r3 = msg.ranges[i+1];
      Eigen::Vector2d p1(r1*std::cos(theta1), r1*std::sin(theta1));
      Eigen::Vector2d p2(r2*std::cos(theta2), r2*std::sin(theta2));
      Eigen::Vector2d p3(r3*std::cos(theta3), r3*std::sin(theta3));
      Eigen::Vector2d pointDelta1 = (p1-p2).normalized();
      Eigen::Vector2d pointDelta2 = (p3-p2).normalized();
      Eigen::Vector2d ray = p2.normalized();
      double angle1 = std::acos(ray.dot(pointDelta1));
      if (angle1 > kPi/2) angle1 = kPi - angle1;
      double angle2 = std::acos(ray.dot(pointDelta2));
      if (angle2 > kPi/2) angle2 = kPi - angle2;
      if ((angle1 < kAngleThresh) && (angle2 < kAngleThresh)) {
        msg.ranges[i] = 0;
      }
    }

    // grab poses at start and end of scan
    const double dt = kScanFrequency*(msg.radstep/(2*kPi));
    Eigen::Isometry3d poseSpindleBegin, poseSpindleEnd;
    mBotWrapper->getTransform(iChannel, "local",
                              poseSpindleEnd, msg.utime);
    int64_t beginTime = msg.utime - int64_t(dt*(n-1)*1e6 + 0.5);
    mBotWrapper->getTransform(iChannel, "local",
                              poseSpindleBegin, beginTime);

    // compute point cloud
    for (int i = 0; i < n; ++i) {
      double r = msg.ranges[i];
      if ((r < mMinRange) || (r > mMaxRange)) continue;

      // interpolate between two poses
      double alpha = (double)i/(n-1);
      double theta = msg.rad0 + i*msg.radstep;
      Eigen::Vector3d p(r*cos(theta), r*sin(theta), 0);
      Eigen::Vector3d pt;
      pt = applyInterpolated(p, poseSpindleBegin, poseSpindleEnd, alpha);
      mCloud.push_back(pt);
    }
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

  void writeCloud(const std::list<Eigen::Vector3d>& iPoints,
                  const std::string& iFileName) {
    Eigen::Isometry3d transform;
    int64_t coordTime = (mFinalCoordTime>=0) ? mFinalCoordTime : mEndTime;
    mBotWrapper->getTransform("local", mFinalCoordFrame, transform, coordTime);

    std::ofstream ofs(iFileName);
    for (auto p : iPoints) {
      p = transform*p;
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
  std::string coordFrame = "local";
  double coordTime = -1;

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv, "<logfile> <configfile> <outdir>");
  opt.add(startTime, "s", "start_time", "log start time");
  opt.add(endTime, "e", "end_time", "log end time");
  opt.add(minRange, "m", "min_range", "min lidar range to include");
  opt.add(maxRange, "x", "max_range", "max lidar range to include");
  opt.add(coordFrame, "c", "coord_frame", "coordinate frame for final cloud");
  opt.add(coordTime, "t", "coord_time", "time from log start to grab coord frame");
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
  state->mFinalCoordFrame = coordFrame;
  state->mFinalCoordTime = -1;
  if (coordTime >= 0) {
    state->mFinalCoordTime = firstTimestamp + int64_t(coordTime*1e6);
  }
  state->setup();

  // run to gather all data
  lcmWrapper->startHandleThread(true);
  std::cout << "finished processing data" << std::endl;

  // write scans
  state->writeCloud(state->mCloud, outputFileBase + "cloud.txt");
  std::cout << "wrote point cloud" << std::endl;

  // clean up
  bot_param_destroy(param);
  return 0;
}
