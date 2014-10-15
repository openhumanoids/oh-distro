#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>

#include <opencv2/opencv.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <bot_param/param_client.h>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/multisense/images_t.hpp>

#include <zlib.h>

struct State {
  std::shared_ptr<drc::BotWrapper> mBotWrapper;
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::string mRootDir;
  std::ofstream mScanFile;
  std::ofstream mScanPoseFile;
  std::ofstream mSpindlePoseFile;
  std::ofstream mCamPoseFile;
  int64_t mStartTime;
  int64_t mEndTime;

  void setup() {
    auto lcm = mBotWrapper->getLcm();
    mScanFile.open(mRootDir + "/scans.txt");
    mScanFile << std::setprecision(12);
    mScanPoseFile.open(mRootDir + "/scan_poses.txt");
    mScanPoseFile << std::setprecision(15);
    mSpindlePoseFile.open(mRootDir + "/spindle_poses.txt");
    mSpindlePoseFile << std::setprecision(15);
    mCamPoseFile.open(mRootDir + "/cam_poses.txt");
    mCamPoseFile << std::setprecision(15);
    lcm->subscribe("CAMERA", &State::onCameras, this);
    lcm->subscribe("CAMERA_LEFT", &State::onCamera, this);
    lcm->subscribe("SCAN", &State::onScan, this);
  }


  void onCamera(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const bot_core::image_t* iMessage) {
    const auto& img = *iMessage;
    const int w = img.width;
    const int h = img.height;
    char outFileName[256];
    cv::Mat cvImg;
    switch (img.pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_MJPEG:
      cvImg = cv::imdecode(cv::Mat(img.data), -1); break;
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      cvImg = cv::Mat(h, w, CV_8UC1, (void*)img.data.data()); break;
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      cvImg = cv::Mat(h, w, CV_8UC3, (void*)img.data.data()); break;
    default:
      std::cout << "error: unknown pixel format" << std::endl; break;
    }
    if (cvImg.channels() == 3) cv::cvtColor(cvImg, cvImg, CV_BGR2RGB);
    sprintf(outFileName, "%s/color_%ld.png", mRootDir.c_str(), img.utime);
    cv::imwrite(outFileName, cvImg);

    Eigen::Isometry3d camToLocal;
    mBotWrapper->getTransform(iChannel,"local",camToLocal,
                              iMessage->utime);
    auto& m = camToLocal;
    mCamPoseFile << iMessage->utime << " " <<
      m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << " " <<
      m(1,0) << " " << m(1,1) << " " << m(1,2) << " " << m(1,3) << " " << 
      m(2,0) << " " << m(2,1) << " " << m(2,2) << " " << m(2,3) << std::endl;
  }


  void onCameras(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const multisense::images_t* iMessage) {
    if ((mEndTime > 0) && (iMessage->utime > mEndTime)) {
      mLcmWrapper->stopHandleThread();
      return;
    }

    for (int i = 0; i < iMessage->n_images; ++i) {
      const bot_core::image_t& img = iMessage->images[i];
      const int w = img.width;
      const int h = img.height;
      char outFileName[256];
      cv::Mat cvImg;
      auto imgType = iMessage->image_types[i];
      if (imgType == multisense::images_t::LEFT) {
        onCamera(iBuf, iChannel + "_LEFT", &img);
      }

      else if ((imgType == multisense::images_t::DISPARITY) ||
               (imgType == multisense::images_t::DISPARITY_ZIPPED)) {
        /*
        if (imgType == multisense::images_t::DISPARITY) {
          //cv::Mat(h,w,CV_16UC1,(void*)img.data.data()).
            //convertTo(cvImg, CV_32F, 1.0f/16);
          cvImg = cv::Mat(h,w,CV_16UC1,(void*)img.data.data());
        }
        else */{
          std::vector<uint8_t> buf(w*h*2);
          unsigned long len = buf.size();
          uncompress(buf.data(), &len, img.data.data(), img.data.size());
          //cvImg = cv::Mat(h,w,CV_16UC1,buf.data());
          cv::Mat(h, w, CV_16UC1, (void*)buf.data()).
            convertTo(cvImg, CV_32F, 1.0/16);
        }
        sprintf(outFileName, "%s/disp_%ld.float", mRootDir.c_str(), img.utime);
        writeFloat(cvImg, outFileName);
      }

    }
  }

  void writeFloat(const cv::Mat& iImage, const std::string& iFileName) {
    std::ofstream ofs(iFileName, std::ios::binary);
    ofs.write((char*)&iImage.cols, sizeof(iImage.cols));
    ofs.write((char*)&iImage.rows, sizeof(iImage.rows));
    ofs.write((char*)iImage.data, sizeof(float)*iImage.rows*iImage.cols);
    ofs.close();
  }

  void onScan(const lcm::ReceiveBuffer* iBuf,
              const std::string& iChannel,
              const bot_core::planar_lidar_t* iMessage) {
    if ((mEndTime > 0) && (iMessage->utime > mEndTime)) {
      mLcmWrapper->stopHandleThread();
      return;
    }
    {
      Eigen::Isometry3d scanToLocal;
      mBotWrapper->getTransform(iChannel,"local",scanToLocal,
                                iMessage->utime);
      auto& m = scanToLocal;
      mScanPoseFile << iMessage->utime << " " <<
        m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << " " <<
        m(1,0) << " " << m(1,1) << " " << m(1,2) << " " << m(1,3) << " " << 
        m(2,0) << " " << m(2,1) << " " << m(2,2) << " " << m(2,3) << std::endl;
    }
    {
      Eigen::Isometry3d spindlePose;
      mBotWrapper->getTransform(iChannel,"CAMERA_LEFT",spindlePose,
                                iMessage->utime);      
      auto& m = spindlePose;
      mSpindlePoseFile << iMessage->utime << " " <<
        m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << " " <<
        m(1,0) << " " << m(1,1) << " " << m(1,2) << " " << m(1,3) << " " << 
        m(2,0) << " " << m(2,1) << " " << m(2,2) << " " << m(2,3) << std::endl;
    }

    mScanFile << iMessage->utime << " " << iMessage->rad0 << " " <<
      iMessage->radstep << " ";
    for (auto r : iMessage->ranges) {
      mScanFile << r << " ";
    }
    mScanFile << std::endl;
  }
  
};

int main(const int iArgc, const char** iArgv) {

  std::string logFileName;
  std::string botParamFileName;
  std::string rootDir;
  double startTime = 0;
  double endTime = -1;

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(logFileName, "l", "log_file", "log file");
  opt.add(botParamFileName, "p", "param_file", "bot param config file");
  opt.add(rootDir, "r", "root_dir", "output root directory");
  opt.add(startTime, "s", "start_time", "log start time");
  opt.add(endTime, "e", "end_time", "log end time");
  opt.parse();

  // get start time
  lcm::LogFile logFile(logFileName, "r");
  if (!logFile.good()) {
    std::cerr << "cannot open log file " << logFileName << std::endl;
    return -1;
  }
  const lcm::LogEvent* event = logFile.readNextEvent();
  int64_t firstTimestamp = event->timestamp;
  int64_t startTimestamp = firstTimestamp + int64_t(1e6*startTime + 0.5);
  int64_t endTimestamp = (endTime<0) ? -1 :
    firstTimestamp + int64_t(1e6*endTime + 0.5);
  std::cout << "time range " <<
    startTimestamp << " " << endTimestamp << std::endl;

  // open lcm
  std::ostringstream oss;
  oss << "file://" << logFileName << "?speed=0&mode=r&start_timestamp=" <<
    startTimestamp;
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
  state->mRootDir = rootDir;
  state->mStartTime = startTimestamp;
  state->mEndTime = endTimestamp;
  state->setup();

  // run
  lcmWrapper->startHandleThread(true);

  // clean up
  bot_param_destroy(param);
  return 0;
}
