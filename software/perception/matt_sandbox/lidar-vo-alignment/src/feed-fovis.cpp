#include <sstream>
#include <fstream>

#include <ConciseArgs>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>

#include <drcvision/fovision.hpp>
#include <drcvision/voconfig.hpp>

#include <opencv2/opencv.hpp>

int main(const int iArgc, const char** iArgv) {
  std::string rootDir;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(rootDir, "r", "root_dir", "input root directory");
  opt.parse();

  // set up vo
  std::shared_ptr<drc::BotWrapper> botWrapper(new drc::BotWrapper());
  std::shared_ptr<drc::LcmWrapper>
    lcmWrapper(new drc::LcmWrapper(botWrapper->getLcm()));
  auto boostLcm = lcmWrapper->getBoost();
  auto config =
    new voconfig::KmclConfiguration(botWrapper->getBotParam(), "CAMERA");
  boost::shared_ptr<fovis::StereoCalibration>
    calib(config->load_stereo_calibration());
  FoVision vo(boostLcm, calib);

  // find file timestamps
  std::ifstream ifs(rootDir + "/cam_poses.txt");
  std::vector<int64_t> times;
  std::string line;
  while (std::getline(ifs,line)) {
    std::istringstream iss(line);
    int64_t utime;
    iss >> utime;
    times.push_back(utime);
  }

  // iterate
  std::string poseFileName = rootDir + "/fovis_poses.txt";
  std::ofstream ofs(poseFileName);
  ofs << std::setprecision(15);
  for (auto utime : times) {
    std::string fileName;
    std::ostringstream oss;

    // read image
    oss << rootDir << "/color_" << utime << ".png";
    cv::Mat img = cv::imread(oss.str());
    cv::cvtColor(img,img,CV_RGB2GRAY);

    // read disparity
    oss.str("");
    oss.clear();
    oss << rootDir << "/disp_" << utime << ".float";
    std::ifstream ifs(oss.str(), std::ios::binary);
    int width, height;
    ifs.read((char*)&width, sizeof(width));
    ifs.read((char*)&height, sizeof(height));
    std::vector<float> vals(width*height);
    ifs.read((char*)vals.data(), width*height*sizeof(float));
    ifs.close();
    cv::Mat disp(height,width,CV_32FC1,vals.data());

    // do fovis
    vo.doOdometry(img.data, (float*)disp.data, utime);
    Eigen::Isometry3d delta;
    auto worldToCamera = Eigen::Isometry3d::Identity();
    Eigen::MatrixXd cov;
    fovis::MotionEstimateStatusCode status;
    vo.getMotion(delta, cov, status);
    worldToCamera = worldToCamera*delta;
    vo.fovis_stats();

    // write pose
    auto cameraPose = worldToCamera.inverse();
    auto& m = cameraPose;
    ofs << utime << " " <<
      m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << " " <<
      m(1,0) << " " << m(1,1) << " " << m(1,2) << " " << m(1,3) << " " << 
      m(2,0) << " " << m(2,1) << " " << m(2,2) << " " << m(2,3) << std::endl;
  }
  ofs.close();

  return 1;
}
