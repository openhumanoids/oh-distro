#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <Eigen/Geometry>
#include <ConciseArgs>


struct Scan {
  int64_t mTimestamp;
  double mThetaInit;
  double mThetaStep;
  std::vector<double> mRanges;
};

struct Pose {
  int64_t mTimestamp;
  Eigen::Isometry3d mTransform;
  Eigen::Vector3d mTrans;
  Eigen::Quaterniond mQuat;
};

std::vector<Pose> readPoses(const std::string& iFileName) {
  std::ifstream ifs(iFileName);
  std::vector<Pose> poses;
  std::string line;
  while (std::getline(ifs,line)) {
    std::istringstream iss(line);
    Pose pose;
    auto matx = Eigen::Isometry3d::Identity();
    iss >> pose.mTimestamp >>
      matx(0,0) >> matx(0,1) >> matx(0,2) >> matx(0,3) >>
      matx(1,0) >> matx(1,1) >> matx(1,2) >> matx(1,3) >>
      matx(2,0) >> matx(2,1) >> matx(2,2) >> matx(2,3);
    pose.mTransform = matx;
    pose.mTrans = matx.translation();
    pose.mQuat = matx.rotation();
    poses.push_back(pose);
  }
  ifs.close();
  return poses;
}

void writePoints(const std::vector<Eigen::Vector3d>& iPoints,
                 const std::string& iFileName) {
  std::ofstream ofs(iFileName);
  for (auto p : iPoints) {
    ofs << p[0] << " " << p[1] << " " << p[2] << std::endl;
  }
  ofs.close();
}

Eigen::Vector3d applyInterpolated(const Eigen::Vector3d& iPoint,
                                  const Eigen::Isometry3d& iPose1,
                                  const Eigen::Isometry3d& iPose2,
                                  const Eigen::Isometry3d& iScanPose,
                                  const double iAlpha) {
  Eigen::Isometry3d scanToWorld1 = iPose1*iScanPose;
  Eigen::Isometry3d scanToWorld2 = iPose2*iScanPose;
  Eigen::Vector3d pos1 = scanToWorld1.translation();
  Eigen::Vector3d pos2 = scanToWorld2.translation();
  Eigen::Quaterniond q1(scanToWorld1.rotation());
  Eigen::Quaterniond q2(scanToWorld2.rotation());
  Eigen::Vector3d pos = pos1*(1-iAlpha) + pos2*iAlpha;
  Eigen::Quaterniond q = q1.slerp(iAlpha, q2);
  return q*iPoint + pos;
}

int main(const int iArgc, const char** iArgv) {
  std::string rootDir;
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(rootDir, "r", "root_dir", "input root directory");
  opt.parse();

  std::string fileName;

  // read lidar scans
  fileName = rootDir + "/scans.txt";
  std::ifstream ifs(fileName);
  std::string line;
  std::vector<Scan> scans;
  int totalPoints = 0;
  while (std::getline(ifs,line)) {
    std::istringstream iss(line);
    Scan scan;
    iss >> scan.mTimestamp >> scan.mThetaInit >> scan.mThetaStep;
    double val;
    while (iss >> val) {
      scan.mRanges.push_back(val);
    }
    scans.push_back(scan);
    totalPoints += scan.mRanges.size();
  }
  ifs.close();

  // read vo camera poses
  std::vector<Pose> voPoses = readPoses(rootDir + "/fovis_poses.txt");
  std::cout << "read " << voPoses.size() << " vo camera poses" << std::endl;

  // read original camera poses
  std::vector<Pose> camPoses = readPoses(rootDir + "/cam_poses.txt");
  std::cout << "read " << camPoses.size() << " camera poses" << std::endl;
  
  // read lidar spindle poses
  std::vector<Pose> spindlePoses = readPoses(rootDir + "/spindle_poses.txt");
  std::cout << "read " << spindlePoses.size() << " spindle poses" << std::endl;

  // interpolate camera for every lidar point
  std::cout << "interpolating " << totalPoints << " points..." << std::flush;
  std::vector<Eigen::Vector3d> pointsBefore, pointsAfter;
  pointsAfter.reserve(totalPoints);
  int poseIdx = 0;
  const double kPi = acos(-1);
  const double kScanFrequency = 1.0/40;
  const double kMaxRange = 20;
  for (int i = 0; i < scans.size(); ++i) {
    auto& scan = scans[i];
    int n = scan.mRanges.size();
    double dt = kScanFrequency*(scan.mThetaStep/(2*kPi));
    int64_t time2 = scans[i].mTimestamp;
    int64_t time1 = time2 - int64_t(dt*(n-1)*1e6 + 0.5);
    for (int j = 0; j < n; ++j) {

      // check whether lidar return is valid
      double r = scan.mRanges[j];
      if (r > kMaxRange) continue;

      // find pose for this time
      int64_t t = time1 + int64_t(dt*j*1e6 + 0.5);
      double alpha = (double)j/(n-1);
      if (t < voPoses[poseIdx].mTimestamp) continue;
      while (t > voPoses[poseIdx+1].mTimestamp) {
        ++poseIdx;
        if (poseIdx >= voPoses.size()) break;
      }

      // interpolate between two camera poses
      double theta = scan.mThetaInit + j*scan.mThetaStep;
      Eigen::Vector3d p(r*cos(theta), r*sin(theta), 0);
      Eigen::Vector3d pt;
      pt = applyInterpolated
        (p, camPoses[poseIdx].mTransform, camPoses[poseIdx+1].mTransform,
         spindlePoses[i].mTransform, alpha);
      pointsBefore.push_back(pt);
      pt = applyInterpolated
        (p, voPoses[poseIdx].mTransform, voPoses[poseIdx+1].mTransform,
         spindlePoses[i].mTransform, alpha);
      pointsAfter.push_back(pt);
    }
  }
  std::cout << "done." << std::endl;

  // dump points to files
  writePoints(pointsBefore, rootDir + "/cloud_before.txt");
  writePoints(pointsAfter, rootDir + "/cloud_after.txt");

  return 1;
}
