#include <fstream>
#include <sstream>
#include <ConciseArgs>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;

Cloud::Ptr readCloud(const std::string& iFileName) {
  Cloud::Ptr cloud(new Cloud());
  std::ifstream ifs(iFileName);
  std::string line;
  Point pt;
  while (std::getline(ifs,line)) {
    std::istringstream iss(line);
    iss >> pt.x >> pt.y >> pt.z;
    cloud->push_back(pt);
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

int main(const int iArgc, const char** iArgv) {
  std::string fileName1;
  std::string fileName2;
  ConciseArgs opt(iArgc, (char**)iArgv, "<cloud1> <cloud2>");
  opt.parse(fileName1, fileName2);
  
  Cloud::Ptr cloud1 = readCloud(fileName1);
  Cloud::Ptr cloud2 = readCloud(fileName2);

  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setInputCloud(cloud2);
  icp.setInputTarget(cloud1);
  Cloud final;
  icp.align(final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;  

  std::ofstream ofs("/home/antone/temp/head_data/registered.txt");
  for (auto p : final.points) {
    ofs << p.x << " " << p.y << " " << p.z << std::endl;
  }
  ofs.close();

  return 1;
}
