// Example test program for 2D LIDAR alignment
// using the FRSM library. It uses pretty brute force
// settings
//
// Example with attached data sample:
// testRegister2D -a scan_00.csv  -b scan_01.csv  -c scan_02.csv

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include <lidar-odom/lidar-odometry.hpp>
#include <lcmtypes/bot_core.hpp>
#include <ConciseArgs>

#include <iterator>
#include <sstream>

struct CommandLineConfig
{
  std::string filenameA;
  std::string filenameB;
  std::string filenameC;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_);
    
    ~App(){
    }

    void doWork();

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;    
    LidarOdom* lidarOdom_;
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_) : 
       lcm_(lcm_), cl_cfg_(cl_cfg_){
  
  LidarOdomConfig lo_cfg = LidarOdomConfig();

  // Overwrite default config values with more brute force search:
  lo_cfg.matchingMode = FRSM_GRID_COORD;
  lo_cfg.initialSearchRangeXY = 1.7;
  lo_cfg.initialSearchRangeTheta = 1.7;
  lo_cfg.maxSearchRangeXY = 2.0;
  lo_cfg.maxSearchRangeTheta = 2.0;

  lidarOdom_ = new LidarOdom(lcm_, lo_cfg);
}

void readCSVFile(std::string filename, std::vector<float> &x, std::vector<float> &y){

    std::vector<std::vector<double> > values;
    std::ifstream fin(filename.c_str());
    for (std::string line; std::getline(fin, line); )
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream in(line);
        values.push_back(
            std::vector<double>(std::istream_iterator<double>(in),
                                std::istream_iterator<double>()));
    }

   for (size_t i=0; i<values.size(); i++){
     x.push_back(values[i][0]);
     y.push_back(values[i][1]);
   }
}

void App::doWork(){

  std::vector<float> xA;
  std::vector<float> yA;
  readCSVFile(cl_cfg_.filenameA, xA, yA);
  std::cout << xA.size() << " points in File A\n";
  lidarOdom_->doOdometry(xA, yA, xA.size(), 0);

  // Match second scan without a prior
  std::string i;
  cout << "Match B: Continue? ";
  cin >> i;

  std::vector<float> xB;
  std::vector<float> yB;
  readCSVFile(cl_cfg_.filenameB, xB, yB);
  std::cout << xB.size() << " points in File B\n";
  lidarOdom_->doOdometry(xB, yB, xB.size(), 1);

  // Match third scan giving a prior rotation for the heading
  // this alignment would otherwise fail:
  cout << "Match C: Continue? ";
  cin >> i;

  std::vector<float> xC;
  std::vector<float> yC;
  readCSVFile(cl_cfg_.filenameC, xC, yC);
  std::cout << xC.size() << " points in File C\n";

  ScanTransform prior;
  prior.x = 0;
  prior.y = 0;
  prior.theta = 1.7;
  lidarOdom_->doOdometry(xC, yC, xC.size(), 2, &prior);



  // 2. Determine the body position using the LIDAR motion estimate:
  Eigen::Isometry3d pose = lidarOdom_->getCurrentPose();
  Eigen::Quaterniond orientation(pose.rotation());

  Eigen::Vector3d rpy = orientation.matrix().eulerAngles(0,1,2);

  std::cout << "\n";
  std::cout << "x,y,yaw (m,m,deg): "<< pose.translation().x() << ", " << pose.translation().y()
            << ", "
            << rpy[2]*180/M_PI << "\n";
}

int main(int argc, char **argv){

  CommandLineConfig cl_cfg;
  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.filenameA, "a", "filenameA", "FilenameA.csv");
  parser.add(cl_cfg.filenameB, "b", "filenameB", "FilenameB.csv");
  parser.add(cl_cfg.filenameC, "c", "filenameC", "FilenameC.csv");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App app= App(lcm, cl_cfg);

  app.doWork();
}
