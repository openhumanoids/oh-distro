// Example test program for 2D LIDAR alignment
// using the FRSM library. It uses pretty brute force
// settings
//
// Example with attached data sample:
// testRegister2D -a scan_00.csv  -b scan_01.csv  -c scan_02.csv

#include <registration-test/icp_utils.h>

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include <lidar-odom/lidar-odometry.hpp>
#include <lcmtypes/bot_core.hpp>
#include <ConciseArgs>

#include <iterator>
#include <sstream>

void parseTransf(string& transform, float& x, float& y, float& theta) 
{
  transform.erase(std::remove(transform.begin(), transform.end(), '['),
            transform.end());
  transform.erase(std::remove(transform.begin(), transform.end(), ']'),
            transform.end());
  std::replace( transform.begin(), transform.end(), ',', ' ');
  std::replace( transform.begin(), transform.end(), ';', ' ');

  float transValues[3] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 3; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "An error occured while trying to parse the initial "
         << "transformation." << endl
         << "No initial transformation will be used" << endl;
      return;
    }
  }

  x = transValues[0];
  y = transValues[1];
  theta = transValues[2];
}

struct CommandLineConfig
{
  std::string filenameA;
  std::string filenameB;
  std::string filenameC;
};

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, int indTransfFile);
    
    ~App(){
    }

    const char *homedir;

    int tot_clouds; //number of clouds to combine

    string cloudA;
    string cloudB;

    void doWork(Eigen::MatrixXf &transf_matrix, int transf_index);
    void doWork();

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;    
    LidarOdom* lidarOdom_;
};    

App::App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, int indTransfFile) : 
       lcm_(lcm_), cl_cfg_(cl_cfg_){
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }
  
  LidarOdomConfig lo_cfg = LidarOdomConfig();

  // Overwrite default config values with more brute force search:
  lo_cfg.matchingMode = FRSM_GRID_COORD;
  lo_cfg.initialSearchRangeXY = 1.7;
  lo_cfg.initialSearchRangeTheta = 1.7;
  lo_cfg.maxSearchRangeXY = 2.0;
  lo_cfg.maxSearchRangeTheta = 2.0;

  string fileinitname;
  fileinitname.clear();
  fileinitname.append(homedir);
  fileinitname.append("/logs/multisenselog__2015-11-16/initialization/initTransf.txt");
  
  cout << "indTransfFile: " << indTransfFile << endl;
  string initialT = readLineFromFile(fileinitname, indTransfFile);

  cout << "STRING initialT: " << initialT << endl;

  parseTransf(initialT, lo_cfg.startPoseInputX, 
         lo_cfg.startPoseInputY, lo_cfg.startPoseInputTheta);

  lo_cfg.do_two_scans_matching = true;
  tot_clouds = 10;
  
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

void App::doWork(Eigen::MatrixXf &transf_matrix, int transf_index){
  std::vector<float> xA;
  std::vector<float> yA;
  readCSVFile(cloudA, xA, yA);
  std::cout << xA.size() << " points in File A\n";
  lidarOdom_->doOdometry(xA, yA, xA.size(), 0);

  std::vector<float> xB;
  std::vector<float> yB;
  readCSVFile(cloudB, xB, yB);
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
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  CommandLineConfig cl_cfg;
  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.filenameA, "a", "filenameA", "FilenameA.csv");
  parser.add(cl_cfg.filenameB, "b", "filenameB", "FilenameB.csv");
  parser.add(cl_cfg.filenameC, "c", "filenameC", "FilenameC.csv");
  parser.parse();

  App* app = new App(lcm, cl_cfg, 0);
  
  if (cl_cfg.filenameA.empty())
  {
    int transf_index = 0;
    //-------------------------
    // To store...
    int cols = 0;
    for (int i = app->tot_clouds-1; i > 0; i--)
    { 
      cols = cols + i;
    }
    Eigen::MatrixXf truth_res = Eigen::MatrixXf::Zero(3, cols);
    //-------------------------
    for (int i = 0; i < app->tot_clouds-1; i++)
    {
      for (int j = 1+i; j < app->tot_clouds; j++)
      {
        // Set index to load inital transformations from file
        transf_index++;

        // Name reference cloud from file
        app->cloudA.clear();
        app->cloudA.append(app->homedir);
        app->cloudA.append("/logs/multisenselog__2015-11-16/planar_scans/scan_0");
        app->cloudA.append(to_string(i));
        app->cloudA.append(".csv");

        // Name input cloud from file
        app->cloudB.clear();
        app->cloudB.append(app->homedir);
        app->cloudB.append("/logs/multisenselog__2015-11-16/planar_scans/scan_0");
        app->cloudB.append(to_string(j));
        app->cloudB.append(".csv");

        app->doWork(truth_res, transf_index);

        app->~App();
        app = new App(lcm, cl_cfg, transf_index);
      }
    }  
    cout << "Final matrix:" << endl << truth_res.block(0,0,3,6) << endl;  

    string out_file;
    out_file.append(app->homedir);
    out_file.append("/logs/multisenselog__2015-11-16/results/truth_transf.txt");
    writeTransformToFile(truth_res, out_file);
  }
  else
  {
    app->cloudA = cl_cfg.filenameA;
    app->cloudB = cl_cfg.filenameB;

    app->doWork();
  }
}
