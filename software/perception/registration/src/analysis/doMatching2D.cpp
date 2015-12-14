// doMatching2D

// Program for 2D LIDAR alignment
// using the FRSM library. It uses pretty brute force
// settings.
//
// Options: - take 2 scans from user and give transformation between them (-a reference.csv, -b input.csv)
//          - loop over a dataset and give transformation between all combinations (write to file)

#include <icp-registration/icp_utils.h>

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include <lidar-odom/lidar-odometry.hpp>
#include <lcmtypes/bot_core.hpp>
#include <ConciseArgs>

#include <iterator>
#include <sstream>
#include <iostream>

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
};

Eigen::Isometry3d getXYThetaAsIsometry3d(Eigen::Vector3d transl, Eigen::Vector3d rpy){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << transl[0], transl[1], transl[2];

  double rpy_d[3] = { rpy[0], rpy[1], rpy[2] };
  double quat[4];
  bot_roll_pitch_yaw_to_quat(rpy_d, quat);
  Eigen::Quaterniond q(quat[0], quat[1],quat[2],quat[3]);
  tf_out.rotate(q);

  return tf_out;
}

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_, const CommandLineConfig& cl_cfg_, int indTransfFile);
    
    ~App(){
    }

    const char *homedir;

    int num_clouds; //number of clouds to combine

    string cloudA;
    string cloudB;

    string initialT;  // We store the transform used to initialize the 
                      // pose of cloud B. The lidar odometry class outputs the complete
                      // transformation from the starting pose (without initialization) 
                      // of cloud B to cloud A. Instead, we want to store just the remaining 
                      // x,y,theta after initialization. 
    ScanTransform initInput;
    ScanTransform initReference;

    void doWork(Eigen::MatrixXf &transf_matrix, int transf_index);
    void doWork();

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    const CommandLineConfig cl_cfg_;    
    LidarOdom* lidarOdom_;
};   

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
  
  initialT = readLineFromFile(fileinitname, indTransfFile);

  float tmpx, tmpy, tmptheta;
  parseTransf(initialT, tmpx, tmpy, tmptheta);
  initInput.x = tmpx;
  initInput.y = tmpy;
  initInput.theta = (tmptheta * M_PI)/180;

  cout << "initInput: " << tmpx << "," << tmpy << "," << tmptheta << endl;

  initReference.x = 0;
  initReference.y = 0;
  initReference.theta = (0 * M_PI)/180;

  num_clouds = 10;
  
  lidarOdom_ = new LidarOdom(lcm_, lo_cfg);
}

void App::doWork(Eigen::MatrixXf &transf_matrix, int transf_index){
  std::vector<float> xA;
  std::vector<float> yA;
  readCSVFile(cloudA, xA, yA);
  std::cout << xA.size() << " points in File A\n";
  lidarOdom_->doOdometry(xA, yA, xA.size(), 0, &initReference);

  std::vector<float> xB;
  std::vector<float> yB;
  readCSVFile(cloudB, xB, yB);
  std::cout << xB.size() << " points in File B\n";
  lidarOdom_->doOdometry(xB, yB, xB.size(), 1, &initInput);

  // 2. Determine the body position using the LIDAR motion estimate:
  /*
  Eigen::Isometry3d tf_01_fixed = lidarOdom_->getCurrentPose();
  Eigen::Isometry3d head_to_fixscan, fixscan_to_head;
  head_to_fixscan = getXYThetaAsIsometry3d(transl_head_to_fixed, rpy_head_to_fixed);
  fixscan_to_head = head_to_fixscan.inverse();
  Eigen::Isometry3d tf_01_head = fixscan_to_head * tf_01_fixed * head_to_fixscan;
  */
  Eigen::Isometry3d tf_01_head = lidarOdom_->getCurrentPose();

  Eigen::Quaterniond orientation(tf_01_head.rotation());
  Eigen::Vector3d rpy = orientation.matrix().eulerAngles(0,1,2);

  float xres,yres,thetares;
  xres = tf_01_head.translation().x();
  yres = tf_01_head.translation().y();
  thetares = rpy[2] * 180 / M_PI;

  transf_matrix(0, transf_index-1) = xres;
  transf_matrix(1, transf_index-1) = yres;   
  transf_matrix(2, transf_index-1) = thetares;

  cout << "x,y,yaw (m,m,deg): "<< xres << ", " << yres << ", " << thetares << "\n";
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
  parser.parse();

  App* app = new App(lcm, cl_cfg, 0);
  
  if (cl_cfg.filenameA.empty())
  {
    int transf_index = 0;
    //-------------------------
    // To store...
    int cols = 0;
    for (int i = app->num_clouds-1; i > 0; i--)
    { 
      cols = cols + i;
    }
    Eigen::MatrixXf truth_res = Eigen::MatrixXf::Zero(3, cols);
    //-------------------------
    for (int i = 0; i < app->num_clouds-1; i++)
    {
      for (int j = 1+i; j < app->num_clouds; j++)
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

        // Uncomment for visualization of matching results one by one:
        //cout << "Press ENTER to continue..." << endl;
        //cin.get();

        app->~App();
        app = new App(lcm, cl_cfg, transf_index);
      }
    }  
    cout << "Final matrix:" << endl << truth_res.block(0,0,3,6) << endl;  

    string out_file;
    out_file.append(app->homedir);
    out_file.append("/logs/multisenselog__2015-11-16/results/truth_transf.txt");
    writeTransformToFile(truth_res, out_file, app->num_clouds);
  }
  else
  {
    app->~App();
    app = new App(lcm, cl_cfg, 0);

    app->cloudA = cl_cfg.filenameA;
    app->cloudB = cl_cfg.filenameB;

    Eigen::MatrixXf truth_res = Eigen::MatrixXf::Zero(3, 1);
    app->doWork(truth_res, 0);
  }
}
