#include <zlib.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <boost/shared_ptr.hpp>
#include <lidar-odom/lidar-odometry.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/pronto/pointcloud_t.hpp>
#include <ConciseArgs>


using namespace std;

struct CommandLineConfig
{
  bool use_velodyne;
  bool init_with_message; // initialize off of a pose or vicon
  std::string output_channel;
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
  
  lidarOdom_ = new LidarOdom(lcm_);
}


void App::doWork(){
  int n_points = 400;

  // 1. Update LIDAR Odometry
  std::vector<float> x;
  std::vector<float> y;
  for (int i=0;i<n_points; i++){
    x.push_back( 0 );//msg->points[i][0] );
    y.push_back( 0.01*i );//msg->points[i][1] );
  }

  lidarOdom_->doOdometry(x, y, n_points, 0);
  lidarOdom_->doOdometry(x, y, n_points, 1);

  // 2. Determine the body position using the LIDAR motion estimate:
  Eigen::Isometry3d pose = lidarOdom_->getCurrentPose();
  Eigen::Quaterniond orientation(pose.rotation());

  Eigen::Vector3d rpy = orientation.matrix().eulerAngles(2,1,0);

  std::cout << pose.translation().x() << ", " << pose.translation().y() 
            << rpy << "\n";

}




int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.use_velodyne = false;
  cl_cfg.init_with_message = TRUE;
  cl_cfg.output_channel = "POSE_BODY";

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.init_with_message, "g", "init_with_message", "Bootstrap internal estimate using VICON or POSE_INIT");
  parser.add(cl_cfg.output_channel, "o", "output_channel", "Output message e.g POSE_BODY");
  parser.add(cl_cfg.use_velodyne, "v", "use_velodyne", "Use a velodyne instead of the LIDAR");
  parser.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App app= App(lcm, cl_cfg);

  app.doWork();
}
