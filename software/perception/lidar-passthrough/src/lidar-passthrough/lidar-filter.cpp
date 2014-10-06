#include <stdio.h>
#include <inttypes.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <bot_param/param_client.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pronto_utils/pronto_vis.hpp> // visualize pt clds
#include <pronto_utils/pronto_lcm.hpp> // unpack lidar to xyz


#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include <ConciseArgs>

using namespace std;

// set all unlikely returns to this range (same produced by sensor)
#define MAX_RANGE 60.0
// all ranges shorter than this are assumed to be with the head
#define ASSUMED_HEAD 0.3//0.3
// all ranges longer than this are assumed to be free
#define ASSUMED_FAR 2.0// 2.0
// set all collisions to this range
#define COLLISION_RANGE 0.0

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_, double delta_threshold_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string lidar_channel_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    
    // Output filter lidar:
    bot_core::planar_lidar_t lidar_msgout_;    
    double delta_threshold_;
    
    BotParam* botparam_;
    bot::frames* frames_cpp_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_, double delta_threshold_):
    lcm_(lcm_), verbose_(verbose_), 
    lidar_channel_(lidar_channel_), delta_threshold_(delta_threshold_){
  
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  frames_cpp_= new bot::frames(lcm_, botparam_);      
  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  
  cout << "Finished setting up\n";
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  // Make Local copy to later output
  lidar_msgout_ = *msg;
  std::vector<float> original_ranges =  lidar_msgout_.ranges;
  
  
  
  Eigen::Isometry3d scan_to_local; // transform used to do conversion
  frames_cpp_->get_trans_with_utime("SCAN", "body"  , msg->utime, scan_to_local);
    
  // 1. Convert scan into simple XY point cloud:  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  double minRange =-100.0; // consider everything - don't remove any points
  double maxRange = 100.0;
  double validBeamAngles[] ={-10,10}; 
  convertLidar(msg->ranges, msg->nranges, msg->rad0,
      msg->radstep, scan_cloud, minRange, maxRange,
      validBeamAngles[0], validBeamAngles[1]);    

  if (scan_cloud->points.size() !=  msg->nranges ){
    std::cout << "npoints and nranges are not equal\n";
    std::cout << scan_cloud->points.size() << "\n";
    std::cout << msg->nranges << "\n";
    exit(-1); 
  }  
  
  // 2. Project the scan into camera frame:
  Eigen::Isometry3f pose_f = scan_to_local.cast<float>();
  Eigen::Quaternionf pose_quat(pose_f.rotation());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud (*scan_cloud, *cloud_transformed,
      pose_f.translation(), pose_quat);    
  /// cloud_transformed now contains the lidar returns as 3d points in the local frame
    
  for( unsigned int i = 0; i < lidar_msgout_.ranges.size()  ; i++ ){
    
    // Short Range Points:
    if( original_ranges[i] < ASSUMED_HEAD){
      lidar_msgout_.ranges[i] = COLLISION_RANGE;
    }
    
    // heuristic filtering of the weak intensity lidar returns
    if (( lidar_msgout_.intensities[i] < 2000 ) && ( original_ranges[i] < 2) ){
      lidar_msgout_.ranges[i] = MAX_RANGE;
    }
    
    // Edge effect filter
    if ( (i>0) && (i<lidar_msgout_.ranges.size()) ){
      float right_diff = fabs(original_ranges[i] - original_ranges[i-1]);
      float left_diff = fabs(original_ranges[i] - original_ranges[i+1]);
      if (( right_diff > delta_threshold_) || (left_diff > delta_threshold_ )){
        // cout << i<< ": " << right_diff << " and " << left_diff <<"\n";
        if (original_ranges[i] < 2){
          lidar_msgout_.ranges[i] = MAX_RANGE;
        }
      }
    }
    

    /*
    // Box filter around the robot (temporary until urdf improves)
    if ((cloud_transformed->points[i].x < 0.65) &&
        ( fabs (cloud_transformed->points[i].y) < 0.75)){
      lidar_msgout_.ranges[i] = COLLISION_RANGE; 
    }
    */
    
  }
//  lidar_msgout_.ranges = ranges;
  
  lcm_->publish( (lidar_channel_ + "_FREE") , &lidar_msgout_);
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=false;
  string lidar_channel="SCAN";
  double threshold = 0.03; 
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(lidar_channel, "l", "lidar_channel", "Incoming LIDAR channel");
  parser.add(threshold, "t", "threshold", "Lidar threshold [lower removes more points]");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << lidar_channel << " is lidar_channel\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,lidar_channel, threshold);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
