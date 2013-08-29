#include <stdio.h>
#include <inttypes.h>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include <ConciseArgs>

using namespace std;

// set all unlikely returns to this range (same produced by sensor)
#define MAX_RANGE 60.0
// all ranges shorter than this are assumed to be with the head
#define ASSUMED_HEAD 0.85//0.3
// all ranges longer than this are assumed to be free
#define ASSUMED_FAR 2.0// 2.0
// set all collisions to this range
#define COLLISION_RANGE 0.0

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_, double threshold_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string lidar_channel_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);   
    
    // Output filter lidar:
    bot_core::planar_lidar_t lidar_msgout_;    
    double threshold_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string lidar_channel_, double threshold_):
    lcm_(lcm_), verbose_(verbose_), 
    lidar_channel_(lidar_channel_), threshold_(threshold_){
  
  lcm_->subscribe( lidar_channel_ ,&Pass::lidarHandler,this);
  
  cout << "Finished setting up\n";
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  // Make Local copy to later output
  lidar_msgout_ = *msg;
  std::vector<float> ranges =  msg->ranges;
  
  for( unsigned int i = 1; i < msg->ranges.size() -1 ; i++ ){
    float right_diff = fabs(msg->ranges[i] - msg->ranges[i-1]);
    float left_diff = fabs(msg->ranges[i] - msg->ranges[i+1]);
    
    if (( right_diff > threshold_) || (left_diff > threshold_ )){
      // cout << i<< ": " << right_diff << " and " << left_diff <<"\n";
      ranges[i] = MAX_RANGE;
    }else if( ranges[i] < ASSUMED_HEAD){
      ranges[i] = COLLISION_RANGE;
      
    }
  }
  lidar_msgout_.ranges = ranges;
  lcm_->publish( (lidar_channel_ + "_FREE") , &lidar_msgout_);
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=false;
  string lidar_channel="SCAN";
  double threshold = 0.04; // was 0.04 for a long time
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
