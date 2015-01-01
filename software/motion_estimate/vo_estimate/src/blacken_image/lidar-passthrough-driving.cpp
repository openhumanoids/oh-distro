#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include <ConciseArgs>
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, double min_range_);

    ~Pass(){
    }
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    double min_range_;
    std::string input_channel_;
    void lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg);
    bot_core::planar_lidar_t last_lidar_msg_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, double min_range_):
    lcm_(lcm_), min_range_(min_range_){
  input_channel_ = "SCAN";
  lcm_->subscribe( input_channel_ ,&Pass::lidarHandler,this);
  cout << "Finished setting up\n";
}

void Pass::lidarHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::planar_lidar_t* msg){
  last_lidar_msg_ = *msg;
  for (size_t i=0; i < last_lidar_msg_.ranges.size(); i++){
    float val = last_lidar_msg_.ranges[i];
    if (val  < min_range_ ){
    last_lidar_msg_.ranges[i] = 30;
    }
  }
  lcm_->publish( "SCAN_FREE" , &last_lidar_msg_ );
}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "blacken-image");
  double min_range=3.0;
  parser.add(min_range, "m", "min_range", "min range");
  parser.parse();
  cout << min_range << " is min_range\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  Pass app(lcm, min_range);
  cout << "Ready to handle" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}