#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"
#include <lcmtypes/microstrain_comm.hpp>
#include <ConciseArgs>

using namespace std;
using namespace drc;


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string ins_channel_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string ins_channel_;
    
    void insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  microstrain::ins_t* msg);   
    microstrain::ins_t lidar_msgout_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string ins_channel_):
    lcm_(lcm_), verbose_(verbose_), 
    ins_channel_(ins_channel_){
  lcm_->subscribe( ins_channel_ ,&Pass::insHandler,this);
  cout << "Finished setting up\n";
}


void Pass::insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  microstrain::ins_t* msg){
    bot_core::pose_t pose_msg;
    pose_msg.utime = msg->utime;
    pose_msg.pos[0] = 0;
    pose_msg.pos[1] = 0;
    pose_msg.pos[2] = 0;
    pose_msg.orientation[0] = msg->quat[0];
    pose_msg.orientation[1] = msg->quat[1];
    pose_msg.orientation[2] = msg->quat[2];
    pose_msg.orientation[3] = msg->quat[3];
    lcm_->publish("POSE_HEAD", &pose_msg); 
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=false;
  string ins_channel="MICROSTRAIN_INS";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(ins_channel, "l", "ins_channel", "Incoming INS channel");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << ins_channel << " is ins_channel\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,ins_channel);
  cout << "Ready to convert from imu to pose" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
