#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/utime_two_t.hpp"
#include "lcmtypes/drc/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/bot_core/pose_t.hpp"
#include <latency/latency.hpp>

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, int period_);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleAtlasStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_state_t * msg);
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg);
  void handleCommandMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_command_t * msg);

  void handleIMUBatch(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_raw_imu_batch_t * msg);
  void handPoseBody(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t * msg);

  void handleUtimeTwoMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_two_t * msg);

  int period_;
  
private:
  std::vector<Latency*> lats_;
  
  std::vector <float> lat_time_;
  std::vector <float> lat_msgs_;
  int counter_;
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int period_):
    _lcm(_lcm),period_(period_){

  Latency* a_lat = new Latency(period_); 
  lats_.push_back(a_lat) ;
  Latency* a_lat2 = new Latency(period_);
  lats_.push_back(a_lat2) ;
  Latency* a_lat3 = new Latency(period_);
  lats_.push_back(a_lat3) ;
  
  lat_time_ = {0.0, 0.0, 0.0};
  lat_msgs_ = {0.0, 0.0, 0.0};
  
  _lcm->subscribe("ATLAS_STATE", &App::handleAtlasStateMsg, this);
  _lcm->subscribe("EST_ROBOT_STATE",&App::handleRobotStateMsg,this);
  _lcm->subscribe("ATLAS_COMMAND",&App::handleCommandMsg,this);
  

  _lcm->subscribe("ATLAS_IMU_BATCH", &App::handleIMUBatch, this);
  _lcm->subscribe("POSE_BODY", &App::handPoseBody, this);  
  counter_=0;
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void App::handleAtlasStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_state_t * msg){
  lats_[0]->add_from(msg->utime, _timestamp_now() );
  lats_[2]->add_from(msg->utime, _timestamp_now() );  
}

void App::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg){
  bool new_data = lats_[0]->add_to(msg->utime, _timestamp_now(), "SYNC", lat_time_[0], lat_msgs_[0] );
  
  if (new_data){
    std::cout.precision(5);
    std::cout.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed
    std::cout << lat_time_[0] << " | " << lat_time_[1] << " | "  << lat_time_[2] << "   ||   "
              << lat_msgs_[0] << " | " << lat_msgs_[1] << " | "  << lat_msgs_[2] << "\n";
              
    if (counter_% 10==0){
      std::cout << "AST-ERS | IMU-SE  | AST-CMD"
                << "   ||   "
                << "AST-ERS | IMU-SE  | AST-CMD\n";//   <msec|msg>\n";
    }
              
    counter_++;
  }
}

void App::handleCommandMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_command_t * msg)  {
  lats_[2]->add_to(msg->utime, _timestamp_now(), "FULL", lat_time_[2], lat_msgs_[2] );      
}



void App::handleIMUBatch(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_raw_imu_batch_t * msg){
  lats_[1]->add_from(msg->utime, _timestamp_now() );
}
void App::handPoseBody(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t * msg)  {
  lats_[1]->add_to(msg->utime, _timestamp_now(), "SEST" , lat_time_[1], lat_msgs_[1]);      
}


void App::handleUtimeTwoMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_two_t * msg)  {
  // latency_->add_to(msg->utime_sim, _timestamp_now(), "BLAH" );
  
  // TODO: switch to use these this timing to use the time calculated within matlab:
  // NOT TESTED
  //latency_->add_to(msg->utime_sim, msg->utime_wall, message_ );
}

int main (int argc, char ** argv){
  std::cout << "0:      ATLAS_STATE <-> EST_ROBOT_STATE\n";
  std::cout << "1:  ATLAS_IMU_BATCH <-> POSE_BODY\n";
  std::cout << "2:      ATLAS_STATE <-> ATLAS_COMMAND\n";
  ConciseArgs parser(argc, argv, "latency-app");
  int period=200;
  parser.add(period, "p", "period", "Counting Period in samples");
  parser.parse();
  cout << "period is: " << period << " samples\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  App app(lcm, period);
  cout << "App ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
