#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"

#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/drc/utime_two_t.hpp"
#include "lcmtypes/drc/double_array_t.hpp"

#include "lcmtypes/pronto.hpp"

#include <latency/latency.hpp>

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, int period_);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleAtlasStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::joint_state_t * msg);
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::robot_state_t * msg);
  void handleCommandMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::atlas_command_t * msg);

  void handleIMUBatch(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::kvh_raw_imu_batch_t * msg);
  void handlePoseBody(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t * msg);

  // GPF:
  void handleSES(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::filter_state_t * msg);
  void handleGPF(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::indexed_measurement_t * msg);
  void handleLidar(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::planar_lidar_t * msg);

  int period_;
  
private:
  std::vector<Latency*> lats_;
  
  std::vector <float> lat_time_;
  std::vector <float> lat_msgs_;
  int counter_;

  bool measure_gpf_;
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int period_):
    _lcm(_lcm),period_(period_){

  // 0 AS  ERS
  // 1 IMU POSE_BODY
  // 2 ERS AC
  // 3 AS  ERS
  // 4 AS  STATE_EST_STATE
  // 5 AS  GPF
  // 6 SCN GPF

  Latency* a_lat0 = new Latency(period_);
  lats_.push_back(a_lat0) ; //
  Latency* a_lat1 = new Latency(period_);
  lats_.push_back(a_lat1) ;
  Latency* a_lat2 = new Latency(period_);
  lats_.push_back(a_lat2) ;
  Latency* a_lat3 = new Latency(period_);
  lats_.push_back(a_lat3) ;
  Latency* a_lat4 = new Latency(period_);
  lats_.push_back(a_lat4) ;
  Latency* a_lat5 = new Latency(period_);
  lats_.push_back(a_lat5) ;
  Latency* a_lat6 = new Latency(period_);
  lats_.push_back(a_lat6) ;
  
  lat_time_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  lat_msgs_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  _lcm->subscribe("ATLAS_STATE", &App::handleAtlasStateMsg, this);
  _lcm->subscribe("EST_ROBOT_STATE",&App::handleRobotStateMsg,this);
  _lcm->subscribe("ATLAS_COMMAND",&App::handleCommandMsg,this);
  _lcm->subscribe("ATLAS_IMU_BATCH", &App::handleIMUBatch, this);
  _lcm->subscribe("POSE_BODY", &App::handlePoseBody, this);

  measure_gpf_ = false;
  if (measure_gpf_){
    _lcm->subscribe("STATE_ESTIMATOR_STATE", &App::handleSES, this);
    _lcm->subscribe("GPF_MEASUREMENT", &App::handleGPF, this);
    _lcm->subscribe("SCAN", &App::handleLidar, this);
  }

  counter_=0;

}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void App::handleAtlasStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::joint_state_t * msg){
  int64_t utime_now = _timestamp_now();
  lats_[0]->add_from(msg->utime,  utime_now);
  lats_[3]->add_from(msg->utime,  utime_now);
  lats_[4]->add_from(msg->utime,  utime_now);

  if (measure_gpf_){
  lats_[5]->add_from(msg->utime,  utime_now);
  }
}

void App::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::robot_state_t * msg){
  int64_t utime_now = _timestamp_now();
  bool new_data = lats_[0]->add_to(msg->utime, utime_now, "SYNC", lat_time_[0], lat_msgs_[0] );
  lats_[2]->add_from(msg->utime, utime_now );
  
  
  if (new_data){
    /*
    if (counter_% 10==0){
      std::cout << "AST-ERS | IMU-SE  | ERS-CMD | AST-CMD"
                << "   ||   "
                << "AST-ERS | IMU-SE  | ERS-CMD | AST-CMD\n";//   <msec|msg>\n";
    }
    
    std::cout.precision(5);
    std::cout.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed
    std::cout << lat_time_[0] << " | " << lat_time_[1] << " | "  << lat_time_[2] << " | "  << lat_time_[3] << "   ||   "
              << lat_msgs_[0] << " | " << lat_msgs_[1] << " | "  << lat_msgs_[2] << " | "  << lat_msgs_[3] << "\n";
      */

    drc::double_array_t msgout;
    msgout.utime = utime_now;
    msgout.num_values = 7;
    msgout.values = { lat_time_[0],  lat_time_[1], lat_time_[2], lat_time_[3], lat_time_[4], lat_time_[5], lat_time_[6]};
    _lcm->publish( ("LATENCY") , &msgout);

    counter_++;
  }
}

void App::handleCommandMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::atlas_command_t * msg)  {
  lats_[2]->add_to(msg->utime, _timestamp_now(), "CTRL", lat_time_[2], lat_msgs_[2] );
  lats_[3]->add_to(msg->utime, _timestamp_now(), "FULL", lat_time_[3], lat_msgs_[3] );
}

void App::handleIMUBatch(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::kvh_raw_imu_batch_t * msg){
  lats_[1]->add_from(msg->utime, _timestamp_now() );
}
void App::handlePoseBody(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t * msg)  {
  lats_[1]->add_to(msg->utime, _timestamp_now(),  "SEST" , lat_time_[1], lat_msgs_[1]);
}



/// Measure GPF:
void App::handleSES(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::filter_state_t * msg)  {
  lats_[4]->add_to(msg->utime, _timestamp_now(),  "GPFI" , lat_time_[4], lat_msgs_[4]);
}
void App::handleGPF(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::indexed_measurement_t * msg)  {
  // utime is paired with lidar i.e. SCAN
  // state_utime is paired with STATE_ESTIMATE_STATE
  lats_[5]->add_to(msg->state_utime, _timestamp_now(),  "GPFO" , lat_time_[5], lat_msgs_[5]);
  lats_[6]->add_to(msg->utime, _timestamp_now(),  "LIDR" , lat_time_[6], lat_msgs_[6]);
}
void App::handleLidar(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const bot_core::planar_lidar_t * msg){
  lats_[6]->add_from(msg->utime, _timestamp_now() );
}

int main (int argc, char ** argv){
  std::cout << "0: CORE_ROBOT_STATE <-> EST_ROBOT_STATE\n";
  std::cout << "1:  ATLAS_IMU_BATCH <-> POSE_BODY\n";
  std::cout << "2:  EST_ROBOT_STATE <-> ATLAS_COMMAND\n";  
  std::cout << "3: CORE_ROBOT_STATE <-> ATLAS_COMMAND\n";
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
