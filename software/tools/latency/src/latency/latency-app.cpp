#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"
#include <latency/latency.hpp>

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg);
  void handleJointCommandMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::joint_command_t * msg);
  void handleUtimeTwoMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_two_t * msg);

  int mode_;
  std::string message_;
  
private:
  Latency* latency_;  
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, int mode_): _lcm(_lcm),mode_(mode_){
  latency_ = new Latency();  
  
  if ( mode_==0){
    _lcm->subscribe("TRUE_ROBOT_STATE", &App::handleRobotStateMsg, this); //
    _lcm->subscribe("JOINT_COMMANDS",&App::handleJointCommandMsg,this);
    message_ = "TRAN";
  }else if(mode_==1){
    _lcm->subscribe("EST_ROBOT_STATE", &App::handleRobotStateMsg, this); //
    _lcm->subscribe("JOINT_COMMANDS",&App::handleJointCommandMsg,this);
    message_ = "ESTM";
  }else if(mode_==2){
    _lcm->subscribe("EST_ROBOT_STATE", &App::handleRobotStateMsg, this); //
    _lcm->subscribe("LATENCY_CONTROL_A",&App::handleUtimeTwoMsg,this);
    message_ = "CNTA";
  }else if(mode_==3){
    _lcm->subscribe("EST_ROBOT_STATE", &App::handleRobotStateMsg, this); //
    _lcm->subscribe("LATENCY_CONTROL_B",&App::handleUtimeTwoMsg,this);
    message_ = "CNTB";
  }else if(mode_==4){
    _lcm->subscribe("EST_ROBOT_STATE", &App::handleRobotStateMsg, this); //
    _lcm->subscribe("LATENCY_CONTROL_C",&App::handleUtimeTwoMsg,this);
    message_ = "CNTC";
  }else if(mode_==5){
    _lcm->subscribe("EST_ROBOT_STATE", &App::handleRobotStateMsg, this); //
    _lcm->subscribe("LATENCY_CONTROL_D",&App::handleUtimeTwoMsg,this);
    message_ = "CNTD";
  }
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void App::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg){
  latency_->add_from(msg->utime, _timestamp_now() );
}

void App::handleJointCommandMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::joint_command_t * msg)  {
  latency_->add_to(msg->utime, _timestamp_now(), message_ );
}

void App::handleUtimeTwoMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_two_t * msg)  {
  latency_->add_to(msg->utime_sim, _timestamp_now(), message_ );
}

int main (int argc, char ** argv){
  std::cout << "0: TRUE_ROBOT_STATE <-> JOINT_COMMANDS\n";
  std::cout << "1:  EST_ROBOT_STATE <-> JOINT_COMMANDS\n";
  std::cout << "2:  EST_ROBOT_STATE <-> LATENCY_CONTROL_A\n";
  std::cout << "3:  EST_ROBOT_STATE <-> LATENCY_CONTROL_B\n";
  std::cout << "4:  EST_ROBOT_STATE <-> LATENCY_CONTROL_C\n";
  std::cout << "5:  EST_ROBOT_STATE <-> LATENCY_CONTROL_D\n";
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  int mode=0;
  parser.add(mode, "m", "mode", "Mode [0,1]");
  parser.parse();
  cout << "mode is: " << mode << "\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  App app(lcm, mode);
  cout << "App ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}