#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/bot_core/atlas_command_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/utime_two_t.hpp"
#include "lcmtypes/drc/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/drc/double_array_t.hpp"
#include "lcmtypes/bot_core/pose_t.hpp"
#include <drc_utils/joint_utils.hpp>

#include <chrono>
#include <thread>

#include <ConciseArgs>
using namespace std;

class App
{
public:
  App(boost::shared_ptr<lcm::LCM> &_lcm, std::string channel_from_, std::string channel_to_);
  ~App() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  void MsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_state_t * msg);
  void RobotStateMsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg);

private:
  std::string channel_from_, channel_to_;
  JointUtils joint_utils_;
  
  bot_core::atlas_command_t cmd_msg_;
  
  double sum_total_;
  int64_t tic_prev_;
};

App::App(boost::shared_ptr<lcm::LCM> &_lcm, std::string channel_from_, std::string channel_to_):
    _lcm(_lcm),channel_from_(channel_from_),channel_to_(channel_to_){

  if (channel_from_ == "ATLAS_STATE"){
    std::cout << "a\n";
    lcm::Subscription* sub0 = _lcm->subscribe( channel_from_ , &App::MsgHandler, this);
    sub0->setQueueCapacity(1);
  } else if(channel_from_ == "EST_ROBOT_STATE"){
    std::cout << "b\n";
    lcm::Subscription* sub1 = _lcm->subscribe( channel_from_ , &App::RobotStateMsgHandler, this);
    sub1->setQueueCapacity(1); 
  }
  
  
  
  std::vector<std::string> atlas_joints_names = joint_utils_.atlas_joint_names; 
  bot_core::atlas_command_t cmd_msg_here;
  cmd_msg_here.joint_names = atlas_joints_names;
  cmd_msg_here.position.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.velocity.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.effort.assign ( atlas_joints_names.size()  ,0.);
  
  cmd_msg_here.k_q_p.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.k_q_i.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.k_qd_p.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.k_f_p.assign ( atlas_joints_names.size()  ,0.);
  
  cmd_msg_here.ff_qd.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.ff_qd_d.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.ff_f_d.assign ( atlas_joints_names.size()  ,0.);
  cmd_msg_here.ff_const.assign ( atlas_joints_names.size()  ,0.);
  
  cmd_msg_here.k_effort.assign ( atlas_joints_names.size()  ,0);
  cmd_msg_here.num_joints = atlas_joints_names.size();	
  
  cmd_msg_ = cmd_msg_here;
  
  sum_total_=0;
}

// same as bot_timestamp_now():
static int64_t _timestamp_now(){
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


void App::RobotStateMsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg){
  //std::cout << "got msg from "<< msg->utime << " | " << chan << "\n";
  int64_t tic0 = _timestamp_now();
  // 30000 gives 5.4 msec in latency app but 2msec from tic/toc and 500Hz in spy
 
  //int64_t a = tic0 - tic_prev_;
// std::cout << "                       " << a << "\n";
// tic_prev_ = tic0;
 
 if (1==1){
  
  for (int j =0; j <30000; j++){
    sum_total_ = 0;
  for (int i=0; i <100; i++){
   sum_total_= sum_total_ + sqrt(10); 
  }
  }
  
}
  
//  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  
  cmd_msg_.utime = msg->utime;
  _lcm->publish( ("ATLAS_COMMAND") , 	&cmd_msg_);
  
  int64_t b = (_timestamp_now() - tic0);
  std::cout << b << "\n";
  
}


void App::MsgHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::atlas_state_t * msg){
  // std::cout << "got msg from "<< msg->utime << " | " << chan << "\n";
  
  /*
  for (int k =0; k <1000000; k++){
  for (int j =0; j <1000000; j++){
  int sum_total = 0;
  for (int i=0; i <1000000; i++){
   sum_total= sum_total + i; 
  }  
  }
  }
  */
  
  //std::this_thread::sleep_for(std::chrono::milliseconds(3));
  
  cmd_msg_.utime = msg->utime;
  _lcm->publish( ("ATLAS_COMMAND") , 	&cmd_msg_);
  
}

int main (int argc, char ** argv){
  ConciseArgs parser(argc, argv, "latency-app");
  std::string channel_from="ATLAS_STATE";
  std::string channel_to="ATLAS_COMMAND";  
  parser.add(channel_from, "f", "channel_from", "From channel");
  parser.add(channel_to, "t", "channel_to", "To channel");  
  parser.parse();
  cout << "channel_from is: " << channel_from << "\n"; 
  cout << "channel_to is: " << channel_to << "\n"; 
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  App app(lcm, channel_from, channel_to);
  cout << "App ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
