#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>      // std::stringstream
#include <algorithm>

#include <sys/time.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/drc_lcmtypes.hpp>


using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
        
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    void commandHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_command_t* msg);   
    
    void trsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);   

    void hangupHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::utime_t* msg);

    int64_t last_com_utime_;
    int64_t last_com_wall_utime_;
    int64_t last_trs_utime_;
    int64_t last_trs_wall_utime_;

};


Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_): 
    lcm_(lcm_){
  
  lcm_->subscribe( "ATLAS_COMMAND" ,&Pass::commandHandler,this);
  lcm_->subscribe( "TRUE_ROBOT_STATE" ,&Pass::trsHandler,this);  
  lcm_->subscribe( "HANGUP_CONTROL" ,&Pass::hangupHandler,this);  
}


// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void Pass::trsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg){
 

//  std::cout << "got TRS\n";
  last_trs_utime_ = msg->utime;
  last_trs_wall_utime_ = _timestamp_now(); 
  int64_t diff =msg->utime - last_com_utime_;
  if ((diff  > 16000) && (diff  < 100000) ){  
    // more than a threshold but less than a safety
    std::cout << "get BDI diff: "<< diff<< " | " << msg->utime << " ====================\n";

    int n =28;
    drc::atlas_command_t mout;
    mout.utime =msg->utime;
    mout.num_joints = n;
    mout.position.assign (n,-1.0); 
    mout.velocity.assign (n,-1.0);// n values, all -1
    mout.effort.assign (n,-1.0); 
    mout.kp_position.assign (n,-1.0); 
    mout.ki_position.assign (n,-1.0); 
    mout.kd_position.assign (n,-1.0); 
    mout.kp_velocity.assign (n,-1.0); 
    mout.k_effort.assign (n,-0);  // ie no control output - use BDI
    mout.i_effort_min.assign( n, -1.0);
    mout.i_effort_max.assign( n, -1.0);
//    mout.desired_controller_period_ms = 5;
    lcm_->publish( "ATLAS_COMMAND_HANGUP", &mout);

  }
   
}


void Pass::commandHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::atlas_command_t* msg){
//  std::cout << "got COM\n";
  last_com_utime_ = msg->utime;
  last_com_wall_utime_ = _timestamp_now();
}


void Pass::hangupHandler(const lcm::ReceiveBuffer* rbuf, 
                        const std::string& channel, const  drc::utime_t* msg){
  std::cout << "got Hangup Command\n";


}

int main(int argc, char ** argv) {

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "control-hangup Ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}


// if of size 3:
// push_back  ... puts in [2] and removes [0]
// pu

//boost circular buffer
// signal tap.cpp
