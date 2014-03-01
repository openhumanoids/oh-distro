#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <ConciseArgs>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_state_extra_t.hpp"
#include "lcmtypes/multisense.hpp"

using namespace std;
#define DO_TIMING_PROFILE FALSE


///////////////////////////////////////////////////////////////
class state_sync{
  public:
    state_sync(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~state_sync(){
    }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    void atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg);
    void atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg);
    
    drc::atlas_state_t as;
    bool init_;
    
    int counter_;
};   


/////////////////////////////////////

state_sync::state_sync(boost::shared_ptr<lcm::LCM> &lcm_):
   lcm_(lcm_){
  lcm_->subscribe("ATLAS_STATE",&state_sync::atlasHandler,this);  
  lcm_->subscribe("ATLAS_STATE_EXTRA",&state_sync::atlasExtraHandler,this);  
  
  init_ = false;
  counter_=0;
}


void state_sync::atlasExtraHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_extra_t* msg){
  //std::cout << "got atlasExtraHandler\n";
  //std::cout << as.utime << " and " << msg->utime << "\n";
  
  
  
  
  if (msg->utime == as.utime){
    /*
    if (!init_){
      std::stringstream ss;
      ss << "utime, ";
      for (int i=16; i < msg->num_joints ; i++){
        ss << as.joint_name[i] << ", ";
      }      
      for (int i=16; i < msg->num_joints ; i++){
        ss << as.joint_name[i] << ", ";
      }      
      std::cout << ss.str() << "\n";
      init_ = true;
    } */
    
    /*
    if (counter_<30){
      counter_++; 
      return;
    }else{
      counter_ =0; 
    }
    */
    
    
    
    std::stringstream ss;
    ss << (long) msg->utime << ", ";
    for (int i=16; i < msg->num_joints ; i++){
      ss << as.joint_position[i] << ", ";
    }
    for (int i=16; i < msg->num_joints ; i++){
      ss << msg->joint_position_out[i] << ", ";
    }
    std::cout << ss.str() << "\n";
  }  
  

}

void state_sync::atlasHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::atlas_state_t* msg){
  //std::cout << "got atlasHandler\n";
  as = *msg;   
}


int
main(int argc, char ** argv){
  /*
  bool standalone_head = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(standalone_head, "l", "standalone_head","Standalone Head");
  opt.parse();
  */

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_sync app(lcm);
  while(0 == lcm->handle());
  return 0;
}

