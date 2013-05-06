#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <boost/shared_ptr.hpp>
#include <ConciseArgs>

using namespace std;

class StatePub
{
public:
  StatePub(boost::shared_ptr<lcm::LCM> &_lcm);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> _lcm;
  
  void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * TRUE_state_msg);
  void triggerHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::data_request_t* msg);

private:
  
  /// used for Trigger based message demand:
  drc::robot_state_t _last_est_state;    
  void sendTriggerOutput();  
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &_lcm): _lcm(_lcm){
      
  _lcm->subscribe("EST_ROBOT_STATE", &StatePub::handleRobotStateMsg, this); //
  _lcm->subscribe("TRIGGER_STATE",&StatePub::triggerHandler,this);
  _last_est_state.utime = 0; // used to indicate no message recieved yet
}


void StatePub::handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
  const std::string& chan, const drc::robot_state_t * msg){
  _last_est_state = *msg;
}

// This function can also be used to publish on 
// a regular basis e.g. if set to publish at 1Hz
void StatePub::sendTriggerOutput(){
  
  if (_last_est_state.utime==0){     return;   } // if no msg recieved yet then ignore output command
  drc::minimal_robot_state_t msgout;
  msgout.utime = _last_est_state.utime;
  msgout.origin_position = _last_est_state.origin_position;
  msgout.num_joints = _last_est_state.num_joints;
  msgout.joint_position = _last_est_state.joint_position;
  _lcm->publish("EST_ROBOT_STATE_MINIMAL", &msgout);        
  cout << "Sending Triggered Message\n";
}

void StatePub::triggerHandler(const lcm::ReceiveBuffer* rbuf, 
                    const std::string& channel, const  drc::data_request_t* msg){
  sendTriggerOutput();
}

int main (int argc, char ** argv){
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  StatePub app(lcm);
  cout << "StatePub ready"<< endl;
  while(0 == lcm->handle());
  return 0;
}
