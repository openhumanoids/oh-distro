#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/retired_lcmtypes.hpp>

#include <ConciseArgs>

using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    
    void retired_robot_urdf_29oct2013_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  retired::robot_urdf_29oct2013_t* msg);

    void retired_atlas_state_28oct2013_handler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  retired::atlas_state_28oct2013_t* msg);

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
      
  lcm_->subscribe("RETIRED_ROBOT_MODEL",&Pass::retired_robot_urdf_29oct2013_handler,this);

  lcm_->subscribe("RETIRED_ATLAS_STATE",&Pass::retired_atlas_state_28oct2013_handler,this);    
}

void Pass::retired_robot_urdf_29oct2013_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  retired::robot_urdf_29oct2013_t* msg){
  drc::robot_urdf_t out;
  out.utime = msg->utime;
  out.robot_name = msg->robot_name;
  out.urdf_xml_string = msg->urdf_xml_string;  
  out.left_hand = 6;
  out.right_hand = 7;

  lcm_->publish("ROBOT_MODEL", &out);
}

void Pass::retired_atlas_state_28oct2013_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  retired::atlas_state_28oct2013_t* msg){
  drc::atlas_state_t out;
  out.utime = msg->utime;
  out.num_joints = msg->num_joints;
  // The joint names are dropped
  out.joint_position = msg->joint_position;
  out.joint_velocity = msg->joint_velocity;
  out.joint_effort = msg->joint_effort;
  out.force_torque = msg->force_torque;
  lcm_->publish("ATLAS_STATE", &out);
}


int main(int argc, char ** argv) {
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm);
  cout << "Tool ready" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}

