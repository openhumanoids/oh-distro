#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <math.h>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/robot_urdf_t.hpp>
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

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_):
    lcm_(lcm_){
      
  lcm_->subscribe("RETIRED_ROBOT_MODEL",&Pass::retired_robot_urdf_29oct2013_handler,this);  
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

