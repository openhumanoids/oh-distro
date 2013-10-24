#include <stdio.h>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <ConciseArgs>
#include <string>
#include <deque>
#include <pointcloud_tools/pointcloud_math.hpp>

using namespace std;


class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }
    
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    
    //////////////////////////////////////
    void robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
                             const std::string& channel, const  drc::robot_state_t* msg);    
    
};   

App::App(boost::shared_ptr<lcm::LCM> &lcm_):
   lcm_(lcm_){
  verbose_=false;
  
  lcm_->subscribe("EST_ROBOT_STATE",  &App::robotStateHandler, this);  

  
}

void App::robotStateHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  drc::robot_state_t* msg){
  
  std::cout << "EST_ROBOT_STATE - x, y, z, yaw\n";
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
  double rpy[3];
  quat_to_euler( quat , rpy[0], rpy[1], rpy[2]);
  
  std::cout << msg->pose.translation.x  << ", "
            << msg->pose.translation.y  << ", "
            << msg->pose.translation.z  << ", "
            << rpy[2]  << "\n";     
}



 
int main(int argc, char *argv[]){
/*  bool lhand=false, rhand=false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(rhand, "r", "right","Process right hand message");
  opt.add(lhand, "l", "left","Process left hand message");
  
  opt.parse();
  */
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  App app(lcm);
  while(0 == lcm->handle());
  return 0;
}
