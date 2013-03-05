// Republish old robot state message (jan 2013) as new message

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <ConciseArgs>

using namespace std;
using namespace Eigen;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string incoming_channel_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string incoming_channel_;
    
    void robotStateOldHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_prefeb2013_t* msg);   
    
    int vis_counter_; // used for visualization
    int printf_counter_; // used for terminal feedback
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string incoming_channel_):
    lcm_(lcm_), verbose_(verbose_), 
    incoming_channel_(incoming_channel_){
  
  lcm_->subscribe("OLD_TRUE_ROBOT_STATE",&Pass::robotStateOldHandler,this);
  
  vis_counter_ =0;  
  printf_counter_ =0;
  
  cout << "Finished setting up\n";
}

void Pass::robotStateOldHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_prefeb2013_t* msg){
  std::cout << "got data\n";

  drc::robot_state_t msg_out;

  msg_out.utime = msg->utime;
  msg_out.robot_name = msg->robot_name;
  msg_out.origin_position= msg->origin_position;  
  msg_out.origin_twist = msg->origin_twist;   
  msg_out.origin_cov = msg->origin_cov;
  msg_out.num_joints =1;
  msg_out.joint_name= msg->joint_name;
  msg_out.joint_position = msg->joint_position;
  msg_out.joint_velocity = msg->joint_velocity;
  msg_out.measured_effort = msg->measured_effort;
  msg_out.joint_cov = msg->joint_cov;
  msg_out.contacts.num_contacts=0;

  lcm_->publish( ("TRUE_ROBOT_STATE") , &msg_out);        

}

int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=false;
  string incoming_channel="OLD_TRUE_ROBOT_STATE";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(incoming_channel, "l", "incoming_channel", "Incoming LIDAR channel");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << incoming_channel << " is incoming_channel\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,incoming_channel);
  cout << "Ready to filter lidar points" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
