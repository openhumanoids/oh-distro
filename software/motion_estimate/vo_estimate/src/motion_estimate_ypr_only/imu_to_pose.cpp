#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include <lcmtypes/microstrain.hpp>
#include <ConciseArgs>

#include <Eigen/Dense>
#include <Eigen/StdVector>


using namespace std;
using namespace Eigen;


class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string ins_channel_);
    
    ~Pass(){
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string ins_channel_;
    
    void insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  microstrain::ins_t* msg);   
    microstrain::ins_t lidar_msgout_;
};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string ins_channel_):
    lcm_(lcm_), verbose_(verbose_), 
    ins_channel_(ins_channel_){
  lcm_->subscribe( ins_channel_ ,&Pass::insHandler,this);
  cout << "Finished setting up\n";
}


void Pass::insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  microstrain::ins_t* msg){
  Eigen::Quaterniond meas_rot(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
  Eigen::Matrix3d meas_rotation(meas_rot);

  // rotate coordinate frame so that look vector is +X, and up is +Z
  Eigen::Matrix3d M;
  M <<  -1, 0,  0,
         0, 1,  0,
         0, 0, -1;    

  // which is equivalent to:
  /*
  double rpy[] = {M_PI, 0,M_PI};
  double q[4],rot[9];
  roll_pitch_yaw_to_quat (rpy,q);
  quat_to_matrix(q,rot);
  M << rot[0], rot[1], rot[2],
       rot[3], rot[4], rot[5],
       rot[6], rot[7], rot[8];

  cout << rot[0] << " " << rot[1] << " "<< rot[2] << "\n";
  cout << rot[3] << " "<< rot[4] << " "<< rot[5] << "\n";
  cout << rot[6] << " "<< rot[7] << " "<< rot[8] << "\n";
  */

  meas_rotation= M * meas_rotation;
  Eigen::Quaterniond rotation = Eigen::Quaterniond( meas_rotation * M.transpose() );

  bot_core::pose_t pose_msg;
  pose_msg.utime = msg->utime;
  pose_msg.pos[0] =0;
  pose_msg.pos[1] =0;
  pose_msg.pos[2] =1.6;
  pose_msg.orientation[0] = rotation.w();
  pose_msg.orientation[1] = rotation.x();
  pose_msg.orientation[2] = rotation.y();
  pose_msg.orientation[3] = rotation.z();
  lcm_->publish("POSE_BODY", &pose_msg); 
}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "lidar-passthrough");
  bool verbose=false;
  string ins_channel="MICROSTRAIN_INS";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(ins_channel, "l", "ins_channel", "Incoming INS channel");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << ins_channel << " is ins_channel\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  Pass app(lcm,verbose,ins_channel);
  cout << "Ready to convert from imu to pose" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
