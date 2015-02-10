#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <limits>

#include "accel_from_position.hpp"
#include <ConciseArgs>

using namespace std;
using namespace boost;
using namespace boost::assign;

// false usually, set true to disable the limiting:
#define DONT_LIMIT_FREQUENCY FALSE

/////////////////////////////////////




accel_from_position::accel_from_position(boost::shared_ptr<lcm::LCM> &lcm_, bool show_labels_, bool show_triads_,
  bool standalone_head_, bool ground_height_, bool bdi_motion_estimate_, bool multisense_sim_):
          lcm_(lcm_), show_labels_(show_labels_), show_triads_(show_triads_),
          standalone_head_(standalone_head_), ground_height_(ground_height_),
          bdi_motion_estimate_(bdi_motion_estimate_), multisense_sim_(multisense_sim_){
            
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
            
  // Vis Config:
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(6001,"imu",5,0) );


  lcm_->subscribe("POSE_BODY",&accel_from_position::pose_handler,this);  
}



void accel_from_position::pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){
  Eigen::Isometry3d left_pos;
  left_pos.setIdentity();

  // determine the position of the imu using libbot
  Eigen::Isometry3d imu_to_local = botframes_cpp_->get_trans_with_utime( botframes_, "imu", "local"  , msg->utime);
  Isometry3dTime imu_to_local_T = Isometry3dTime(msg->utime, imu_to_local );

  pc_vis_->pose_to_lcm_from_list(6001, imu_to_local_T);  

  double dt = (msg->utime - utime_prev_)*1E-6;
  Eigen::Isometry3d imu_delta = imu_to_local_prev_.inverse() *imu_to_local  ;
  


  std::stringstream ss;
  print_Isometry3d(imu_to_local, ss);
  cout << msg->utime << ": " << ss.str()  << "\n";

  Eigen::Vector3d velocity =  imu_delta.translation()/dt;

//  Eigen::Vector3d velocity = (imu_to_local.translation() - imu_to_local_prev_.translation())/dt;

  //  cout << msg->utime << ": " << velocity.transpose()  << " vel\n";

  Eigen::Vector3d accel = ( velocity - imu_to_local_vel_prev_  )/dt;
  //  cout << msg->utime << ": " << accel.transpose()  << " accel\n";


  cout << msg->utime << ": " <<velocity.transpose() << " | " << accel.transpose()  << " accel\n";

  bot_core::pose_t pose_msg;
  pose_msg.utime = msg->utime;
  pose_msg.vel[0] = velocity(0);
  pose_msg.vel[1] = velocity(1);
  pose_msg.vel[2] = velocity(2);  
  lcm_->publish( "POSE_IMU_VELOCITY", &pose_msg);


  microstrain::ins_t out;
  out.utime = msg->utime;
  out.accel[0] = accel(0);
  out.accel[1] = accel(1);
  out.accel[2] = accel(2);
  lcm_->publish( "MICROSTRAIN_INS_DIFF", &out);

  imu_to_local_prev_ = imu_to_local;
  imu_to_local_vel_prev_ = velocity;
  utime_prev_ = msg->utime;
}





int
main(int argc, char ** argv){
  bool labels = false;
  bool triads = false;
  bool standalone_head = false;
  bool ground_height = false;
  bool bdi_motion_estimate = false;
  bool multisense_sim = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(triads, "t", "triads","Frame Triads - show no not");
  opt.add(labels, "l", "labels","Frame Labels - show no not");
  opt.add(ground_height, "g", "ground", "Publish the grounded foot pose");
  opt.add(standalone_head, "s", "standalone_head","Standalone Sensor Head");
  opt.add(bdi_motion_estimate, "b", "bdi","Use POSE_BDI to make frames [Temporary!]");
  opt.add(multisense_sim, "m", "multisense_sim","In sim, publish PRE_SPINDLE_TO_POST_SPINDLE");
  opt.parse();
  if (labels){ // require triads if labels is to be published
    triads=true;
  }
  
  std::cout << "triads: " << triads << "\n";
  std::cout << "labels: " << labels << "\n";

  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM("") );
  if(!lcm->good())
    return 1;  
  
  accel_from_position app(lcm,labels,triads, standalone_head, ground_height, bdi_motion_estimate, multisense_sim);
  while(0 == lcm->handle());
  return 0;
}
