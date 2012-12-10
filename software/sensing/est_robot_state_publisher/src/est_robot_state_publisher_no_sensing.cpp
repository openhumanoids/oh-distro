#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/bot_core.hpp"

#include <time.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ConciseArgs>

using namespace Eigen;
using namespace std;

// normally disributied pseudo random numbers:
boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
    randn(boost::mt19937(time(0)),
              boost::normal_distribution<>());

class Handler
{
public:
  Handler(bool add_noise_,double trans_std_dev_, double rot_std_dev_):
      add_noise_(add_noise_),trans_std_dev_(trans_std_dev_) ,rot_std_dev_(rot_std_dev_)  {
     init_dr_pose = false;
  }

  ~Handler() {}
  lcm::LCM _lcm;
  
  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drc::robot_state_t * msg);

  void send_pose(drc::position_3d_t &origin, int64_t utime, std::string channel);
  void add_noise_to_pose(drc::robot_state_t &rstate);
  Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll);
  void set_dr_pose(drc::robot_state_t &rstate);
  
private:
  bool add_noise_;
  bool init_dr_pose;
  Eigen::Isometry3d dr_pose;
  Eigen::Isometry3d last_true_pose;
  
  double trans_std_dev_;
  double rot_std_dev_;
};

void print_Isometry3d(Eigen::Isometry3d pose, std::stringstream &ss){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<" | " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
}


Eigen::Quaterniond Handler::euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

void Handler::set_dr_pose(drc::robot_state_t &rstate){
  drc::position_3d_t o_pos = rstate.origin_position;
  Eigen::Quaterniond rf(o_pos.rotation.w, o_pos.rotation.x, o_pos.rotation.y, o_pos.rotation.z);
  dr_pose.setIdentity();
  dr_pose.translation()  << o_pos.translation.x , o_pos.translation.y , o_pos.translation.z;
  dr_pose.rotate(rf);  
  
  // Also set the pose to be the last true pose
  last_true_pose = dr_pose;
}


void Handler::add_noise_to_pose(drc::robot_state_t &rstate){
  // 1. Create eigen pose from robot position:
  drc::position_3d_t o_pos = rstate.origin_position;
  Eigen::Quaterniond rf(o_pos.rotation.w, o_pos.rotation.x, o_pos.rotation.y, o_pos.rotation.z);
  Eigen::Isometry3d pose_in;
  pose_in.setIdentity();
  pose_in.translation()  << o_pos.translation.x , o_pos.translation.y , o_pos.translation.z;
  pose_in.rotate(rf);

  // 2. Determin the delta pose and apply it to the stored motion estimate:
  Eigen::Isometry3d delta_pose = last_true_pose.inverse()*pose_in;
  dr_pose = dr_pose*delta_pose;

  
  // Add normally disributied noise to the position and euler angles:
  // TODO: make values per unit time
  // TODO: determine if this value is actually the stddev or just a multiple of it
  // TODO: store as a variable:
  
  // Apply noise to ypr:
  double ypr[]= {rot_std_dev_* randn() , rot_std_dev_* randn() , rot_std_dev_* randn() };
  double xyz[]= {trans_std_dev_*randn(),trans_std_dev_*randn() ,trans_std_dev_* randn()};
  if (1==1){ // use 2d only for simplification
    ypr[1]= 0;
    ypr[2]= 0;
    xyz[2]= 0;
  }

  Eigen::Isometry3d noise_dpose;
  noise_dpose.setIdentity();
  noise_dpose.translation() << xyz[0], xyz[1] , xyz[2] ;
  Eigen::Quaterniond m = euler_to_quat(ypr[0], ypr[1], ypr[2]);
  noise_dpose.rotate(m);
  dr_pose =dr_pose*noise_dpose;

  if (1==0){//(VERBOSE_TXT){
    std::stringstream ss5;
    print_Isometry3d(noise_dpose,ss5);
    std::cout << "noise_dpose: " << ss5.str() << "\n";
  }

  // 3. Create output:  
  drc::position_3d_t o_pos_out;
  o_pos_out.translation.x = dr_pose.translation().x();
  o_pos_out.translation.y = dr_pose.translation().y();
  o_pos_out.translation.z = dr_pose.translation().z();
  Eigen::Quaterniond r(dr_pose.rotation());
  o_pos_out.rotation.w = r.w();
  o_pos_out.rotation.x = r.x();
  o_pos_out.rotation.y = r.y();
  o_pos_out.rotation.z = r.z();
  rstate.origin_position = o_pos_out;

  last_true_pose = pose_in;
}


void Handler::send_pose(drc::position_3d_t &origin, int64_t utime, std::string channel){
  bot_core::pose_t pose_msg;
  pose_msg.utime = utime;
  pose_msg.pos[0] = origin.translation.x;
  pose_msg.pos[1] = origin.translation.y;
  pose_msg.pos[2] = origin.translation.z;
  pose_msg.orientation[0] = origin.rotation.w;
  pose_msg.orientation[1] = origin.rotation.x;
  pose_msg.orientation[2] = origin.rotation.y;
  pose_msg.orientation[3] = origin.rotation.z;
  _lcm.publish(channel, &pose_msg);
}


void Handler::handleMessage(const lcm::ReceiveBuffer* rbuf,
  const std::string& chan, const drc::robot_state_t * msg){
  
  drc::robot_state_t msgout;
  msgout= *msg;
  send_pose(msgout.origin_position, msgout.utime, "POSE_TRUE"); 
  
  if (add_noise_){
    if (!init_dr_pose){
     set_dr_pose(msgout);
     init_dr_pose = true;
    }else{
      add_noise_to_pose(msgout);
    }
  }
  
  _lcm.publish("EST_ROBOT_STATE", &msgout);
  send_pose(msgout.origin_position, msgout.utime, "POSE"); 
  // TODO: also publish the CAMERA_STATE pose_t message

}

int main (int argc, char ** argv)
{
  bool add_noise;
  double trans_std_dev = 0.005;
  double rot_std_dev = 0.005;
  ConciseArgs opt(argc, argv);
  opt.add(add_noise, "n", "noise", "add noise to the position estimate");
  opt.add(trans_std_dev, "t", "trans_noise", "translational noise std dev");
  opt.add(rot_std_dev, "r", "rot_noise", "rotational noise std dev");  
  opt.parse();
  cout << "add_noise: " << add_noise << "\n";
  cout << "trans_std_dev: " << trans_std_dev << "\n";
  cout << "rot_std_dev: " << rot_std_dev << "\n";

  lcm::LCM _lcm;
  if(!_lcm.good())
    return 1;

  Handler handlerObject(add_noise,trans_std_dev,rot_std_dev);
  _lcm.subscribe("TRUE_ROBOT_STATE", &Handler::handleMessage,
    &handlerObject);

  while(0 == _lcm.handle());
  return 0;
}