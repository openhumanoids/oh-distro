// Selective lcm2lcm combiner for Edinburgh Kuka Arm including SDH hand
// mfallon
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/joint_state_t.hpp"

using namespace std;

int64_t bot_timestamp_now() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

class App{
public:
  App(boost::shared_ptr<lcm::LCM> &lcm_);
  ~App();

private:
  boost::shared_ptr<lcm::LCM> lcm_;

  void kukaStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::joint_state_t* msg);
  void sdhStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::joint_state_t* msg);
  void broadcastRobotState();

  // Store most recent states of both the hand and the arm, and combine once a new one arrives for each
  boost::mutex mtx_;

  bot_core::joint_state_t kukaState_;
  bot_core::joint_state_t sdhState_;
};

App::App(boost::shared_ptr<lcm::LCM> &lcm_) : lcm_(lcm_) {

  
  lcm_->subscribe("SCHUNK_STATE", &App::sdhStateHandler, this);
  lcm_->subscribe("KUKA_STATE", &App::kukaStateHandler, this);

  // Initialise empty state variables
  kukaState_.joint_position.assign(7 , 0  );
  kukaState_.joint_velocity.assign(7 , 0  );
  kukaState_.joint_effort.assign(7 , 0  );
  kukaState_.num_joints = 7;

  sdhState_.joint_position.assign(8 , 0  );
  sdhState_.joint_velocity.assign(8 , 0  );
  sdhState_.joint_effort.assign(8 , 0  );
  sdhState_.num_joints = 8;
};

App::~App()  {
}

void App::kukaStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::joint_state_t* msg) {
  mtx_.lock();
  kukaState_ = *msg;
  mtx_.unlock();

  // Broadcast new message
  this->broadcastRobotState();
}

void App::sdhStateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::joint_state_t* msg) {
  mtx_.lock();
  sdhState_ = *msg;
  mtx_.unlock();

  // Broadcast new message
  this->broadcastRobotState();
}

void App::broadcastRobotState() {
  // Set up message
  bot_core::robot_state_t msg_out;
  msg_out.utime =  bot_timestamp_now();
  msg_out.pose.translation.x = 0;  msg_out.pose.translation.y = 0;  msg_out.pose.translation.z = 0;
  msg_out.pose.rotation.w = 1;  msg_out.pose.rotation.x = 0;  msg_out.pose.rotation.y = 0;  msg_out.pose.rotation.z = 0;
  int n_joints = 15;
  msg_out.joint_position.assign(n_joints , 0  );
  msg_out.joint_velocity.assign(n_joints , 0  );
  msg_out.joint_effort.assign(n_joints , 0  );
  msg_out.num_joints = n_joints;

  msg_out.joint_name = {"lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint", "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint", "sdh_knuckle_joint", "sdh_thumb_2_joint", "sdh_thumb_3_joint", "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_22_joint", "sdh_finger_23_joint", "sdh_finger_21_joint"};

  // Add individual joint state messages
  int kuka_joints = 7; //kukaState_->joint_position.size();
  int sdh_joints = 8; //sdhState_->joint_position.size();

  mtx_.lock();
  for (int i = 0; i < kuka_joints; i++)  {
    msg_out.joint_position[ i ] = kukaState_.joint_position[ i ];
    msg_out.joint_velocity[ i ] = kukaState_.joint_velocity[ i ];
    msg_out.joint_effort[ i ] = kukaState_.joint_effort[ i ];
  }

  for (int j = 0; j < sdh_joints; j++)  {
    msg_out.joint_position[ j+7 ] = sdhState_.joint_position[ j ];
    msg_out.joint_velocity[ j+7 ] = sdhState_.joint_velocity[ j ];
    msg_out.joint_effort[ j+7 ] = sdhState_.joint_effort[ j ];
  }
  mtx_.unlock();

  lcm_->publish("EST_ROBOT_STATE", &msg_out);
}

int main(int argc, char **argv){
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  App handlerObject(lcm);

  std::cout << "lcm2lcm translator ready\n";
  
  while(0 == lcm->handle());
  return 0;
}
