/*
- Designer will publish:
http://docs.ros.org/indigo/api/trajectory_msgs/html/msg/JointTrajectory.html
- Designer will receive: (with root link as 0,0,0)
http://docs.ros.org/indigo/api/sensor_msgs/html/msg/JointState.html
*/
#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/robotiqhand/command_t.hpp"
#include <trajectory_msgs/JointTrajectory.h>
#include <ipab_msgs/PlannerRequest.h>
#include <ipab_msgs/RobotiqCommand.h>
#include <std_srvs/Empty.h> // TODO: replace with Trigger once it's available

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    ros::NodeHandle* rosnode;

    ros::ServiceClient rosserviceclient_Emergency_Release_;

    void handCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const robotiqhand::command_t* msg);
    ros::Publisher hand_command_pub_;
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  lcm_->subscribe("ROBOTIQ_LEFT_COMMAND",&LCM2ROS::handCommandHandler, this);
  hand_command_pub_ = nh_.advertise<ipab_msgs::RobotiqCommand>("/gripper/ddapp_command",10);

  rosserviceclient_Emergency_Release_ = nh_.serviceClient<std_srvs::Empty>("/gripper/sdh_controller/emergency_stop"); // TODO: change to trigger

  rosnode = new ros::NodeHandle();
}

void LCM2ROS::handCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const robotiqhand::command_t* msg) {
  ROS_ERROR("LCM2ROS_SDH got hand command");

  // TODO: implement engaging on receiving this flag as true and have an auto-timeout to disengage

  // Check whether to perform an emergency release
  if (msg->emergency_release == 1) {    
    std_srvs::Empty er_trigger; // TODO: temporary until std_srvs::Trigger is available
    if (rosserviceclient_Emergency_Release_.call(er_trigger)) {
      ROS_ERROR("Emergency release activated");
    } else {
      ROS_ERROR("Emergency release NOT activated");
    }
    return;
  }

  ipab_msgs::RobotiqCommand m;
  m.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  
  m.activate = (bool) msg->activate;
  m.emergency_release = (bool) msg->emergency_release;
  m.do_move = (bool) msg->do_move;

  m.mode = msg->mode;
  m.position = msg->position;
  m.force = msg->force;
  m.velocity = msg->velocity;

  m.ifc = (bool) msg->ifc;
  m.positionA = msg->positionA;
  m.positionB = msg->positionB;
  m.positionC = msg->positionC;

  m.isc = (bool) msg->isc;
  m.positionS = msg->positionS;

  hand_command_pub_.publish(m);

  // TODO: Implement direct call of driver services
  /*if (msg->do_move == 1) {
    // Command position, force, and velocity (TODO: only position implemented right now)

    // TODO: implement different modes
    switch(msg->mode) {
      // -1 - ignore (use previous mode)
      //  0 - basic (normal)
      //  1 - pinch
      //  2 - wide
      //  3 - scissor
    }

    // TODO: individual finger control and individual scissor control not yet implemented
    // Individual finger control
    // 1 - active
    // 0 - inactive
    // int8_t ifc;
    // int16_t positionA;
    // int16_t positionB;
    // int16_t positionC;

    // Indiviual scissor control
    // 1 - active
    // 0 - inactive
    // int8_t isc;
    // int16_t positionS;
  } else {
    ROS_WARN("Setting modes not supported - only basic mode supported!");
  }*/
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"lcm2ros_sdh",ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  ros::NodeHandle nh;

  LCM2ROS handlerObject(lcm, nh);
  cout << "\nlcm2ros_sdh translator ready\n";
  ROS_ERROR("LCM2ROS_SDH Translator Ready");

  while(0 == lcm->handle());
  return 0;
}
