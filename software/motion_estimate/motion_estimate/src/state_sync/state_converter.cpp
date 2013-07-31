#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <limits>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>

#include <map>

#include "lcmtypes/drc_lcmtypes.hpp"
#include "lcmtypes/multisense.hpp"


#include <ConciseArgs>

using namespace std;
#define DO_TIMING_PROFILE FALSE

/////////////////////////////////////
struct Joints { 
  std::vector<float> position;
  std::vector<float> velocity;
  std::vector<float> effort;
  std::vector<std::string> name;
};


///////////////////////////////////////////////////////////////
class state_converter{
  public:
    state_converter(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~state_converter(){
    }
    void Identity();
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    void stateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::state_t* msg);

    void publishRobotState(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints(drc::state_t& msg_out, Joints joints);
    
    
    void publishRobotState_VRC(int64_t utime_in, const  drc::force_torque_t& msg);
    void appendJoints_VRC(drc::robot_state_t& msg_out, Joints joints);
    drc::contact_state_t setContacts_VRC(const  drc::force_torque_t& msg);

    Joints head_joints_;
    Joints atlas_joints_;
    Joints sandia_left_joints_;
    Joints sandia_right_joints_;
    
};    


state_converter::state_converter(boost::shared_ptr<lcm::LCM> &lcm_):lcm_(lcm_){
  lcm_->subscribe("EST_ROBOT_STATE_NEW",&state_converter::stateHandler,this);  
}


void state_converter::stateHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::state_t* msg){
  std::cout << "got stateHandler\n";
  
  drc::robot_state_t robot_state_msg;
  robot_state_msg.utime = msg->utime;
  robot_state_msg.robot_name = "atlas";
  
  // Pelvis Pose:
  robot_state_msg.origin_position = msg->origin_position;
  robot_state_msg.origin_twist    = msg->origin_twist;
  for(int i = 0; i < 6; i++)  {
    for(int j = 0; j < 6; j++) {
      robot_state_msg.origin_cov.position_cov[i][j] = 0;
      robot_state_msg.origin_cov.twist_cov[i][j] = 0;
    }
  }

  
  robot_state_msg.joint_name = msg->joint_name;
  robot_state_msg.joint_position = msg->joint_position;
  robot_state_msg.joint_velocity = msg->joint_velocity;
  robot_state_msg.measured_effort= msg->joint_effort;
  robot_state_msg.num_joints =msg->joint_name.size();
  drc::joint_covariance_t j_cov;
  j_cov.variance = 0;
  for (size_t i = 0; i < msg->joint_name.size(); i++)  {
    robot_state_msg.joint_cov.push_back(j_cov);
  }
  //std::cout << robot_state_msg.joint_name.size() << " Number of Joints\n";
  
  // Limb Sensor states
  robot_state_msg.contacts = setContacts_VRC(msg->force_torque);
  
  lcm_->publish("EST_ROBOT_STATE", &robot_state_msg);    
}


drc::contact_state_t state_converter::setContacts_VRC(const  drc::force_torque_t& msg_in){
  drc::contact_state_t contacts;
  {
    drc::vector_3d_t l_foot_force;
    l_foot_force.x = 0;                       l_foot_force.y = 0;                       l_foot_force.z = msg_in.l_foot_force_z;
    drc::vector_3d_t l_foot_torque;
    l_foot_torque.x = msg_in.l_foot_torque_x; l_foot_torque.y = msg_in.l_foot_torque_y; l_foot_torque.z = 0;
    contacts.id.push_back("l_foot");
    contacts.contact_force.push_back(l_foot_force);
    contacts.contact_torque.push_back(l_foot_torque);  
  }

  
  {
    drc::vector_3d_t r_foot_force;
    r_foot_force.x = 0;                       r_foot_force.y = 0;                       r_foot_force.z = msg_in.r_foot_force_z;
    drc::vector_3d_t r_foot_torque;
    r_foot_torque.x = msg_in.r_foot_torque_x; r_foot_torque.y = msg_in.r_foot_torque_y; r_foot_torque.z = 0;
    contacts.id.push_back("r_foot");
    contacts.contact_force.push_back(r_foot_force);
    contacts.contact_torque.push_back(r_foot_torque);  
  }  

  {
    drc::vector_3d_t l_hand_force;
    l_hand_force.x = msg_in.l_hand_force[0]; l_hand_force.y = msg_in.l_hand_force[1]; l_hand_force.z = msg_in.l_hand_force[2];
    drc::vector_3d_t l_hand_torque;
    l_hand_torque.x = msg_in.l_hand_torque[0]; l_hand_torque.y = msg_in.l_hand_torque[1]; l_hand_torque.z = msg_in.l_hand_torque[2];
    contacts.id.push_back("l_hand");
    contacts.contact_force.push_back(l_hand_force);
    contacts.contact_torque.push_back(l_hand_torque);  
  }
    
  {
    drc::vector_3d_t r_hand_force;
    r_hand_force.x = msg_in.r_hand_force[0]; r_hand_force.y = msg_in.r_hand_force[1]; r_hand_force.z = msg_in.r_hand_force[2];
    drc::vector_3d_t r_hand_torque;
    r_hand_torque.x = msg_in.r_hand_torque[0]; r_hand_torque.y = msg_in.r_hand_torque[1]; r_hand_torque.z = msg_in.r_hand_torque[2];
    contacts.id.push_back("r_hand");
    contacts.contact_force.push_back(r_hand_force);
    contacts.contact_torque.push_back(r_hand_torque);  
  } 
  
  contacts.num_contacts = contacts.id.size();
  
  return contacts;
}

void state_converter::appendJoints_VRC(drc::robot_state_t& msg_out, Joints joints){
}


int
main(int argc, char ** argv){
  bool labels = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(labels, "l", "labels","Frame Labels - show no not");
  opt.parse();
  
  std::cout << "labels: " << labels << "\n";

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  if(!lcm->good())
    return 1;  
  
  state_converter app(lcm);
  while(0 == lcm->handle());
  return 0;
}

