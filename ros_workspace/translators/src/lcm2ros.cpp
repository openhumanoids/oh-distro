#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/actuator_cmd_t.hpp"
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "lcmtypes/drc/controller_mode_t.hpp"
#include "lcmtypes/drc/joint_command_t.hpp"
#include "lcmtypes/drc/sensor_request_t.hpp"
#include "lcmtypes/drc/simple_grasp_t.hpp"
#include "lcmtypes/drc/system_status_t.hpp"
#include "lcmtypes/drc/twist_t.hpp"
#include "lcmtypes/drc/deprecated_footstep_plan_t.hpp"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <sandia_hand_msgs/SimpleGrasp.h>
#include <map>
#include <ConciseArgs>

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    lcm::LCM lcm_publish_ ;

    std::map<std::string,std::string> jointNameMap;
    int last_command_timestamp;
    
    // DRCSIM 2.6 atlas command API
    void atlasCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::atlas_command_t* msg);
    ros::Publisher atlas_cmd_pub_;
       
    ros::Publisher spindle_speed_pub_, head_fps_pub_, hand_fps_pub_, multisense_sl_fps_pub_;
    void sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg);
    
    void sandiaLHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    void sandiaRHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg);
    ros::Publisher sandia_l_hand_joint_cmd_pub_;
    ros::Publisher sandia_r_hand_joint_cmd_pub_;
    
    // Non-api translations:
    ros::Publisher body_twist_cmd_pub_;
    void actuatorCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::actuator_cmd_t* msg);
    void bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg);
   
    ros::Publisher simple_grasp_pub_right_ , simple_grasp_pub_left_ ;
    void simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::simple_grasp_t* msg);   
    
    // BDI Footstep Messaging 
    ros::Publisher committed_footstep_plan_pub_;
    void committedFootStepPlanHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::deprecated_footstep_plan_t* msg);
    
    // for swapping between BDI and MIT control
    bool use_bdi;
    void controllerModeHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::controller_mode_t* msg);
    
    ros::NodeHandle* rosnode;
    bool have_set_multisense_rate_; // have you set the initial multisesne rate
};


LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
  have_set_multisense_rate_= false;
  
  lcm_->subscribe("CONTROLLER_MODE",&LCM2ROS::controllerModeHandler, this);
       
  /// DRCSIM 2.6 atlas command API
  lcm_->subscribe("ATLAS_COMMAND",&LCM2ROS::atlasCommandHandler,this);  
  // hang up to the bdi controller:
  lcm_->subscribe("ATLAS_COMMAND_HANGUP",&LCM2ROS::atlasCommandHandler,this);  
  atlas_cmd_pub_ = nh_.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command",10, true);

  // by default use MIT control 100%
  use_bdi = false;

  /// Spinning Laser control:
  lcm_->subscribe("SENSOR_REQUEST",&LCM2ROS::sensor_request_Callback,this);
  spindle_speed_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/set_spindle_speed",10);
  hand_fps_pub_ = nh_.advertise<std_msgs::Float64>("/mit/set_hand_fps",10); 
  multisense_sl_fps_pub_ = nh_.advertise<std_msgs::Float64>("/multisense_sl/fps",10);
  
  /// Sandia Hands joint command API
  lcm_->subscribe("L_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaLHandJointCommandHandler,this);  
  sandia_l_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/l_hand/joint_commands",10);
  lcm_->subscribe("R_HAND_JOINT_COMMANDS",&LCM2ROS::sandiaRHandJointCommandHandler,this);  
  sandia_r_hand_joint_cmd_pub_ = nh_.advertise<osrf_msgs::JointCommands>("/sandia_hands/r_hand/joint_commands",10); 

  lcm_->subscribe("SIMPLE_GRASP_COMMAND",&LCM2ROS::simpleGraspCmdHandler,this);  
  simple_grasp_pub_left_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/l_hand/simple_grasp",10); 
  simple_grasp_pub_right_ = nh_.advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp",10); 
//drc_robot.pmd:        exec = "rostopic pub /sandia_hands/r_hand/simple_grasp sandia_hand_msgs/SimpleGrasp  '{closed_amount: 100.0, name: cylindrical}'";

  
  lcm_->subscribe("COMMITTED_FOOTSTEP_PLAN",&LCM2ROS::committedFootStepPlanHandler,this);  
  committed_footstep_plan_pub_ = nh_.advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command",10);
  
  
  lcm_->subscribe("NAV_CMDS",&LCM2ROS::bodyTwistCmdHandler,this);
  body_twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

  // maps joint names from BDI format to sim/VRC format
  jointNameMap["back_bkx"] = "back_ubx";
  jointNameMap["back_bky"] = "back_mby";
  jointNameMap["back_bkz"] = "back_lbz";
  jointNameMap["l_leg_akx"] = "l_leg_lax";
  jointNameMap["l_leg_aky"] = "l_leg_uay";
  jointNameMap["l_leg_kny"] = "l_leg_kny";
  jointNameMap["l_leg_hpx"] = "l_leg_mhx";
  jointNameMap["l_leg_hpy"] = "l_leg_lhy";
  jointNameMap["l_leg_hpz"] = "l_leg_uhz";
  jointNameMap["r_leg_akx"] = "r_leg_lax";
  jointNameMap["r_leg_aky"] = "r_leg_uay";
  jointNameMap["r_leg_kny"] = "r_leg_kny";
  jointNameMap["r_leg_hpx"] = "r_leg_mhx";
  jointNameMap["r_leg_hpy"] = "r_leg_lhy";
  jointNameMap["r_leg_hpz"] = "r_leg_uhz";
  jointNameMap["l_arm_mwx"] = "l_arm_mwx";
  jointNameMap["l_arm_uwy"] = "l_arm_uwy";
  jointNameMap["l_arm_elx"] = "l_arm_elx";
  jointNameMap["l_arm_ely"] = "l_arm_ely";
  jointNameMap["l_arm_shx"] = "l_arm_shx";
  jointNameMap["l_arm_usy"] = "l_arm_usy";
  jointNameMap["r_arm_mwx"] = "r_arm_mwx";
  jointNameMap["r_arm_uwy"] = "r_arm_uwy";
  jointNameMap["r_arm_elx"] = "r_arm_elx";
  jointNameMap["r_arm_ely"] = "r_arm_ely";
  jointNameMap["r_arm_shx"] = "r_arm_shx";
  jointNameMap["r_arm_usy"] = "r_arm_usy";

  last_command_timestamp = -1;

  rosnode = new ros::NodeHandle();
}

void LCM2ROS::committedFootStepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::deprecated_footstep_plan_t* msg) {
  ROS_ERROR("LCM2ROS Sending committed footsteps to BDI sim control");

  // This code is functional but the BDI sim driver keeps its own state estimate
  // which the footsteps have to be relative to
  // ... so this module needs to listen to /atlas/imu
  // and then convert the steps into that relative frame. 
  // this requires making the process listen to ROS and this is incomplete
  // mfallon jan 2014
  
  atlas_msgs::AtlasSimInterfaceCommand msgout;
  msgout.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  msgout.behavior = atlas_msgs::AtlasSimInterfaceCommand::WALK;
  msgout.walk_params.use_demo_walk = false;
  for(size_t i=0; i < 4; i++){
    atlas_msgs::AtlasBehaviorStepData step_queue;
    step_queue.step_index = i+1;
    if (msg->footstep_goals[i].is_right_foot){
      step_queue.foot_index = 1;
    }else{
      step_queue.foot_index = 0;
    }
    step_queue.swing_height = 0.3;
    step_queue.duration = 0.63;
    
    //////////////// These need to be transformed into BDI nav frame
    step_queue.pose.position.x = msg->footstep_goals[i].pos.translation.x;
    step_queue.pose.position.y = msg->footstep_goals[i].pos.translation.y;
    step_queue.pose.position.z = msg->footstep_goals[i].pos.translation.z;
    step_queue.pose.orientation.w = msg->footstep_goals[i].pos.rotation.w;
    step_queue.pose.orientation.x = msg->footstep_goals[i].pos.rotation.x;
    step_queue.pose.orientation.y = msg->footstep_goals[i].pos.rotation.y;
    step_queue.pose.orientation.z = msg->footstep_goals[i].pos.rotation.z;
    msgout.walk_params.step_queue [i] = step_queue;
    ///////////////////////////////////////////////
  }
  committed_footstep_plan_pub_.publish(msgout);
  
}


void LCM2ROS::simpleGraspCmdHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::simple_grasp_t* msg) {
  ROS_ERROR("LCM2ROS Sending got simpele");

  if(ros::ok()) {
    sandia_hand_msgs::SimpleGrasp msgout;
    msgout.name = "cylindrical";
    if (msg->left_state != drc::simple_grasp_t::UNCHANGED) {
      if (msg->left_state == drc::simple_grasp_t::CLOSED) {
        msgout.closed_amount = 100;
      }
      else msgout.closed_amount = 0;
      ROS_ERROR("LCM2ROS Sending simple grasp command (left)");
      simple_grasp_pub_left_.publish(msgout);
    }
    if (msg->right_state != drc::simple_grasp_t::UNCHANGED) {
      if (msg->right_state == drc::simple_grasp_t::CLOSED) {
        msgout.closed_amount = 100;
      }
      else msgout.closed_amount = 0;
      ROS_ERROR("LCM2ROS Sending simple grasp command (right)");
      simple_grasp_pub_right_.publish(msgout);    
    }
  }   
}
  
void LCM2ROS::atlasCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::atlas_command_t* msg) {
//  if (msg->effort[0] == 0){ // assume this is enough to trigger
//    ROS_ERROR("LCM2ROS Handing back control to BDI - effort field zero");
//  }
  
//  if (msg->utime > last_command_timestamp) {
//    ROS_ERROR("NEW COMMAND: %d > %d", msg->utime, last_command_timestamp);
//    last_command_timestamp = msg->utime;

    atlas_msgs::AtlasCommand atlas_command_msg;
    atlas_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
    
    atlas_command_msg.ki_position.resize(msg->num_joints);
    atlas_command_msg.kp_velocity.resize(msg->num_joints);
    atlas_command_msg.i_effort_min.resize(msg->num_joints);
    atlas_command_msg.i_effort_max.resize(msg->num_joints);

    atlas_command_msg.desired_controller_period_ms = msg->desired_controller_period_ms;

    for (int i=0; i<msg->num_joints; i++) {
      //atlas_command_msg.name.push_back("atlas::" + jointNameMap[msg->name[i]]); // must use scoped name
      atlas_command_msg.position.push_back(msg->position[i]);
      atlas_command_msg.velocity.push_back(msg->velocity[i]);
      atlas_command_msg.effort.push_back(msg->effort[i]);

      if (use_bdi) // use the bdi controller, but this does set though the message
        atlas_command_msg.k_effort.push_back(0);
      else
        atlas_command_msg.k_effort.push_back(msg->k_effort[i]);

      atlas_command_msg.kp_position.push_back(msg->k_q_p[i]);
      atlas_command_msg.kd_position.push_back(msg->ff_qd[i]);
    }
    if(ros::ok()) {
      atlas_cmd_pub_.publish(atlas_command_msg);
    } 
//  }
//  else {
//    ROS_ERROR("OLD COMMAND: %d <= %d", msg->utime, last_command_timestamp);
//  }
}  


void LCM2ROS::controllerModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::controller_mode_t* msg) {

  if (msg->mode == msg->BDI) {
    ROS_ERROR("LCM2ROS switching to BDI control");
    drc::system_status_t m;
    m.utime = msg->utime;
    m.system = drc::system_status_t::MESSAGING;
    m.importance = drc::system_status_t::VERY_IMPORTANT;
    m.frequency = drc::system_status_t::LOW_FREQUENCY;

    m.value = "lcm2ros: Switching to BDI control.";  
    lcm_publish_.publish("SYSTEM_STATUS", &m); // for simplicity stick this out

    use_bdi = true;
  } else {
    ROS_ERROR("LCM2ROS switching to MIT control");
    drc::system_status_t m;
    m.utime = msg->utime;
    m.system = drc::system_status_t::MESSAGING;
    m.importance = drc::system_status_t::VERY_IMPORTANT;
    m.frequency = drc::system_status_t::LOW_FREQUENCY;

    m.value = "lcm2ros: Switching to MIT control.";  
    lcm_publish_.publish("SYSTEM_STATUS", &m); // for simplicity stick this out

    use_bdi = false;
  }
}  


// command line dynamic reconfigure:
// rosrun dynamic_reconfigure dynparam set /multisense_sl motor_speed 1
// rosrun dynamic_reconfigure dynparam set /multisense_sl fps 30
// C++ control of dynamic reconfigure is not implemented - need to use system call:
// http://ros.org/wiki/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters#PythonAPI
// UPDATE: Dynamic Reconfigure Conflicts with messaging if no reconfig server is running - disabled here
void LCM2ROS::sensor_request_Callback(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::sensor_request_t* msg){
  std::cout << "Got SENSOR_REQUEST setting sensor rates\n";
  if ((msg->spindle_rpm >=0) && (msg->spindle_rpm <=49) ){ // driver sets max at 5.2rpm
    double spindle_rads = msg->spindle_rpm * (2*M_PI)/60; // convert from RPM to rad/sec

    // Real device:
    //std::stringstream ss;
    //ss << "rosrun dynamic_reconfigure dynparam set /multisense_sl motor_speed " << spindle_rads;
    //ROS_ERROR("[%s]", ss.str().c_str() );    
    //system( ss.str().c_str() );

    // Simulation:
    std_msgs::Float64 spindle_speed_msg;
    spindle_speed_msg.data = spindle_rads;
    spindle_speed_pub_.publish(spindle_speed_msg);    
    ROS_ERROR("LCM2ROS Setting Spindle RPM: %d", ((int) msg->spindle_rpm) );
    ROS_ERROR("                    rad/sec: %f", spindle_rads );
  }else {
    ROS_ERROR("App Ignoring Out of Range Spindle RPM: %d", ((int) msg->spindle_rpm) );
  }
    
  if ((msg->head_fps >=0) && (msg->head_fps <=30) ){ // driver sets minimum at 1fps
    // Real device:
    //std::stringstream ss;
    //ss << "rosrun dynamic_reconfigure dynparam set /multisense_sl fps " << (int) msg->head_fps;
    //ROS_ERROR("[%s]", ss.str().c_str() );    
    //system( ss.str().c_str() );

    // Simulation:
    std_msgs::Float64 head_fps_msg;
    head_fps_msg.data = msg->head_fps;
    multisense_sl_fps_pub_.publish(head_fps_msg);
    ROS_ERROR("LCM2ROS Setting Head Camera FPS: %d", ((int) msg->head_fps) );
  }else {
    ROS_ERROR("LCM2ROS Ignoring Negative Head FPS: %d", ((int) msg->head_fps) );
  }
    
  if (msg->hand_fps >=0){
    std_msgs::Float64 hand_fps_msg;
    hand_fps_msg.data = msg->hand_fps;
    hand_fps_pub_.publish(hand_fps_msg);    
    ROS_ERROR("LCM2ROS Setting Hand Camera FPS: %d [discard in Translator]", ((int) msg->hand_fps) );
  }else {
    ROS_ERROR("LCM2ROS Ignoring Negative Hand FPS: %d", ((int) msg->hand_fps) );
  }    

}


// Sandia Hands joint command handlers
void LCM2ROS::sandiaLHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);

  for (int i=0; i<msg->num_joints; i++) {
    joint_command_msg.name.push_back("sandia_hands::l_hand::" + msg->name[i]); // must use scoped name
    joint_command_msg.position.push_back(msg->position[i]);
    joint_command_msg.velocity.push_back(msg->velocity[i]);
    joint_command_msg.effort.push_back(msg->effort[i]);
    joint_command_msg.kp_position.push_back(msg->kp_position[i]);
    joint_command_msg.kd_position.push_back(msg->kd_position[i]);
    joint_command_msg.ki_position.push_back(msg->ki_position[i]);
    joint_command_msg.kp_velocity.resize(msg->kp_velocity[i]);
    joint_command_msg.i_effort_min.push_back(msg->i_effort_min[i]);
    joint_command_msg.i_effort_max.push_back(msg->i_effort_max[i]);
  }
  if(ros::ok()) {
    sandia_l_hand_joint_cmd_pub_.publish(joint_command_msg);
  } 
} 

void LCM2ROS::sandiaRHandJointCommandHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::joint_command_t* msg) {

  osrf_msgs::JointCommands joint_command_msg;
  joint_command_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);

  for (int i=0; i<msg->num_joints; i++) {
    joint_command_msg.name.push_back("sandia_hands::r_hand::" + msg->name[i]); // must use scoped name
    joint_command_msg.position.push_back(msg->position[i]);
    joint_command_msg.velocity.push_back(msg->velocity[i]);
    joint_command_msg.effort.push_back(msg->effort[i]);
    joint_command_msg.kp_position.push_back(msg->kp_position[i]);
    joint_command_msg.kd_position.push_back(msg->kd_position[i]);
    joint_command_msg.ki_position.push_back(msg->ki_position[i]);
    joint_command_msg.kp_velocity.resize(msg->kp_velocity[i]);
    joint_command_msg.i_effort_min.push_back(msg->i_effort_min[i]);
    joint_command_msg.i_effort_max.push_back(msg->i_effort_max[i]);
  }
  if(ros::ok()) {
    sandia_r_hand_joint_cmd_pub_.publish(joint_command_msg);
  } 
}
  
  int counter =0;

void LCM2ROS::bodyTwistCmdHandler(const lcm::ReceiveBuffer* rbuf,const std::string &channel,const drc::twist_t* msg){
  geometry_msgs::Twist body_twist_cmd_msg;
  //body_twist_cmd_msg.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  body_twist_cmd_msg.linear.x =  msg->linear_velocity.x;
  body_twist_cmd_msg.linear.y =  msg->linear_velocity.y;
  body_twist_cmd_msg.linear.z =  msg->linear_velocity.z;
  body_twist_cmd_msg.angular.x =  msg->angular_velocity.x;
  body_twist_cmd_msg.angular.y =  msg->angular_velocity.y;
  body_twist_cmd_msg.angular.z =  msg->angular_velocity.z;
  if(ros::ok()){
    body_twist_cmd_pub_.publish(body_twist_cmd_msg);
  }
}


int main(int argc,char** argv) {

  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh);
  cout << "\nlcm2ros translator ready\n";
  
  
  ROS_ERROR("LCM2ROS Translator Sleeping");
  //handlerObject.setInitialMultisenseRate(5.0);
  sleep(4);
  ROS_ERROR("LCM2ROS Translator Ready");
  //handlerObject.setInitialMultisenseRate(5.0);  
  
  while(0 == lcm->handle());
  return 0;
}
