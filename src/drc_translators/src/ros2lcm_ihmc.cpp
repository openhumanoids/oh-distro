// Selective ros2lcm translator
// mfallon
// two modes:
// - passthrough: produces POSE_BODY and EST_ROBOT_STATE and CAMERA_LEFT
// - state estimation: produces ATLAS_STATE and POSE_BDI
//
// both modes produce SCAN

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <ihmc_msgs/BatchRawImuData.h>
#include <ihmc_msgs/RawImuData.h>
#include <ihmc_msgs/LastReceivedMessage.h>
//#include <pronto_msgs/CachedRawIMUData.h>
//#include <pronto_msgs/RawIMUData.h>
//#include <pronto_msgs/FootSensor.h>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/pronto/force_torque_t.hpp"
#include "lcmtypes/pronto/robot_state_t.hpp"
#include "lcmtypes/pronto/atlas_state_t.hpp"
#include "lcmtypes/pronto/utime_t.hpp"
#include "lcmtypes/pronto/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/pronto/atlas_behavior_t.hpp"
#include "lcmtypes/pronto/multisense_state_t.hpp"
#include "lcmtypes/pronto/plan_status_t.hpp"
#include "lcmtypes/ipab/last_received_message_t.hpp"
#include "lcmtypes/mav/ins_t.hpp"

#include <tf/transform_listener.h>

#define MODE_PASSTHROUGH 0
#define MODE_STATE_ESTIMATION 1

using namespace std;


struct Joints {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<std::string> name;
};


class App{
public:
  App(ros::NodeHandle node_, int mode_, std::string robotName_, std::string imuSensor_);
  ~App();

private:
  lcm::LCM lcmPublish_ ;
  ros::NodeHandle node_;
  int mode_;
  string robotName_;
  string imuSensor_;

//  tf::TransformListener listener_;
  
  ros::Subscriber jointStatesSub_;
  ros::Subscriber headJointStatesSub_;
  ros::Subscriber poseSub_;
  ros::Subscriber laserScanSub_;
  ros::Subscriber imuBatchSub_;
  ros::Subscriber imuSensorSub_;
  ros::Subscriber leftFootSensorSub_;
  ros::Subscriber rightFootSensorSub_;
  ros::Subscriber behaviorSub_;
  ros::Subscriber lastReceivedMessageSub_;

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void headJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void poseCallBack(const nav_msgs::OdometryConstPtr& msg);
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void imuBatchCallback(const ihmc_msgs::BatchRawImuDataConstPtr& msg);
  void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);

  void leftFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg);
  void rightFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg);
  void behaviorCallback(const std_msgs::Int32ConstPtr& msg);
  void lastReceivedMessageCallback(const ihmc_msgs::LastReceivedMessageConstPtr& msg);

  void appendFootSensors(pronto::force_torque_t& msg_out, geometry_msgs::Wrench left_sensor, geometry_msgs::Wrench right_sensor);
  void publishLidar(const sensor_msgs::LaserScanConstPtr& msg,string channel );
  void publishMultisenseState(int64_t utime, float position, float velocity);

  nav_msgs::Odometry lastPoseMsg_;
  geometry_msgs::Wrench lastLeftFootSensorMsg_;
  geometry_msgs::Wrench lastRightFootSensorMsg_;

  int64_t lastJointStateUtime_;
  bool verbose_;
};

App::App(ros::NodeHandle node_, int mode_, std::string robotName_, std::string imuSensor_) :
    node_(node_), mode_(mode_), robotName_(robotName_), imuSensor_(imuSensor_){
  ROS_INFO("Initializing Translator");
  if(!lcmPublish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  lastPoseMsg_.pose.pose.position.x = 0;
  lastPoseMsg_.pose.pose.position.y = 0;
  lastPoseMsg_.pose.pose.position.z = 0.85;
  lastPoseMsg_.pose.pose.orientation.w = 1;
  lastPoseMsg_.pose.pose.orientation.x = 0;
  lastPoseMsg_.pose.pose.orientation.y = 0;
  lastPoseMsg_.pose.pose.orientation.z = 0;

  // setting a queue of 1 reduces clumping of messages [desired for state estimation] but significantly reduces frequency
  // this is true even for just 2 subscriptions
  // setting to 100 means no messages are dropped during translation but imu data tends to be clumped into 50 message blocks

  int queue_size = 100;

  // Robot joint angles
  jointStatesSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/joint_states"), queue_size, &App::jointStatesCallback,this);
  poseSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/robot_pose"), queue_size, &App::poseCallBack,this);

  imuBatchSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/imu/" + imuSensor_ + "_batch"), queue_size, &App::imuBatchCallback,this);
  imuSensorSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/imu/" + imuSensor_), queue_size, &App::imuSensorCallback,this);

  leftFootSensorSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/foot_force_sensor/left"), queue_size, &App::leftFootSensorCallback,this);
  rightFootSensorSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/foot_force_sensor/right"), queue_size, &App::rightFootSensorCallback,this);
  // using previously used queue_size for scan:
  behaviorSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/behavior"), 100, &App::behaviorCallback,this);
  lastReceivedMessageSub_ = node_.subscribe(string("/ihmc_ros/" + robotName_ + "/output/last_received_message"), 100, &App::lastReceivedMessageCallback,this);

  // Multisense Joint Angles:
  if (mode_ == MODE_STATE_ESTIMATION){
    headJointStatesSub_ = node_.subscribe(string("/multisense/joint_states"), queue_size, &App::headJointStatesCallback,this);
  }
  laserScanSub_ = node_.subscribe(string("/multisense/lidar_scan"), 100, &App::laserScanCallback,this);

  verbose_ = false;
//  listener_;
};

App::~App()  {
}


void App::headJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg){
  if ( msg->name.size() > 1){
    // ROS_ERROR("Error: Unrecognised multisense joint: %s",msg->name[0].c_str());
    return;
  }

  int64_t utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  publishMultisenseState(utime, msg->position[0], msg->velocity[0]);

  /*
  std::string jname = "hokuyo_joint";
  int i = std::distance( msg->name.begin(), std::find( msg->name.begin(), msg->name.end(), jname ) );
  if( i == msg->name.size() ){
    //std::cout << "not found: " << jname << "\n";
  }else{
    publishMultisenseState(utime, msg->position[i], msg->velocity[i]);
  }
  */
}

void App::publishMultisenseState(int64_t utime, float position, float velocity){
  pronto::multisense_state_t msg_out;
  msg_out.utime = utime;
  msg_out.joint_position.push_back( position );
  msg_out.joint_velocity.push_back( velocity );
  msg_out.joint_effort.push_back( 0 );
  msg_out.joint_name.push_back( "hokuyo_joint");
  msg_out.num_joints = 1;
  lcmPublish_.publish("MULTISENSE_STATE", &msg_out);

  // publish message for bot_frames
  bot_core::rigid_transform_t preToPostFrame;
  preToPostFrame.utime = utime;
  preToPostFrame.trans[0] = 0;
  preToPostFrame.trans[1] = 0;
  preToPostFrame.trans[2] = 0;
  preToPostFrame.quat[0] = std::cos(position/2);
  preToPostFrame.quat[1] = 0;
  preToPostFrame.quat[2] = 0;
  preToPostFrame.quat[3] = std::sin(position/2);
  lcmPublish_.publish("PRE_SPINDLE_TO_POST_SPINDLE",
                              &preToPostFrame);
}


void App::poseCallBack(const nav_msgs::OdometryConstPtr& msg){
  lastPoseMsg_ = *msg;

  // ROS_ERROR("%d | %d %d", lastPoseMsg_.header.seq, lastPoseMsg_.header.stamp.sec, lastPoseMsg_.header.stamp.nsec);
  bot_core::pose_t lcm_pose_msg;
  lcm_pose_msg.utime = (int64_t) lastPoseMsg_.header.stamp.toNSec()/1000; // from nsec to usec
  lcm_pose_msg.pos[0] = lastPoseMsg_.pose.pose.position.x;
  lcm_pose_msg.pos[1] = lastPoseMsg_.pose.pose.position.y;
  lcm_pose_msg.pos[2] = lastPoseMsg_.pose.pose.position.z;
  lcm_pose_msg.orientation[0] =  lastPoseMsg_.pose.pose.orientation.w;
  lcm_pose_msg.orientation[1] =  lastPoseMsg_.pose.pose.orientation.x;
  lcm_pose_msg.orientation[2] =  lastPoseMsg_.pose.pose.orientation.y;
  lcm_pose_msg.orientation[3] =  lastPoseMsg_.pose.pose.orientation.z;
  lcmPublish_.publish("POSE_BDI", &lcm_pose_msg);
  if (mode_ == MODE_PASSTHROUGH){
    lcmPublish_.publish("POSE_BODY", &lcm_pose_msg);
  }

}

void App::leftFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg){
  lastLeftFootSensorMsg_ = *msg;
}

void App::rightFootSensorCallback(const geometry_msgs::WrenchConstPtr& msg){
  lastRightFootSensorMsg_ = *msg;
}

void App::behaviorCallback(const std_msgs::Int32ConstPtr& msg){
  // IHMC publish: 3 when standing or manipulating, 4 when walking (as atlas_behavior_t)
  // BDI publishs: 3 stand, 4 walk, 5 step, 6 manip (as atlas_behavior_t)
  // MIT publish: 1 stand, 2 walk, 8 mani  (as plan_status_t)
  // We (i.e. ddapp) need either:
  // - drc.atlas_behavior_t which comes from BDI typically) - and matches with IHMC output
  // - drc.plan_status_t which comes from MIT controller typically) - typically atlas_behavior_t is 'user' than
  // ddapp requires plan_status_t messages for autonomy. (pronto uses controller_status_t to contact ground points)
  bool behaviorMode = 0;

  if (behaviorMode == 0){
    pronto::atlas_behavior_t msg_out;

    msg_out.utime = lastJointStateUtime_;
    int temp = (int) msg->data;
    msg_out.behavior = (int) temp;
    lcmPublish_.publish("ATLAS_BEHAVIOR", &msg_out);
  }else{

    pronto::plan_status_t msg_out;
    msg_out.utime = lastJointStateUtime_;
    if (msg->data == 3){ // stand
      msg_out.plan_type = 1; // stand
    }else if (msg->data == 4){ // walk for ihmc. NB: this is different to step from bdi
      msg_out.plan_type = 2; // walk
      msg_out.execution_status = 0; // 'executing'
    }else{
      std::cout << "behavior not understood\n";
      return;
    }
    lcmPublish_.publish("PLAN_EXECUTION_STATUS", &msg_out);

    pronto::atlas_behavior_t msg_out_beh;
    msg_out_beh.utime = lastJointStateUtime_;
    msg_out_beh.behavior = 7; // user
    lcmPublish_.publish("ATLAS_BEHAVIOR", &msg_out);    
    
  }

}

void App::lastReceivedMessageCallback(const ihmc_msgs::LastReceivedMessageConstPtr& msg){
  ipab::last_received_message_t msg_out;
  msg_out.type = msg->type;
  msg_out.unique_id = msg->uniqueId;
  msg_out.receive_timestamp = msg->receiveTimestamp/1000;
  msg_out.time_since_last_received = msg->timeSinceLastReceived;
  lcmPublish_.publish("IHMC_LAST_RECEIVED", &msg_out);

}

void App::imuBatchCallback(const ihmc_msgs::BatchRawImuDataConstPtr& msg){

  pronto::atlas_raw_imu_batch_t imu;
  imu.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);

  if (verbose_)
    std::cout <<"                              " << imu.utime << " imu\n";

  imu.num_packets = 15;
  for (size_t i=0; i < 15 ; i++){

    //std::cout << i
    //  << " | " <<  msg->data[i].imu_timestamp
    //  << " | " <<  msg->data[i].packet_count
    //  << " | " <<  msg->data[i].dax << " " << msg->data[i].day << " " << msg->data[i].daz
    //  << " | " <<  msg->data[i].ddx << " " << msg->data[i].ddy << " " << msg->data[i].ddz << "\n";

    pronto::atlas_raw_imu_t raw;
    raw.utime = msg->data[i].imu_timestamp;
    raw.packet_count = msg->data[i].packet_count;
    raw.delta_rotation[0] = msg->data[i].dax;
    raw.delta_rotation[1] = msg->data[i].day;
    raw.delta_rotation[2] = msg->data[i].daz;

    raw.linear_acceleration[0] = msg->data[i].ddx;
    raw.linear_acceleration[1] = msg->data[i].ddy;
    raw.linear_acceleration[2] = msg->data[i].ddz;
    imu.raw_imu.push_back( raw );
  }
  lcmPublish_.publish( ("ATLAS_IMU_BATCH") , &imu);
}


void App::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg){

  mav::ins_t imu;
  imu.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  imu.device_time = imu.utime;
  imu.gyro[0] = msg->angular_velocity.x;
  imu.gyro[1] = msg->angular_velocity.y;
  imu.gyro[2] = msg->angular_velocity.z;
  imu.mag[0] = 0;
  imu.mag[1] = 0;
  imu.mag[2] = 0;
  imu.accel[0] = msg->linear_acceleration.x;
  imu.accel[1] = msg->linear_acceleration.y;
  imu.accel[2] = msg->linear_acceleration.z;
  imu.quat[0] = msg->orientation.w;
  imu.quat[1] = msg->orientation.x;
  imu.quat[2] = msg->orientation.y;
  imu.quat[2] = msg->orientation.z;
  imu.pressure = 0;
  imu.rel_alt = 0;

  lcmPublish_.publish( ("MICROSTRAIN_INS") , &imu);
}


int scan_counter=0;
void App::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg){
  if (scan_counter%80 ==0){
    ROS_ERROR("LSCAN [%d]", scan_counter );
    //std::cout << "SCAN " << scan_counter << "\n";
  }  
  scan_counter++;
  publishLidar(msg, "SCAN");
}

void App::publishLidar(const sensor_msgs::LaserScanConstPtr& msg,string channel ){
  bot_core::planar_lidar_t scan_out;
  scan_out.ranges = msg->ranges;
  scan_out.intensities = msg->intensities;
  scan_out.utime = (int64_t) floor(msg->header.stamp.toNSec()/1000);
  scan_out.nranges =msg->ranges.size();
  scan_out.nintensities=msg->intensities.size();
  scan_out.rad0 = msg->angle_min;
  scan_out.radstep = msg->angle_increment;
  lcmPublish_.publish(channel.c_str(), &scan_out);
}


void filterJointNames(std::vector<std::string> &joint_name  ){
  // Rename these joints to expected values:

  int n_joints = joint_name.size();
  for (int i = 0; i < n_joints; i++)  {
    // ihmc v3 to mit v3:
    //if (joint_name[i] == "l_arm_shz"){
    //  joint_name[i] = "l_arm_usy";
    //}
    //if (joint_name[i] == "r_arm_shz"){
    //  joint_name[i] = "r_arm_usy";
    //}

    if (joint_name[i] == "l_arm_wry"){
      joint_name[i] = "l_arm_uwy";
    }
    if (joint_name[i] == "l_arm_wrx"){
      joint_name[i] = "l_arm_mwx";
    }
    if (joint_name[i] == "l_arm_wrx"){
      joint_name[i] = "l_arm_mwx";
    }
    if (joint_name[i] == "r_arm_wry"){
      joint_name[i] = "r_arm_uwy";
    }
    if (joint_name[i] == "r_arm_wrx"){
      joint_name[i] = "r_arm_mwx";
    }

    // ihmc v5 to mit v5:
    if (joint_name[i] == "l_arm_wry2"){
      joint_name[i] = "l_arm_lwy";
    }
    if (joint_name[i] == "r_arm_wry2"){
      joint_name[i] = "r_arm_lwy";
    }

    if (joint_name[i] == "neck_ry"){
      joint_name[i] = "neck_ay";
    }
    if (joint_name[i] == "hokuyo_joint"){
      //double output = remainderf( joint_position[i] , M_PI);
      //std::cout << (joint_position[i]) << " "  << output << "\n";
      //joint_position[i] = output;
      //joint_name[i] = "hokuyo_link";
    }
  }
}


Joints reorderJoints(Joints &joints ){
  // Reorder the joints to this ordering:
  Joints joints_out;
  std::vector<std::string>  atlas_joint_names;
  atlas_joint_names = {"back_bkz", "back_bky", "back_bkx",
      "neck_ay", "l_leg_hpz", "l_leg_hpx", "l_leg_hpy",
      "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz",
      "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky",
      "r_leg_akx", "l_arm_shz", "l_arm_shx", "l_arm_ely",
      "l_arm_elx", "l_arm_uwy", "l_arm_mwx", "l_arm_lwy", "r_arm_shz",
      "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx", "r_arm_lwy"};

  joints_out.name = atlas_joint_names;
  joints_out.position.assign( joints_out.name.size(), 0);
  joints_out.velocity.assign( joints_out.name.size(), 0);
  joints_out.effort.assign( joints_out.name.size(), 0);


  for (int j =0; j< joints.name.size() ; j++){
    std::string jname = joints.name[j];
    int i = std::distance( atlas_joint_names.begin(), std::find( atlas_joint_names.begin(), atlas_joint_names.end(), jname ) );

    if( i == atlas_joint_names.size() ){
      //std::cout << "not found: " << jname << "\n";
    }else{
      joints_out.position[i] = joints.position[j];
      joints_out.velocity[i] = joints.velocity[j];
      joints_out.effort[i] = joints.effort[j];
      //std::cout << "found: " << jname << "\n";
    }
  }

  return joints_out;
}

void App::jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg){

  Joints joints;
  joints.position = msg->position;
  joints.velocity = msg->velocity;
  joints.effort = msg->effort;
  joints.name = msg->name;



  pronto::force_torque_t force_torque;
  appendFootSensors(force_torque, lastLeftFootSensorMsg_, lastRightFootSensorMsg_);

  if (mode_ == MODE_STATE_ESTIMATION){
    filterJointNames(joints.name);
    joints = reorderJoints(joints);
    pronto::atlas_state_t amsg;
    amsg.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec

    int n_joints = joints.position.size();
    amsg.joint_position.assign(n_joints , 0  );
    amsg.joint_velocity.assign(n_joints , 0  );
    amsg.joint_effort.assign(n_joints , 0  );
    amsg.num_joints = n_joints;
    //amsg.joint_name= msg->name;
    for (int i = 0; i < n_joints; i++)  {
      amsg.joint_position[ i ] = joints.position[ i ];
      amsg.joint_velocity[ i ] =0;// (double) msg->velocity[ i ];
      amsg.joint_effort[ i ] = 0;//msg->effort[i];
    }

    /*
    int n_joints = msg->position.size();
    amsg.joint_position.assign(n_joints , 0  );
    amsg.joint_velocity.assign(n_joints , 0  );
    amsg.joint_effort.assign(n_joints , 0  );
    amsg.num_joints = n_joints;
    //amsg.joint_name= msg->name;
    for (int i = 0; i < n_joints; i++)  {
      amsg.joint_position[ i ] = msg->position[ i ];
      amsg.joint_velocity[ i ] =0;// (double) msg->velocity[ i ];
      amsg.joint_effort[ i ] = 0;//msg->effort[i];
    }
    */


    //filter_joint_names( amsg.joint_name );
    amsg.force_torque = force_torque;
    lcmPublish_.publish("ATLAS_STATE", &amsg);
  }else if(mode_ == MODE_PASSTHROUGH){
    if(robotName_.compare("atlas")==0){
      filterJointNames(joints.name);
      // don't reorder in passthrough mode
    }

    if(robotName_.compare("valkyrie")==0){
      // temporary:
      //joints.name = {"l_leg_hpz", "l_leg_hpx", "l_leg_hpy", "l_leg_kny", "l_leg_aky", "l_leg_akx", "r_leg_hpz", "r_leg_hpx", "r_leg_hpy", "r_leg_kny", "r_leg_aky", "r_leg_akx", "back_bkz", "back_bky", "back_bkx", "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx", "l_arm_lwy", "neck_ay", "neck_by", "neck_cy", "r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx", "r_arm_lwy"};
    }

    pronto::robot_state_t msg_out;
    msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
    int n_joints = joints.position.size();
    msg_out.joint_position.assign(n_joints , 0  );
    msg_out.joint_velocity.assign(n_joints , 0  );
    msg_out.joint_effort.assign(n_joints , 0  );
    msg_out.num_joints = n_joints;
    msg_out.joint_name= joints.name;
    for (int i = 0; i < n_joints; i++)  {
      msg_out.joint_position[ i ] = joints.position[ i ];
      msg_out.joint_velocity[ i ] =0;// (double) msg->velocity[ i ];
      msg_out.joint_effort[ i ] = 0;//msg->effort[i];
    }
    msg_out.pose.translation.x = lastPoseMsg_.pose.pose.position.x;
    msg_out.pose.translation.y = lastPoseMsg_.pose.pose.position.y;
    msg_out.pose.translation.z = lastPoseMsg_.pose.pose.position.z;
    msg_out.pose.rotation.w = lastPoseMsg_.pose.pose.orientation.w;
    msg_out.pose.rotation.x = lastPoseMsg_.pose.pose.orientation.x;
    msg_out.pose.rotation.y = lastPoseMsg_.pose.pose.orientation.y;
    msg_out.pose.rotation.z = lastPoseMsg_.pose.pose.orientation.z;
    msg_out.force_torque = force_torque;
    lcmPublish_.publish("EST_ROBOT_STATE", &msg_out);
  }

  pronto::utime_t utime_msg;
  int64_t joint_utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec
  utime_msg.utime = joint_utime;
  lcmPublish_.publish("ROBOT_UTIME", &utime_msg);

  lastJointStateUtime_ = joint_utime;
}

void App::appendFootSensors(pronto::force_torque_t& msg_out, geometry_msgs::Wrench left_sensor, geometry_msgs::Wrench right_sensor){
  msg_out.l_foot_force_z  =  left_sensor.force.z;
  msg_out.l_foot_torque_x =  left_sensor.torque.x;
  msg_out.l_foot_torque_y =  left_sensor.torque.y;
  msg_out.r_foot_force_z  = right_sensor.force.z;
  msg_out.r_foot_torque_x = right_sensor.torque.x;
  msg_out.r_foot_torque_y = right_sensor.torque.y;

  msg_out.l_hand_force[0] =  0;
  msg_out.l_hand_force[1] =  0;
  msg_out.l_hand_force[2] =  0;
  msg_out.l_hand_torque[0] = 0;
  msg_out.l_hand_torque[1] = 0;
  msg_out.l_hand_torque[2] = 0;
  msg_out.r_hand_force[0] =  0;
  msg_out.r_hand_force[1] =  0;
  msg_out.r_hand_force[2] =  0;
  msg_out.r_hand_torque[0] =  0;
  msg_out.r_hand_torque[1] =  0;
  msg_out.r_hand_torque[2] =  0;
}




int main(int argc, char **argv){
  std::string robotName;// = "valkyrie"; // "atlas"
  std::string modeArgument;
  std::string imuSensor = "pelvis_imu_sensor_at_pelvis_frame"; // pelvis_imu_sensor_at_imu_frame


  if (argc >= 4){
     modeArgument = argv[1];
     robotName = argv[2];
     imuSensor = argv[3];
  }else {
    ROS_ERROR("Need to have three arguments: mode, robotName imuSensor");
    exit(-1);
  }

  int mode; // MODE_PASSTHROUGH or MODE_STATE_ESTIMATION
  if (modeArgument.compare("passthrough") == 0){
    mode = MODE_PASSTHROUGH;
  }else if (modeArgument.compare("state_estimation") == 0){
    mode = MODE_STATE_ESTIMATION;
  }else {
    ROS_ERROR("modeArgument not understood: use passthrough or state_estimation");
    exit(-1);
  }

  ros::init(argc, argv, "ros2lcm");
  ros::NodeHandle nh;
  new App(nh, mode, robotName, imuSensor);
  ROS_ERROR("ROS2LCM Translator Ready [mode: %d, %s] [robotName: %s] [imuSensor: %s]", mode, modeArgument.c_str(), robotName.c_str(), imuSensor.c_str());
  ros::spin();
  return 0;
}
