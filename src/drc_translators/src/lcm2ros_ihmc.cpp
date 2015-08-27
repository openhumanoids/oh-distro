#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"
#include "lcmtypes/drc/walking_plan_t.hpp"
#include "lcmtypes/drc/walking_plan_request_t.hpp"
#include "lcmtypes/drc/footstep_plan_t.hpp"
#include "lcmtypes/drc/plan_control_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"

#include "lcmtypes/ipab/com_height_packet_message_t.hpp"
#include "lcmtypes/ipab/pause_command_message_t.hpp"
#include "lcmtypes/ipab/hand_pose_packet_message_t.hpp"
#include "lcmtypes/ipab/scs_api_command_t.hpp"

#include <ihmc_msgs/FootstepDataListMessage.h>
#include <ihmc_msgs/ComHeightPacketMessage.h>
#include <ihmc_msgs/PauseCommandMessage.h>
#include <ihmc_msgs/HandPosePacketMessage.h>
#include <ihmc_msgs/ArmJointTrajectoryPacketMessage.h>
#include <ihmc_msgs/WholeBodyTrajectoryPacketMessage.h>
#include <ihmc_msgs/StopMotionPacketMessage.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pronto_utils/pronto_vis.hpp> // visualize pt clds


#define LEFT 0
#define RIGHT 1

using namespace std;

class LCM2ROS{
  public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string robotName_);
    ~LCM2ROS() {}

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;
    ros::NodeHandle* node_;
    string robotName_;

    void footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg);
    void footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::footstep_plan_t* msg);
    ros::Publisher walking_plan_pub_;
    ihmc_msgs::FootstepDataMessage createFootStepList(int foot_to_start_with, double x_pos, double y_pos, double z_pos, double orient_w, double orient_x, double orient_y, double orient_z);
    void sendBasicSteps();

    void comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::com_height_packet_message_t* msg);
    ros::Publisher com_height_pub_;

    void pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::pause_command_message_t* msg);
    void stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
    void stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg);
    ros::Publisher pause_pub_, stop_manip_pub_;

    void handPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::hand_pose_packet_message_t* msg);
    ros::Publisher hand_pose_pub_;

    void robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg);
    ros::Publisher arm_joint_traj_pub_, arm_joint_traj2_pub_, whole_body_trajectory_pub_;

    void sendSingleArmPlan(const drc::robot_plan_t* msg, std::vector<string> output_joint_names_arm,
                           std::vector<string> input_joint_names, bool is_right);
    bool getSingleArmPlan(const drc::robot_plan_t* msg, std::vector<string> output_joint_names_arm,
                          std::vector<string> input_joint_names, bool is_right,
                          trajectory_msgs::JointTrajectory &m);
    bool getSingleArmPlan(const drc::robot_plan_t* msg, std::vector<string> output_joint_names_arm,
                          std::vector<string> input_joint_names, bool is_right,
                          ihmc_msgs::ArmJointTrajectoryPacketMessage &m);

    void scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::scs_api_command_t* msg);
    ros::Publisher scs_api_pub_;

    pronto_vis* pc_vis_;
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string robotName_):
    lcm_(lcm_),nh_(nh_), robotName_(robotName_) {
  // If pronto is running never send plans like this:
  lcm_->subscribe("WALKING_CONTROLLER_PLAN_REQUEST",&LCM2ROS::footstepPlanHandler, this);
    // COMMITTED_FOOTSTEP_PLAN is creating in Pronto frame and transformed into BDI/IHMC frame:
  lcm_->subscribe("BDI_ADJUSTED_FOOTSTEP_PLAN",&LCM2ROS::footstepPlanBDIModeHandler, this);
  walking_plan_pub_ = nh_.advertise<ihmc_msgs::FootstepDataListMessage>("/ihmc_ros/" + robotName_ + "/control/footstep_list",10);

  lcm_->subscribe("VAL_COMMAND_COM_HEIGHT",&LCM2ROS::comHeightHandler, this);
  com_height_pub_ =  nh_.advertise<ihmc_msgs::ComHeightPacketMessage>("/ihmc_ros/" + robotName_ + "/control/com_height",10);

  lcm_->subscribe("VAL_COMMAND_PAUSE",&LCM2ROS::pauseHandler, this);
  lcm_->subscribe("STOP_WALKING",&LCM2ROS::stopHandler, this); // from drake-designer
  pause_pub_ =  nh_.advertise<ihmc_msgs::PauseCommandMessage>("/ihmc_ros/" + robotName_ + "/control/pause_footstep_exec",10);
  lcm_->subscribe("COMMITTED_PLAN_PAUSE",&LCM2ROS::stopManipHandler, this); // from drake-designer to stop manipulation plans
  stop_manip_pub_ =  nh_.advertise<ihmc_msgs::StopMotionPacketMessage>("/ihmc_ros/" + robotName_ + "/control/stop_motion",10);

  // robot plan messages now used, untested
  lcm_->subscribe("COMMITTED_ROBOT_PLAN",&LCM2ROS::robotPlanHandler, this);
  arm_joint_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/ihmc_ros/" + robotName_ + "/control/arm_joint_trajectory2",10);
  arm_joint_traj2_pub_ = nh_.advertise<ihmc_msgs::ArmJointTrajectoryPacketMessage>("/ihmc_ros/" + robotName_ + "/control/arm_joint_trajectory",10);
  whole_body_trajectory_pub_ = nh_.advertise<ihmc_msgs::WholeBodyTrajectoryPacketMessage>("/ihmc_ros/" + robotName_ + "/control/whole_body_trajectory",10);


  lcm_->subscribe("SCS_API_CONTROL",&LCM2ROS::scsAPIHandler, this);
  scs_api_pub_ = nh_.advertise<std_msgs::String>("/ihmc_ros/" + robotName_ + "/api_command",10);

  // depreciated:
  //lcm_->subscribe("VAL_COMMAND_HAND_POSE",&LCM2ROS::handPoseHandler, this);
  //hand_pose_pub_ =  nh_.advertise<ihmc_msgs::HandPosePacketMessage>("/ihmc_ros/" + robotName_ + "/control/hand_pose",10);


  node_ = new ros::NodeHandle();

  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(9995,"Output step positions",5,1) );

}


ihmc_msgs::FootstepDataMessage LCM2ROS::createFootStepList(int foot_to_start_with, double x_pos, double y_pos, double z_pos, double orient_w, double orient_x, double orient_y, double orient_z){
  ihmc_msgs::FootstepDataMessage footStepList;
  footStepList.robot_side = foot_to_start_with;
  footStepList.location.x = x_pos;
  footStepList.location.y = y_pos;
  footStepList.location.z = z_pos;
  footStepList.orientation.w = orient_w;
  footStepList.orientation.x = orient_x;
  footStepList.orientation.y = orient_y;
  footStepList.orientation.z = orient_z;
  return footStepList;
}

void LCM2ROS::sendBasicSteps(){
  ihmc_msgs::FootstepDataListMessage mout;
  //mout.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  mout.transfer_time = 1.0;
  mout.swing_time = 1.0;
  //mout.trajectoryWaypointGenerationMethod = 0;
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0,  0.12,   0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0, -0.12,   0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0.1,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0.1, -0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0.2,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0.2, -0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0.1,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0.1, -0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(LEFT , 0  ,  0.12, 0, 1,0,0,0) );
  mout.footstep_data_list.push_back( createFootStepList(RIGHT, 0  , -0.12, 0, 1,0,0,0) );
  walking_plan_pub_.publish(mout);
}


Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {

  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  // Post DRC Trails: replace these with Eigen's own conversions
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }

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

void LCM2ROS::scsAPIHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::scs_api_command_t* msg)
{
  std_msgs::String rmsg;
  rmsg.data = msg->command;
  scs_api_pub_.publish(rmsg);
}

void LCM2ROS::footstepPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::walking_plan_request_t* msg) {
  ROS_ERROR("LCM2ROS got WALKING_CONTROLLER_PLAN_REQUEST (non-pronto and drake mode)");
  // sendBasicSteps();

  // Note:
  // TODO: remove pc_vis_

  std::vector< Eigen::Isometry3d > steps;
  std::vector<Isometry3dTime> stepsT;
  for (int i=0; i < msg->footstep_plan.num_steps; i++){ // skip the first two standing steps
      drc::footstep_t s = msg->footstep_plan.footsteps[i];

      Eigen::Isometry3d step;
      step.setIdentity();
      step.translation().x() = s.pos.translation.x;
      step.translation().y() = s.pos.translation.y;
      step.translation().z() = s.pos.translation.z;
      step.rotate(Eigen::Quaterniond(s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z));

      if(robotName_.compare("valkyrie")==0){
        ROS_ERROR("LCM2ROS flipping valkyrie foot orientations");
/*
        if (s.is_right_foot){
          Eigen::Isometry3d fix_transform;
          fix_transform.setIdentity();
          fix_transform.translation().x() = 0;
          fix_transform.translation().y() = 0;
          fix_transform.translation().z() = 0;
          fix_transform.rotate(euler_to_quat( 0*M_PI/180, -90*M_PI/180, 0 )) ;
          step = step*fix_transform;
        }else{
          Eigen::Isometry3d fix_transform;
          fix_transform.setIdentity();
          fix_transform.translation().x() = 0;
          fix_transform.translation().y() = 0;
          fix_transform.translation().z() = 0;
          fix_transform.rotate(euler_to_quat( 180*M_PI/180, 90*M_PI/180, 0 )) ;
          step = step*fix_transform;
        }
*/
      }else{
        ROS_ERROR("LCM2ROS not flipping atlas foot orientations");
      }

      Isometry3dTime stepT = Isometry3dTime(i, step);
      stepsT.push_back(stepT);
      steps.push_back(step);

  }
  pc_vis_->pose_collection_to_lcm_from_list(9995, stepsT);

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transfer_time = 1.2;
  mout.swing_time = 1.2;
  // mout.trajectoryWaypointGenerationMethod = 0;
  for (int i=2; i < msg->footstep_plan.num_steps; i++){ // skip the first two standing steps
    drc::footstep_t s = msg->footstep_plan.footsteps[i];
    //mout.footstep_data_list.push_back( createFootStepList(s.is_right_foot , s.pos.translation.x, s.pos.translation.y, s.pos.translation.z,
    //                                                    s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z) );

    Eigen::Quaterniond r(steps[i].rotation());
    Eigen::Vector3d t(steps[i].translation());
    mout.footstep_data_list.push_back( createFootStepList(s.is_right_foot , t[0], t[1], t[2], r.w(),r.x(),r.y(),r.z() ));
  }
  walking_plan_pub_.publish(mout);
}

void LCM2ROS::footstepPlanBDIModeHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::footstep_plan_t* msg) {
  ROS_ERROR("LCM2ROS got BDI_ADJUSTED_FOOTSTEP_PLAN (pronto and bdi mode)");
  // sendBasicSteps();

  ihmc_msgs::FootstepDataListMessage mout;
  mout.transfer_time = 1.2;
  mout.swing_time = 1.2;
  // mout.trajectoryWaypointGenerationMethod = 0;
  for (int i=2; i < msg->num_steps; i++){ // skip the first two standing steps
    drc::footstep_t s = msg->footsteps[i];
    mout.footstep_data_list.push_back( createFootStepList(s.is_right_foot , s.pos.translation.x, s.pos.translation.y, s.pos.translation.z,
                                                        s.pos.rotation.w, s.pos.rotation.x, s.pos.rotation.y, s.pos.rotation.z) );
  }
  walking_plan_pub_.publish(mout);
}


void LCM2ROS::comHeightHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::com_height_packet_message_t* msg) {
  ROS_ERROR("LCM2ROS got com height");
  ihmc_msgs::ComHeightPacketMessage mout;
  mout.height_offset = msg->height_offset;
  com_height_pub_.publish(mout);
}

void LCM2ROS::pauseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::pause_command_message_t* msg) {
  ROS_ERROR("LCM2ROS got pause %d", (int) msg->pause);
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = msg->pause;
  pause_pub_.publish(mout);
}

void LCM2ROS::stopHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg) {
  ROS_ERROR("LCM2ROS got drake-designer - sending pause=true");
  ihmc_msgs::PauseCommandMessage mout;
  mout.pause = true;
  pause_pub_.publish(mout);
}


void LCM2ROS::stopManipHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::plan_control_t* msg) {
  ROS_ERROR("LCM2ROS got drake-designer - sending manipulate stop");
  ihmc_msgs::StopMotionPacketMessage mout;
  mout.unique_id = msg->utime;
  stop_manip_pub_.publish(mout);
}

void LCM2ROS::handPoseHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const ipab::hand_pose_packet_message_t* msg) {
  ROS_ERROR("LCM2ROS got handPose packet");
  ihmc_msgs::HandPosePacketMessage mout;
  mout.robot_side = msg->robot_side;
  mout.to_home_position = msg->to_home_position;
  mout.trajectory_time = msg->trajectory_time;
  mout.joint_angles = msg->joint_angles;
  hand_pose_pub_.publish(mout);
}


void filterJointNamesToIHMC(std::vector<std::string> &joint_name  ){
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

    if (joint_name[i] == "l_arm_uwy"){
      joint_name[i] = "l_arm_wry";
    }
    if (joint_name[i] == "l_arm_mwx"){
      joint_name[i] = "l_arm_wrx";
    }
    if (joint_name[i] == "l_arm_mwx"){
      joint_name[i] = "l_arm_wrx";
    }
    if (joint_name[i] == "r_arm_uwy"){
      joint_name[i] = "r_arm_wry";
    }
    if (joint_name[i] == "r_arm_mwx"){
      joint_name[i] = "r_arm_wrx";
    }

    // ihmc v5 to mit v5:
    if (joint_name[i] == "l_arm_lwy"){
      joint_name[i] = "l_arm_wry2";
    }
    if (joint_name[i] == "r_arm_lwy"){
      joint_name[i] = "r_arm_wry2";
    }

    if (joint_name[i] == "neck_ay"){
      joint_name[i] = "neck_ry";
    }
    if (joint_name[i] == "hokuyo_joint"){
      //double output = remainderf( joint_position[i] , M_PI);
      //std::cout << (joint_position[i]) << " "  << output << "\n";
      //joint_position[i] = output;
      //joint_name[i] = "hokuyo_link";
    }
  }
}


void LCM2ROS::sendSingleArmPlan(const drc::robot_plan_t* msg,
                                std::vector<string> output_joint_names_arm,
                                std::vector<string> input_joint_names, bool is_right){
  //trajectory_msgs::JointTrajectory m;
  //bool status = getSingleArmPlan(msg, output_joint_names_arm, input_joint_names, m);
  //if (status)
  //  arm_joint_traj_pub_.publish(m);

  ihmc_msgs::ArmJointTrajectoryPacketMessage m;
  bool status = getSingleArmPlan(msg, output_joint_names_arm, input_joint_names, is_right, m);
  if (status)
    arm_joint_traj2_pub_.publish(m);

}

bool LCM2ROS::getSingleArmPlan(const drc::robot_plan_t* msg,
                               std::vector<string> output_joint_names_arm,
                               std::vector<string> input_joint_names, bool is_right,
                               trajectory_msgs::JointTrajectory &m){

  // Find the indices of the arm joints which we want to extract
  std::vector<int> arm_indices;// = {3,10,11,12,13,14,15};
  for (size_t i =0; i < output_joint_names_arm.size(); i++){
    std:string name = output_joint_names_arm[i];
    vector<string>::iterator it;
    it = find (input_joint_names.begin(), input_joint_names.end(), name );
    int index = std::distance( input_joint_names.begin(), it );
    if ( index < input_joint_names.size() ){
      //std::cout << name << " found in input_joint_names at " << index << '\n';
      arm_indices.push_back(index);
    }else{
      ROS_ERROR("%s not found in input_joint_names, not sending plan", name.c_str());
      std::cout << name << " not found in input_joint_names, not sending plan\n";
      return false;
    }
  }

  // Fish out the arm indices:
  m.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
  m.joint_names = output_joint_names_arm;
  for (int i=1; i < msg->num_states; i++){ // NB: skipping the first sample as it has time = 0
    drc::robot_state_t state = msg->plan[i];
    trajectory_msgs::JointTrajectoryPoint point;
    int i1=(i>0)?(i-1):0;
    int i2=i;
    int i3=(i<msg->num_states-1)?(i+1):(msg->num_states-1);

    for (int j=0; j <arm_indices.size() ; j++){
      point.positions.push_back( state.joint_position[ arm_indices[j] ] );
      double dt1=(msg->plan[i2].utime-msg->plan[i1].utime)*1e-6;
      double dt2=(msg->plan[i3].utime-msg->plan[i2].utime)*1e-6;
      double dq1=msg->plan[i2].joint_position[ arm_indices[j] ]-msg->plan[i1].joint_position[ arm_indices[j] ];
      double dq2=msg->plan[i3].joint_position[ arm_indices[j] ]-msg->plan[i2].joint_position[ arm_indices[j] ];
      point.velocities.push_back( (dt1*dt2!=0)?(dq1/dt1*0.5 + dq2/dt2*0.5):0.0 );
      point.accelerations.push_back( 0  );
      point.effort.push_back( state.joint_effort[ arm_indices[j] ] );
      point.time_from_start = ros::Duration().fromSec(state.utime*1E-6);
    }
    m.points.push_back(point);
  }
  return true;
}


bool LCM2ROS::getSingleArmPlan(const drc::robot_plan_t* msg,
                               std::vector<string> output_joint_names_arm,
                               std::vector<string> input_joint_names, bool is_right,
                               ihmc_msgs::ArmJointTrajectoryPacketMessage &m){

  // Find the indices of the arm joints which we want to extract
  std::vector<int> arm_indices;// = {3,10,11,12,13,14,15};
  for (size_t i =0; i < output_joint_names_arm.size(); i++){
    std:string name = output_joint_names_arm[i];
    vector<string>::iterator it;
    it = find (input_joint_names.begin(), input_joint_names.end(), name );
    int index = std::distance( input_joint_names.begin(), it );
    if ( index < input_joint_names.size() ){
      //std::cout << name << " found in input_joint_names at " << index << '\n';
      arm_indices.push_back(index);
    }else{
      ROS_ERROR("%s not found in input_joint_names, not sending plan", name.c_str());
      std::cout << name << " not found in input_joint_names, not sending plan\n";
      return false;
    }
  }

  // Fish out the arm indices:
  if (is_right){
    m.robot_side = 1;
  }else{
    m.robot_side = 0;
  }

  //m.joint_names = output_joint_names_arm;
  for (int i=1; i < msg->num_states; i++){ // NB: skipping the first sample as it has time = 0
    drc::robot_state_t state = msg->plan[i];
    ihmc_msgs::JointTrajectoryPointMessage point;
    int i1=(i>0)?(i-1):0;
    int i2=i;
    int i3=(i<msg->num_states-1)?(i+1):(msg->num_states-1);


    for (int j=0; j <arm_indices.size() ; j++){
      point.positions.push_back( state.joint_position[ arm_indices[j] ] );
      double dt1=(msg->plan[i2].utime-msg->plan[i1].utime)*1e-6;
      double dt2=(msg->plan[i3].utime-msg->plan[i2].utime)*1e-6;
      double dq1=msg->plan[i2].joint_position[ arm_indices[j] ]-msg->plan[i1].joint_position[ arm_indices[j] ];
      double dq2=msg->plan[i3].joint_position[ arm_indices[j] ]-msg->plan[i2].joint_position[ arm_indices[j] ];
      point.velocities.push_back( (dt1*dt2!=0)?(dq1/dt1*0.5 + dq2/dt2*0.5):0.0 );
      //point.accelerations.push_back( 0  );
      //point.effort.push_back( state.joint_effort[ arm_indices[j] ] );
      point.time = (double) state.utime*1E-6;
    }
    m.trajectory_points.push_back(point);
  }
  return true;
}



void LCM2ROS::robotPlanHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const drc::robot_plan_t* msg) {
  ROS_ERROR("LCM2ROS got robot plan");




  std::vector<string> l_arm_strings;
  std::vector<string> r_arm_strings;
  std::vector<string> input_joint_names = msg->plan[0].joint_name;

  if(robotName_.compare("atlas")==0){
    // Remove MIT/IPAB joint names and use IHMC joint names:
    filterJointNamesToIHMC(input_joint_names);
    l_arm_strings = {"l_arm_shz","l_arm_shx","l_arm_ely","l_arm_elx","l_arm_wry","l_arm_wrx","l_arm_wry2"};
    r_arm_strings = {"r_arm_shz","r_arm_shx","r_arm_ely","r_arm_elx","r_arm_wry","r_arm_wrx","r_arm_wry2"};
  }else if (robotName_.compare("valkyrie")==0){
    l_arm_strings = {"LeftShoulderPitch","LeftShoulderRoll","LeftShoulderYaw","LeftElbowPitch","LeftForearmYaw","LeftWristRoll","LeftWristPitch"};
    r_arm_strings = {"RightShoulderPitch","RightShoulderRoll","RightShoulderYaw","RightElbowPitch","RightForearmYaw","RightWristRoll","RightWristPitch"};
  }


  ihmc_msgs::ArmJointTrajectoryPacketMessage left_arm_trajectory;
  bool status_left = getSingleArmPlan(msg, l_arm_strings, input_joint_names,
                                 false, left_arm_trajectory);
  ihmc_msgs::ArmJointTrajectoryPacketMessage right_arm_trajectory;
  bool status_right = getSingleArmPlan(msg, r_arm_strings, input_joint_names,
                            true, right_arm_trajectory);
  if(!status_left || !status_right){
    ROS_ERROR("LCM2ROS: problem with arm plan, not sending");
  }

  ihmc_msgs::WholeBodyTrajectoryPacketMessage wbt_msg;
  wbt_msg.unique_id = msg->utime;
  wbt_msg.left_arm_trajectory = left_arm_trajectory;
  wbt_msg.right_arm_trajectory = right_arm_trajectory;
  for (int i=1; i < msg->num_states; i++){ // NB: skipping the first sample as it has time = 0
    drc::robot_state_t state = msg->plan[i];

    geometry_msgs::Vector3 pelvis_world_position;
    pelvis_world_position.x = state.pose.translation.x;
    pelvis_world_position.y = state.pose.translation.y;
    pelvis_world_position.z = state.pose.translation.z;
    wbt_msg.pelvis_world_position.push_back(pelvis_world_position);

    geometry_msgs::Quaternion pelvis_world_orientation;
    pelvis_world_orientation.w = state.pose.rotation.w;
    pelvis_world_orientation.x = state.pose.rotation.x;
    pelvis_world_orientation.y = state.pose.rotation.y;
    pelvis_world_orientation.z = state.pose.rotation.z;
    wbt_msg.pelvis_world_orientation.push_back(pelvis_world_orientation);
    wbt_msg.time_at_waypoint.push_back(state.utime*1E-6);
  }
  wbt_msg.num_waypoints = msg->num_states-1; // NB: skipping the first sample as it has time = 0
  wbt_msg.num_joints_per_arm = l_arm_strings.size();

  whole_body_trajectory_pub_.publish(wbt_msg);

  ROS_ERROR("LCM2ROS sent Whole Body Trajectory");

/*
  sendSingleArmPlan(msg, l_arm_strings, input_joint_names, false);
  ROS_ERROR("LCM2ROS sent left arm, sleeping for 1 second");
  sleep(1);
  ROS_ERROR("LCM2ROS sent right arm");
  sendSingleArmPlan(msg, r_arm_strings, input_joint_names, true);
*/
}

int main(int argc,char** argv) {
  std::string robotName;// = "valkyrie"; // "atlas"

  if (argc >= 2){
     ROS_ERROR("meah %s", argv[1] );
     robotName = argv[1];
  }else {
    ROS_ERROR("Need to have an argument: robot name");
    exit(-1);
  }

  ros::init(argc,argv,"lcm2ros",ros::init_options::NoSigintHandler);
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }  
  ros::NodeHandle nh;
  
  LCM2ROS handlerObject(lcm, nh, robotName);
  ROS_ERROR("LCM2ROS Translator Ready [robotName: %s]", robotName.c_str() );
  
  while(0 == lcm->handle());
  return 0;
}
