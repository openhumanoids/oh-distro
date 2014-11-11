#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/atlas_state_t.hpp"
#include "lcmtypes/drc/atlas_command_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/utime_two_t.hpp"
#include "lcmtypes/drc/atlas_raw_imu_batch_t.hpp"
#include "lcmtypes/drc/double_array_t.hpp"
#include "lcmtypes/bot_core/pose_t.hpp"
#include <drc_utils/joint_utils.hpp>

#include <chrono>
#include <thread>
// same as bot_timestamp_now():
static int64_t _timestamp_now(){
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}



int main (int argc, char ** argv){
int n_q = 28;

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;


for (int j =0 ; j < 1000000 ; j++){

  //// Main Joint State Message:
  drc::atlas_state_t msg;
  msg.utime = _timestamp_now()	;
  for (size_t i=0; i <n_q ; i++){
    // msg.joint_position.push_back(s_data_from_robot.j[i].q);
    // msg.joint_velocity.push_back(s_data_from_robot.j[i].qd);
    // msg.joint_effort.push_back(s_data_from_robot.j[i].f);
    msg.joint_position.push_back(0);
    msg.joint_velocity.push_back(0);
    msg.joint_effort.push_back(0);

    //msg.joint_position_out.push_back(s_data_from_robot.j[i].q_out);
    //msg.joint_velocity_out.push_back(s_data_from_robot.j[i].qd_out);
  }
  //msg.joint_name = ATLAS_JOINT_NAMES;
  msg.num_joints = n_q;
  
  /*
    msg.force_torque.l_foot_force_z = s_data_from_robot.foot_sensors[FS_LEFT].fz;
    msg.force_torque.l_foot_torque_x = s_data_from_robot.foot_sensors[FS_LEFT].mx;
    msg.force_torque.l_foot_torque_y = s_data_from_robot.foot_sensors[FS_LEFT].my;

    msg.force_torque.r_foot_force_z = s_data_from_robot.foot_sensors[FS_RIGHT].fz;
    msg.force_torque.r_foot_torque_x = s_data_from_robot.foot_sensors[FS_RIGHT].mx;
    msg.force_torque.r_foot_torque_y = s_data_from_robot.foot_sensors[FS_RIGHT].my;
  
    memcpy( msg.force_torque.l_hand_force, s_data_from_robot.wrist_sensors[WS_LEFT].f.n, 3*sizeof(float) );
    memcpy( msg.force_torque.l_hand_torque, s_data_from_robot.wrist_sensors[WS_LEFT].m.n, 3*sizeof(float) );
    memcpy( msg.force_torque.r_hand_force, s_data_from_robot.wrist_sensors[WS_RIGHT].f.n, 3*sizeof(float) );
    memcpy( msg.force_torque.r_hand_torque, s_data_from_robot.wrist_sensors[WS_RIGHT].m.n, 3*sizeof(float) );
*/
    lcm->publish( ("ATLAS_STATE") , &msg);  
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

}
  
  
  return 0;
}
