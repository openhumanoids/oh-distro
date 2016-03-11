#include "AtlasCommandDriver.hpp"

using namespace std;

AtlasCommandDriver::AtlasCommandDriver(const JointNames *input_joint_names, const vector<string> &state_coordinate_names) {
  getRobotJointIndexMap(input_joint_names, &input_index_map);

  state_to_drake_input_map = Eigen::VectorXi::Zero(state_coordinate_names.size());
  for (int i=0; i < state_coordinate_names.size(); i++) {
    state_to_drake_input_map[i] = -1;
    for (int j=0; j < input_joint_names->drake.size(); j++) {
      if (state_coordinate_names[i].compare(input_joint_names->drake[j]) == 0) {
        state_to_drake_input_map[i] = j;
        //cout << "state coordinate: " << state_coordinate_names[i] << " matches input name: " << input_joint_names->drake[j] << endl;
      }
    }
  }

  m_num_joints = input_joint_names->robot.size();
  
  msg.num_joints = m_num_joints;
  msg.joint_names.resize(msg.num_joints);

  msg.position.resize(msg.num_joints);
  msg.velocity.resize(msg.num_joints);
  msg.effort.resize(msg.num_joints);

  msg.k_q_p.resize(msg.num_joints);
  msg.k_q_i.resize(msg.num_joints);
  msg.k_qd_p.resize(msg.num_joints);
  msg.k_f_p.resize(msg.num_joints);
  msg.ff_qd.resize(msg.num_joints);
  msg.ff_qd_d.resize(msg.num_joints);
  msg.ff_f_d.resize(msg.num_joints);
  msg.ff_const.resize(msg.num_joints);

  msg.k_effort.resize(msg.num_joints); // only used in sim
  msg.desired_controller_period_ms = 1; // set desired controller rate (ms), only used in sim

  for (int i=0; i<m_num_joints; i++) {
    msg.position[i] = 0.0;
    msg.velocity[i] = 0.0;
    msg.effort[i] = 0.0;

    msg.joint_names[input_index_map.drake_to_robot[i]] = input_joint_names->drake[i];
    
    msg.k_q_p[input_index_map.drake_to_robot[i]] = 0;
    msg.k_q_i[input_index_map.drake_to_robot[i]] = 0;;
    msg.k_qd_p[input_index_map.drake_to_robot[i]] = 0;;
    msg.k_f_p[input_index_map.drake_to_robot[i]] = 0;;
    msg.ff_qd[input_index_map.drake_to_robot[i]] = 0;;
    msg.ff_qd_d[input_index_map.drake_to_robot[i]] = 0;;
    msg.ff_f_d[input_index_map.drake_to_robot[i]] = 0;;
    msg.ff_const[input_index_map.drake_to_robot[i]] = 0;;

    msg.k_effort[input_index_map.drake_to_robot[i]] = (uint8_t)255; // take complete control of joints (remove BDI control), sim only
  }
}

void AtlasCommandDriver::updateGains(const HardwareGains *gains) {

  if (gains->k_q_p.size() != m_num_joints)
    throw std::runtime_error("Length of k_q_p must be equal to m_num_joints");
  if (gains->k_q_i.size() != m_num_joints)
    throw std::runtime_error("Length of k_q_i must be equal to m_num_joints");
  if (gains->k_qd_p.size() != m_num_joints)
    throw std::runtime_error("Length of k_qd_p must be equal to m_num_joints");
  if (gains->k_f_p.size() != m_num_joints)
    throw std::runtime_error("Length of k_f_p must be equal to m_num_joints");
  if (gains->ff_qd.size() != m_num_joints)
    throw std::runtime_error("Length of ff_qd must be equal to m_num_joints");
  if (gains->ff_qd_d.size() != m_num_joints)
    throw std::runtime_error("Length of ff_qd_d must be equal to m_num_joints");
  if (gains->ff_f_d.size() != m_num_joints)
    throw std::runtime_error("Length of ff_f_d must be equal to m_num_joints");
  if (gains->ff_const.size() != m_num_joints)
    throw std::runtime_error("Length of ff_const must be equal to m_num_joints");
  
  for (int i=0; i<m_num_joints; i++) {
    msg.k_q_p[input_index_map.drake_to_robot[i]] = gains->k_q_p[i];
    msg.k_q_i[input_index_map.drake_to_robot[i]] = gains->k_q_i[i];
    msg.k_qd_p[input_index_map.drake_to_robot[i]] = gains->k_qd_p[i];
    msg.k_f_p[input_index_map.drake_to_robot[i]] = gains->k_f_p[i];
    msg.ff_qd[input_index_map.drake_to_robot[i]] = gains->ff_qd[i];
    msg.ff_qd_d[input_index_map.drake_to_robot[i]] = gains->ff_qd_d[i];
    msg.ff_f_d[input_index_map.drake_to_robot[i]] = gains->ff_f_d[i];
    msg.ff_const[input_index_map.drake_to_robot[i]] = gains->ff_const[i];
  }
}
    
bot_core::atlas_command_t* AtlasCommandDriver::encode(double t, QPControllerOutput *qp_output, const HardwareParams &params) {
  // Copy data from the given qp_output into the stored LCM message object. 

  msg.utime = (long)(t*1000000);
  int state_index, drake_input_index, robot_input_index;
  // cout << "force: " << endl << params.joint_is_force_controlled << endl << endl;
  // cout << "position: " << endl << params.joint_is_position_controlled << endl << endl;
  for (int i=0; i < m_num_joints; i++) {
    drake_input_index = i;
    robot_input_index = input_index_map.drake_to_robot[i];
    if (params.joint_is_force_controlled(drake_input_index)) {
      msg.effort[robot_input_index] = qp_output->u(drake_input_index);
    } else {
      msg.effort[robot_input_index] = 0;
    }
  }
  for (int i=0; i < state_to_drake_input_map.size(); i++) {
    state_index = i;
    drake_input_index = state_to_drake_input_map[i];
    if (drake_input_index != -1) {
      robot_input_index = input_index_map.drake_to_robot[drake_input_index];
      msg.position[robot_input_index] = qp_output->q_ref(state_index);
      if (params.joint_is_position_controlled(drake_input_index)) {
        msg.velocity[robot_input_index] = 0;
      } else {
        msg.velocity[robot_input_index] = qp_output->qd_ref(state_index);
      }
    }
  }
  updateGains(&params.gains);
  return &msg;
}
