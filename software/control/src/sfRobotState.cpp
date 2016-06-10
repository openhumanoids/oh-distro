#include "sfRobotState.h"
#include <iostream>

void sfRobotState::init(const bot_core::robot_state_t &msg) 
{
  this->joint_name_to_id.clear();
  for (int i = 0; i < msg.joint_name.size(); i++) {
    this->joint_name_to_id[msg.joint_name[i]] = i;
  }

  this->_inited = true;
}

void sfRobotState::parseMsg(const bot_core::robot_state_t &msg)
{
  // extract q, qd from msg
  robot_state.q.resize(robot->num_positions);
  robot_state.qd.resize(robot->num_velocities);
  state_driver->decode(&msg, &robot_state);

  this->q = robot_state.q;
  this->qd = robot_state.qd;

  this->cache.initialize(this->q, this->qd);
  this->robot->doKinematics(cache, true);

  int foot_id[2], pelv_id;
  foot_id[0] = body_or_frame_name_to_id.at("leftFoot");
  foot_id[1] = body_or_frame_name_to_id.at("rightFoot");
  pelv_id = body_or_frame_name_to_id.at("pelvis");

  KinematicPath body_path;
  MatrixXd Jcompact;

  // com
  this->com = this->robot->centerOfMass(cache);

  this->J_com = this->robot->centerOfMassJacobian(cache);
  this->Jdqd_com = this->robot->centerOfMassJacobianDotTimesV(cache);
  this->comd = this->J_com * this->qd;

  // pelvis
  this->pelv = this->robot->relativeTransform(cache, 0, pelv_id);

  body_path = this->robot->findKinematicPath(0, pelv_id);
  Jcompact = this->robot->geometricJacobian(this->cache, 0, pelv_id, pelv_id, true);
  this->J_pelv = this->robot->compactToFull(Jcompact, body_path.joint_path, true);
  this->Jdqd_pelv = this->robot->geometricJacobianDotTimesV(cache, 0, pelv_id, pelv_id);

  this->pelvd = this->J_pelv * this->qd;

  // feet 
  Eigen::Isometry3d ft_offset = Translation3d(Vector3d(0.0215646, 0.0, -0.051054)) * AngleAxisd(M_PI, Vector3d(1, 0, 0));
  Eigen::Isometry3d ft_rot_offset(AngleAxisd(M_PI, Vector3d(1, 0, 0)));
  for (int i = 0; i < 2; i++) {
    this->foot[i] = this->robot->relativeTransform(cache, 0, foot_id[i]);
    this->foot_ft_sensor[i] = this->foot[i] * ft_offset;

    body_path = this->robot->findKinematicPath(0, foot_id[i]);
    Jcompact = this->robot->geometricJacobian(this->cache, 0, foot_id[i], foot_id[i], true);
    this->J_foot[i] = this->robot->compactToFull(Jcompact, body_path.joint_path, true);
    this->Jdqd_foot[i] = this->robot->geometricJacobianDotTimesV(cache, 0, foot_id[i], foot_id[i]);

    this->footd[i] = this->J_foot[i] * this->qd; 
  }
  
  // foot ft, TODO, add the other 3
  this->footFT_b[0].setZero();
  this->footFT_b[0][2] = msg.force_torque.l_foot_force_z;
  this->footFT_b[0][3] = msg.force_torque.l_foot_torque_x;
  this->footFT_b[0][4] = msg.force_torque.l_foot_torque_y;
  this->footFT_b[1].setZero();
  this->footFT_b[1][2] = msg.force_torque.r_foot_force_z;
  this->footFT_b[1][3] = msg.force_torque.r_foot_torque_x;
  this->footFT_b[1][4] = msg.force_torque.r_foot_torque_y;

  for (int i = 0; i < 2; i++) {
    // rotate the ft measurement to the same orientation as the foot
    this->footFT_b[i].segment<3>(0) = ft_rot_offset.rotation() * this->footFT_b[i].segment<3>(0);
    this->footFT_b[i].segment<3>(3) = ft_rot_offset.rotation() * this->footFT_b[i].segment<3>(3);

    this->footFT_w[i].segment<3>(0) = this->foot[i].rotation() * this->footFT_b[i].segment<3>(0);
    this->footFT_w[i].segment<3>(3) = this->foot[i].rotation() * this->footFT_b[i].segment<3>(3);
  }
  
  // cop
  Vector2d cop_w[2];
  for (int i = 0; i < 2; i++) {
    // cop relative to the ft sensor
    this->cop_b[i][0] = -this->footFT_b[i][4] / this->footFT_b[i][2];
    this->cop_b[i][1] = this->footFT_b[i][3] / this->footFT_b[i][2];
    cop_w[i] = this->cop_b[i] + this->foot_ft_sensor[i].translation().segment<2>(0);
  }
  this->cop = (cop_w[0] * this->footFT_b[0][2] + cop_w[1] * this->footFT_b[1][2]) / (this->footFT_b[0][2] + this->footFT_b[1][2]);

  // joint data
  for (int i = 0; i < this->pos.size(); i++) {
    auto it = this->joint_name_to_id.find(this->robot->getPositionName(i));
    if (it == this->joint_name_to_id.end()) {
      this->pos[i] = this->vel[i] = this->trq[i] = 0;
    }
    else {
      this->pos[i] = msg.joint_position[it->second];
      this->vel[i] = msg.joint_velocity[it->second];
      this->trq[i] = msg.joint_effort[it->second];
    }
  }
}

void sfQPOutput::parseMsg(const drc::controller_state_t &msg, const sfRobotState &rs)
{
  for (int i = 0; i < msg.num_joints; i++) {
    this->qdd[i] = msg.qdd[i];
    this->trq[i] = msg.u[i]; // first 6 = zero
  }
  
  this->comdd = rs.J_com * this->qdd + rs.Jdqd_com;
  this->pelvdd = rs.J_pelv * this->qdd + rs.Jdqd_pelv;
  for (int i = 0; i < 2; i++)
    this->footdd[i] = rs.J_foot[i] * this->qdd + rs.Jdqd_foot[i];
}

