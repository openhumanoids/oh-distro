#include "sfRobotState.h"
#include <iostream>

void sfRobotState::init(const bot_core::robot_state_t &msg) 
{
  this->joint_name_to_id.clear();
  for (int i = 0; i < msg.joint_name.size(); i++) {
    this->joint_name_to_id[msg.joint_name[i]] = i;
  }

  _time0 = msg.utime / 1e6;
  this->_inited = true;
}

void sfRobotState::_fillKinematics(const std::string &name, Isometry3d &pose, Vector6d &vel, MatrixXd &J, Vector6d &Jdv, const Vector3d &local_offset)
{
  int id = body_or_frame_name_to_id.at(name);
  pose = Isometry3d::Identity();
  pose.translation() = local_offset;
  pose = this->robot->relativeTransform(cache, 0, id) * pose;

  vel = getTaskSpaceVel(*(this->robot), this->cache, id, local_offset);
  J = getTaskSpaceJacobian(*(this->robot), this->cache, id, local_offset);
  Jdv = getTaskSpaceJacobianDotTimesV(*(this->robot), this->cache, id, local_offset);
}

// assumeing init is called. otherwise memory isn't quite right
void sfRobotState::addToLog(Logger &logger) const
{
  logger.add_datapoint("time", "s", &time);
  
  pelv.addToLog(logger);
  l_foot.addToLog(logger);
  r_foot.addToLog(logger);
  torso.addToLog(logger);

  logger.add_datapoint("com[x]", "m", com.data());
  logger.add_datapoint("com[y]", "m", com.data()+1);
  logger.add_datapoint("com[z]", "m", com.data()+2);
  logger.add_datapoint("comd[x]", "m/s", comd.data());
  logger.add_datapoint("comd[y]", "m/s", comd.data()+1);
  logger.add_datapoint("comd[z]", "m/s", comd.data()+2);

  logger.add_datapoint("cop[x]", "m", cop.data());
  logger.add_datapoint("cop[y]", "m", cop.data()+1);

  logger.add_datapoint("F_w[L][x]", "N", footFT_w[Side::LEFT].data()+3);
  logger.add_datapoint("F_w[L][y]", "N", footFT_w[Side::LEFT].data()+4);
  logger.add_datapoint("F_w[L][z]", "N", footFT_w[Side::LEFT].data()+5);
  logger.add_datapoint("M_w[L][x]", "Nm", footFT_w[Side::LEFT].data()+0);
  logger.add_datapoint("M_w[L][y]", "Nm", footFT_w[Side::LEFT].data()+1);
  logger.add_datapoint("M_w[L][z]", "Nm", footFT_w[Side::LEFT].data()+1);
  logger.add_datapoint("F_w[R][x]", "N", footFT_w[Side::RIGHT].data()+3);
  logger.add_datapoint("F_w[R][y]", "N", footFT_w[Side::RIGHT].data()+4);
  logger.add_datapoint("F_w[R][z]", "N", footFT_w[Side::RIGHT].data()+5);
  logger.add_datapoint("M_w[R][x]", "Nm", footFT_w[Side::RIGHT].data()+0);
  logger.add_datapoint("M_w[R][y]", "Nm", footFT_w[Side::RIGHT].data()+1);
  logger.add_datapoint("M_w[R][z]", "Nm", footFT_w[Side::RIGHT].data()+1);

  for (int i = 0; i < pos.size(); i++)
    logger.add_datapoint("q["+robot->getPositionName(i)+"]", "rad", pos.data()+i);
  for (int i = 0; i < vel.size(); i++)
    logger.add_datapoint("v["+robot->getPositionName(i)+"]", "rad/s", vel.data()+i);
  for (int i = 0; i < trq.size(); i++)
    logger.add_datapoint("trq["+robot->getPositionName(i)+"]", "Nm", trq.data()+i);
}

void sfRobotState::parseMsg(const bot_core::robot_state_t &msg)
{
  time = (double)msg.utime / 1e6 - _time0;

  // extract q, qd from msg
  robot_state.q.resize(robot->num_positions);
  robot_state.qd.resize(robot->num_velocities);
  state_driver->decode(&msg, &robot_state);

  this->q = robot_state.q;
  this->qd = robot_state.qd;

  this->cache.initialize(this->q, this->qd);
  this->robot->doKinematics(cache, true);

  // com
  this->com = this->robot->centerOfMass(cache);

  this->J_com = this->robot->centerOfMassJacobian(cache);
  this->Jdv_com = this->robot->centerOfMassJacobianDotTimesV(cache);
  this->comd = this->J_com * this->qd;

  _fillKinematics(pelv.name, pelv.pose, pelv.vel, pelv.J, pelv.Jdv);
  _fillKinematics(l_foot.name, l_foot.pose, l_foot.vel, l_foot.J, l_foot.Jdv);
  _fillKinematics(r_foot.name, r_foot.pose, r_foot.vel, r_foot.J, r_foot.Jdv);
  _fillKinematics(torso.name, torso.pose, torso.vel, torso.J, torso.Jdv);
  
  Eigen::Isometry3d ft_offset = Translation3d(Vector3d(0.0215646, 0.0, -0.051054)) * AngleAxisd(M_PI, Vector3d::UnitX());
  this->foot_ft_sensor[Side::LEFT] = l_foot.pose * ft_offset;
  this->foot_ft_sensor[Side::RIGHT] = r_foot.pose * ft_offset;
  
  this->footFT_b[Side::LEFT].setZero();
  this->footFT_b[Side::LEFT][3] = msg.force_torque.l_foot_force_x;
  this->footFT_b[Side::LEFT][4] = msg.force_torque.l_foot_force_y;
  this->footFT_b[Side::LEFT][5] = msg.force_torque.l_foot_force_z;
  this->footFT_b[Side::LEFT][0] = msg.force_torque.l_foot_torque_x;
  this->footFT_b[Side::LEFT][1] = msg.force_torque.l_foot_torque_y;
  this->footFT_b[Side::LEFT][2] = msg.force_torque.l_foot_torque_z;
  this->footFT_b[Side::RIGHT].setZero();
  this->footFT_b[Side::RIGHT][3] = msg.force_torque.r_foot_force_x;
  this->footFT_b[Side::RIGHT][4] = msg.force_torque.r_foot_force_y;
  this->footFT_b[Side::RIGHT][5] = msg.force_torque.r_foot_force_z;
  this->footFT_b[Side::RIGHT][0] = msg.force_torque.r_foot_torque_x;
  this->footFT_b[Side::RIGHT][1] = msg.force_torque.r_foot_torque_y;
  this->footFT_b[Side::RIGHT][2] = msg.force_torque.r_foot_torque_z;

  for (int i = 0; i < 2; i++) {
    // rotate the ft measurement to the same orientation as the foot
    this->footFT_b[i].segment<3>(0) = ft_offset.linear() * this->footFT_b[i].segment<3>(0);
    this->footFT_b[i].segment<3>(3) = ft_offset.linear() * this->footFT_b[i].segment<3>(3);

    this->footFT_w[i].segment<3>(0) = foot[i]->pose.linear() * this->footFT_b[i].segment<3>(0);
    this->footFT_w[i].segment<3>(3) = foot[i]->pose.linear() * this->footFT_b[i].segment<3>(3);

    MatrixXd test = foot[i]->J;
    test.block(0,0,6,6).setZero();
    this->footFT_w_statics[i] = -(test.transpose().fullPivHouseholderQr().solve(this->trq));
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

Vector6d getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset)
{
  Isometry3d H_body_to_frame;
  int body_idx = r.parseBodyOrFrameID(body_or_frame_id, &H_body_to_frame);

  const auto &element = cache.getElement(*(r.bodies[body_idx]));
  Vector6d T = element.twist_in_world;
  Vector3d pt = element.transform_to_world.translation();
  
  // get the body's task space vel
  Vector6d v = T;
  v.tail<3>() += v.head<3>().cross(pt); 

  // global offset between pt and body
  auto H_world_to_frame = element.transform_to_world * H_body_to_frame;
  Isometry3d H_frame_to_pt(Isometry3d::Identity());
  H_frame_to_pt.translation() = local_offset;
  auto H_world_to_pt = H_world_to_frame * H_frame_to_pt; 
  Vector3d world_offset = H_world_to_pt.translation() - element.transform_to_world.translation();
  
  // add the linear vel from the body rotation
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, KinematicsCache<double> &cache, int body, const Vector3d &local_offset)
{
  std::vector<int> v_or_q_indices;
  KinematicPath body_path = r.findKinematicPath(0, body);
  MatrixXd Jg = r.geometricJacobian(cache, 0, body, 0, true, &v_or_q_indices);
  MatrixXd J(6, r.num_velocities);
  J.setZero();

  Vector3d points = r.transformPoints(cache, local_offset, body, 0);

  int col = 0;
  for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
    // angular
    J.template block<SPACE_DIMENSION, 1>(0,*it) = Jg.block<3,1>(0,col);
    // linear, just like the linear velocity, assume qd = 1, the column is the linear velocity.
    J.template block<SPACE_DIMENSION, 1>(3,*it) = Jg.block<3,1>(3,col);
    J.template block<SPACE_DIMENSION, 1>(3,*it).noalias() += Jg.block<3,1>(0,col).cross(points);
    col++;
  }

  return J;
}

Vector6d getTaskSpaceJacobianDotTimesV(const RigidBodyTree &r, KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset)
{
  // position of point in world
  Vector3d p = r.transformPoints(cache, local_offset, body_or_frame_id, 0);
  Vector6d twist = r.relativeTwist(cache, 0, body_or_frame_id, 0);
  Vector6d J_geometric_dot_times_v = r.geometricJacobianDotTimesV(cache, 0, body_or_frame_id, 0);

  // linear vel of r
  Vector3d pdot = twist.head<3>().cross(p) + twist.tail<3>();

  // each column of J_task Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // Jt * v, angular part stays the same, 
  // linear part = [\dot{Jg_v}v + \dot{Jg_omega}.cross(p) + Jg_omega.cross(rdot)] * v 
  //             = [lin of JgdotV + ang of JgdotV.cross(p) + omega.cross(rdot)]
  Vector6d Jdv = J_geometric_dot_times_v;
  Jdv.tail<3>() += twist.head<3>().cross(pdot) + J_geometric_dot_times_v.head<3>().cross(p);

  return Jdv;
}
 






void sfQPOutput::parseMsg(const drc::controller_state_t &msg, const sfRobotState &rs)
{
  for (int i = 0; i < msg.num_joints; i++) {
    this->qdd[i] = msg.qdd[i];
    this->trq[i] = msg.u[i]; // first 6 = zero
  }
  
  this->comdd = rs.J_com * this->qdd + rs.Jdv_com;
  this->pelvdd = rs.pelv.J * this->qdd + rs.pelv.Jdv;
  this->torsodd = rs.torso.J * this->qdd + rs.pelv.Jdv;
  for (int i = 0; i < 2; i++)
    this->footdd[i] = rs.foot[i]->J * this->qdd + rs.foot[i]->Jdv;
}

void sfQPOutput::addToLog(Logger &logger, const sfRobotState &rs) const
{
  logger.add_datapoint("QP.comdd[x]", "m/s2", comdd.data()+0);
  logger.add_datapoint("QP.comdd[y]", "m/s2", comdd.data()+1);
  logger.add_datapoint("QP.comdd[z]", "m/s2", comdd.data()+2);

  logger.add_datapoint("QP.pelvdd[x]", "m/s2", pelvdd.data()+3);
  logger.add_datapoint("QP.pelvdd[y]", "m/s2", pelvdd.data()+4);
  logger.add_datapoint("QP.pelvdd[z]", "m/s2", pelvdd.data()+5);
  logger.add_datapoint("QP.pelvdd[wx]", "rad/s2", pelvdd.data()+0);
  logger.add_datapoint("QP.pelvdd[wy]", "rad/s2", pelvdd.data()+1);
  logger.add_datapoint("QP.pelvdd[wz]", "rad/s2", pelvdd.data()+2);

  logger.add_datapoint("QP.torsodd[x]", "m/s2", torsodd.data()+3);
  logger.add_datapoint("QP.torsodd[y]", "m/s2", torsodd.data()+4);
  logger.add_datapoint("QP.torsodd[z]", "m/s2", torsodd.data()+5);
  logger.add_datapoint("QP.torsodd[wx]", "rad/s2", torsodd.data()+0);
  logger.add_datapoint("QP.torsodd[wy]", "rad/s2", torsodd.data()+1);
  logger.add_datapoint("QP.torsodd[wz]", "rad/s2", torsodd.data()+2);

  logger.add_datapoint("QP.footdd[L][x]", "m/s2", footdd[Side::LEFT].data()+3);
  logger.add_datapoint("QP.footdd[L][y]", "m/s2", footdd[Side::LEFT].data()+4);
  logger.add_datapoint("QP.footdd[L][z]", "m/s2", footdd[Side::LEFT].data()+5);
  logger.add_datapoint("QP.footdd[L][wx]", "rad/s2", footdd[Side::LEFT].data()+0);
  logger.add_datapoint("QP.footdd[L][wy]", "rad/s2", footdd[Side::LEFT].data()+1);
  logger.add_datapoint("QP.footdd[L][wz]", "rad/s2", footdd[Side::LEFT].data()+2);
  
  logger.add_datapoint("QP.footdd[R][x]", "m/s2", footdd[Side::RIGHT].data()+3);
  logger.add_datapoint("QP.footdd[R][y]", "m/s2", footdd[Side::RIGHT].data()+4);
  logger.add_datapoint("QP.footdd[R][z]", "m/s2", footdd[Side::RIGHT].data()+5);
  logger.add_datapoint("QP.footdd[R][wx]", "rad/s2", footdd[Side::RIGHT].data()+0);
  logger.add_datapoint("QP.footdd[R][wy]", "rad/s2", footdd[Side::RIGHT].data()+1);
  logger.add_datapoint("QP.footdd[R][wz]", "rad/s2", footdd[Side::RIGHT].data()+2);

  for (int i = 0; i < qdd.size(); i++) 
    logger.add_datapoint("QP.qdd["+rs.robot->getPositionName(i)+"]", "rad/s2", qdd.data()+i);
  
  for (int i = 0; i < trq.size(); i++) 
    logger.add_datapoint("QP.trq["+rs.robot->getPositionName(i)+"]", "Nm", trq.data()+i);

  logger.add_datapoint("QP.F[L][x]", "N", grf[Side::LEFT].data()+3);
  logger.add_datapoint("QP.F[L][y]", "N", grf[Side::LEFT].data()+4);
  logger.add_datapoint("QP.F[L][z]", "N", grf[Side::LEFT].data()+5);
  logger.add_datapoint("QP.M[L][x]", "Nm", grf[Side::LEFT].data()+0);
  logger.add_datapoint("QP.M[L][y]", "Nm", grf[Side::LEFT].data()+1);
  logger.add_datapoint("QP.M[L][z]", "Nm", grf[Side::LEFT].data()+2);
  logger.add_datapoint("QP.F[R][x]", "N", grf[Side::RIGHT].data()+3);
  logger.add_datapoint("QP.F[R][y]", "N", grf[Side::RIGHT].data()+4);
  logger.add_datapoint("QP.F[R][z]", "N", grf[Side::RIGHT].data()+5);
  logger.add_datapoint("QP.M[R][x]", "Nm", grf[Side::RIGHT].data()+0);
  logger.add_datapoint("QP.M[R][y]", "Nm", grf[Side::RIGHT].data()+1);
  logger.add_datapoint("QP.M[R][z]", "Nm", grf[Side::RIGHT].data()+2);
}
