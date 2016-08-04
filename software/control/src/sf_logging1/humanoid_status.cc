#include "humanoid_status.h"
#include <iostream>

const Vector3d HumanoidStatus::kFootToContactOffset = Vector3d(0, 0, -0.09);
const Vector3d HumanoidStatus::kFootToSensorOffset =
    Vector3d(0.0215646, 0.0, -0.051054);

/*
void HumanoidStatus::FillKinematics(const RigidBody& body, Isometry3d* pose,
                                    Vector6d* vel, MatrixXd* J,
                                    Vector6d* Jdot_times_v,
                                    const Vector3d& local_offset) const {
  *pose = Isometry3d::Identity();
  pose->translation() = local_offset;
  *pose = robot_->relativeTransform(cache_, 0, body.body_index) * (*pose);

  *vel = GetTaskSpaceVel(*(robot_), cache_, body, local_offset);
  *J = GetTaskSpaceJacobian(*(robot_), cache_, body, local_offset);
  *Jdot_times_v =
      GetTaskSpaceJacobianDotTimesV(*(robot_), cache_, body, local_offset);
}
*/

void HumanoidStatus::Update(double t, const VectorXd& q, const VectorXd& v,
                            const VectorXd& trq, const Vector6d& l_ft,
                            const Vector6d& r_ft, const Matrix3d& rot) {
  if (q.size() != position_.size() || v.size() != velocity_.size() ||
      trq.size() != joint_torque_.size()) {
    throw std::runtime_error("robot_ state update dimension mismatch");
  }

  time_ = t;
  position_ = q;
  velocity_ = v;
  joint_torque_ = trq;

  cache_.initialize(position_, velocity_);
  robot_->doKinematics(cache_, true);

  M_ = robot_->massMatrix(cache_);
  eigen_aligned_unordered_map<RigidBody const*, Matrix<double, TWIST_SIZE, 1> >
      f_ext;
  bias_term_ = robot_->dynamicsBiasTerm(cache_, f_ext);

  // com
  com_ = robot_->centerOfMass(cache_);
  J_com_ = robot_->centerOfMassJacobian(cache_);
  Jdot_times_v_com_ = robot_->centerOfMassJacobianDotTimesV(cache_);
  comd_ = J_com_ * v;

  // body parts
  /*
  FillKinematics(*pelv_.body, &pelv_.pose, &pelv_.vel, &pelv_.J,
                 &pelv_.Jdot_times_v);
  FillKinematics(*torso_.body, &torso_.pose, &torso_.vel, &torso_.J,
                 &torso_.Jdot_times_v);
  */
  pelv_.Update(*robot_, cache_);
  torso_.Update(*robot_, cache_);
  for (int s = 0; s < 2; s++) {
    /*
    FillKinematics(*foot_[s].body, &foot_[s].pose, &foot_[s].vel, &foot_[s].J,
                   &foot_[s].Jdot_times_v, kFootToContactOffset);
    FillKinematics(*foot_sensor_[s].body, &foot_sensor_[s].pose,
                   &foot_sensor_[s].vel, &foot_sensor_[s].J,
                   &foot_sensor_[s].Jdot_times_v, kFootToSensorOffset);
    */
    foot_[s].Update(*robot_, cache_);
    foot_sensor_[s].Update(*robot_, cache_);
  }

  // ft sensor
  foot_wrench_in_body_frame_[Side::LEFT] = l_ft;
  foot_wrench_in_body_frame_[Side::RIGHT] = r_ft;
  for (int i = 0; i < 2; i++) {
    foot_wrench_in_body_frame_[i].head(3) =
        rot * foot_wrench_in_body_frame_[i].head(3);
    foot_wrench_in_body_frame_[i].tail(3) =
        rot * foot_wrench_in_body_frame_[i].tail(3);
    foot_wrench_in_world_frame_[i].head(3) =
        foot_sensor_[i].pose.linear() * foot_wrench_in_body_frame_[i].head(3);
    foot_wrench_in_world_frame_[i].tail(3) =
        foot_sensor_[i].pose.linear() * foot_wrench_in_body_frame_[i].tail(3);
  }

  // cop
  Vector2d cop_w[2];
  double Fz[2];
  for (int i = 0; i < 2; i++) {
    Fz[i] = foot_wrench_in_world_frame_[i][5];
    if (fabs(Fz[i]) < 1e-3) {
      cop_in_body_frame_[i][0] = 0;
      cop_in_body_frame_[i][1] = 0;
      cop_w[i][0] = foot_sensor_[i].pose.translation()[0];
      cop_w[i][1] = foot_sensor_[i].pose.translation()[1];
    } else {
      // cop relative to the ft sensor
      cop_in_body_frame_[i][0] =
          -foot_wrench_in_body_frame_[i][1] / foot_wrench_in_body_frame_[i][5];
      cop_in_body_frame_[i][1] =
          foot_wrench_in_body_frame_[i][0] / foot_wrench_in_body_frame_[i][5];

      cop_w[i][0] = -foot_wrench_in_world_frame_[i][1] / Fz[i] +
                    foot_sensor_[i].pose.translation()[0];
      cop_w[i][1] = foot_wrench_in_world_frame_[i][0] / Fz[i] +
                    foot_sensor_[i].pose.translation()[1];
    }
  }

  // This is assuming that both feet are on the same horizontal surface.
  cop_ = (cop_w[Side::LEFT] * Fz[Side::LEFT] +
          cop_w[Side::RIGHT] * Fz[Side::RIGHT]) /
         (Fz[Side::LEFT] + Fz[Side::RIGHT]);
}

void HumanoidStatus::AddToLog(MRDLogger &logger) const {
  logger.AddChannel("time", "s", &time_);

  pelv_.AddToLog("RS.", logger);
  foot_[0].AddToLog("RS.", logger);
  foot_[1].AddToLog("RS.", logger);
  torso_.AddToLog("RS.", logger);

  logger.AddChannel("RS.com[x]", "m", com_.data());
  logger.AddChannel("RS.com[y]", "m", com_.data()+1);
  logger.AddChannel("RS.com[z]", "m", com_.data()+2);
  logger.AddChannel("RS.comd[x]", "m/s", comd_.data());
  logger.AddChannel("RS.comd[y]", "m/s", comd_.data()+1);
  logger.AddChannel("RS.comd[z]", "m/s", comd_.data()+2);

  logger.AddChannel("RS.cop[x]", "m", cop_.data());
  logger.AddChannel("RS.cop[y]", "m", cop_.data()+1);

  logger.AddChannel("RS.F_w[L][x]", "N", foot_wrench_in_world_frame_[Side::LEFT].data()+3);
  logger.AddChannel("RS.F_w[L][y]", "N", foot_wrench_in_world_frame_[Side::LEFT].data()+4);
  logger.AddChannel("RS.F_w[L][z]", "N", foot_wrench_in_world_frame_[Side::LEFT].data()+5);
  logger.AddChannel("RS.M_w[L][x]", "Nm", foot_wrench_in_world_frame_[Side::LEFT].data()+0);
  logger.AddChannel("RS.M_w[L][y]", "Nm", foot_wrench_in_world_frame_[Side::LEFT].data()+1);
  logger.AddChannel("RS.M_w[L][z]", "Nm", foot_wrench_in_world_frame_[Side::LEFT].data()+1);

  logger.AddChannel("RS.F_w[R][x]", "N", foot_wrench_in_world_frame_[Side::RIGHT].data()+3);
  logger.AddChannel("RS.F_w[R][y]", "N", foot_wrench_in_world_frame_[Side::RIGHT].data()+4);
  logger.AddChannel("RS.F_w[R][z]", "N", foot_wrench_in_world_frame_[Side::RIGHT].data()+5);
  logger.AddChannel("RS.M_w[R][x]", "Nm", foot_wrench_in_world_frame_[Side::RIGHT].data()+0);
  logger.AddChannel("RS.M_w[R][y]", "Nm", foot_wrench_in_world_frame_[Side::RIGHT].data()+1);
  logger.AddChannel("RS.M_w[R][z]", "Nm", foot_wrench_in_world_frame_[Side::RIGHT].data()+1);

  for (int i = 0; i < position_.size(); i++)
    logger.AddChannel("RS.q["+robot_->getPositionName(i)+"]", "rad", position_.data()+i);
  for (int i = 0; i < velocity_.size(); i++)
    logger.AddChannel("RS.v["+robot_->getPositionName(i)+"]", "rad/s", velocity_.data()+i);
  // TODO: don't hard code this
  for (int i = 0; i < joint_torque_.size(); i++)
    logger.AddChannel("RS.trq["+robot_->getPositionName(i+6)+"]", "Nm", joint_torque_.data()+i);
}

void HumanoidStatus::Init(const bot_core::robot_state_t &msg)
{
  inited_ = true;
  time0_ = msg.utime / 1e6;
}

void HumanoidStatus::ParseMsg(const bot_core::robot_state_t &msg)
{
  double time = (double)msg.utime / 1e6 - time0_;

  // extract q, qd from msg
  DrakeRobotState robot_state;
  robot_state.q.resize(robot_->num_positions);
  robot_state.qd.resize(robot_->num_velocities);
  state_driver_->decode(&msg, &robot_state);
  auto floating_map = state_driver_->get_floating_joint_map();
  auto state_map = state_driver_->get_joint_map();

  // extract trq
  for (size_t i = 0; i < msg.joint_effort.size(); i++) {
    std::string state_name = msg.joint_name[i];
    // a normal joint
    if (floating_map.find(state_name) == floating_map.end()) {
      auto it = state_map.find(state_name);
      if (it != state_map.end()) {
        int idx = it->second;
        joint_torque_[idx-6] = msg.joint_effort[i];
      }
    }
  }

  Vector6d raw_ft[2];
  raw_ft[Side::LEFT][3] = msg.force_torque.l_foot_force_x;
  raw_ft[Side::LEFT][4] = msg.force_torque.l_foot_force_y;
  raw_ft[Side::LEFT][5] = msg.force_torque.l_foot_force_z;
  raw_ft[Side::LEFT][0] = msg.force_torque.l_foot_torque_x;
  raw_ft[Side::LEFT][1] = msg.force_torque.l_foot_torque_y;
  raw_ft[Side::LEFT][2] = msg.force_torque.l_foot_torque_z;

  raw_ft[Side::RIGHT][3] = msg.force_torque.r_foot_force_x;
  raw_ft[Side::RIGHT][4] = msg.force_torque.r_foot_force_y;
  raw_ft[Side::RIGHT][5] = msg.force_torque.r_foot_force_z;
  raw_ft[Side::RIGHT][0] = msg.force_torque.r_foot_torque_x;
  raw_ft[Side::RIGHT][1] = msg.force_torque.r_foot_torque_y;
  raw_ft[Side::RIGHT][2] = msg.force_torque.r_foot_torque_z;

  Matrix3d rot(AngleAxisd(M_PI, Vector3d::UnitX()));

  Update(time, robot_state.q, robot_state.qd, joint_torque_, raw_ft[0], raw_ft[1], rot);
}
