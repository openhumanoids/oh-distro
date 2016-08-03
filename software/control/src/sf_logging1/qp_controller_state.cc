#include "qp_controller_state.h"

void QPIO::ParseZMPInput(const drake::lcmt_qp_controller_input &msg)
{
  A_ls = Map<const Matrix<double, 4, 4, RowMajor>>(&msg.zmp_data.A[0][0]);
  B_ls = Map<const Matrix<double, 4, 2, RowMajor>>(&msg.zmp_data.B[0][0]);
  C_ls = Map<const Matrix<double, 2, 4, RowMajor>>(&msg.zmp_data.C[0][0]);
  D_ls = Map<const Matrix<double, 2, 2, RowMajor>>(&msg.zmp_data.D[0][0]);
  x0 = Map<const Matrix<double, 4, 1>>(&msg.zmp_data.x0[0][0]);
  y0 = Map<const Matrix<double, 2, 1>>(&msg.zmp_data.y0[0][0]);
  u0 = Map<const Matrix<double, 2, 1>>(&msg.zmp_data.u0[0][0]);
  R_ls = Map<const Matrix<double, 2, 2, RowMajor>>(&msg.zmp_data.R[0][0]);
  Qy = Map<const Matrix<double, 2, 2, RowMajor>>(&msg.zmp_data.Qy[0][0]);
  S = Map<const Matrix<double, 4, 4, RowMajor>>(&msg.zmp_data.S[0][0]);
  s1 = Map<const Matrix<double, 4, 1>>(&msg.zmp_data.s1[0][0]);
  s1dot = Map<const Matrix<double, 4, 1>>(&msg.zmp_data.s1dot[0][0]);
  _hasZMPInput = true;
}

void QPIO::ParseMsg(const drc::controller_state_t &msg, const HumanoidStatus &rs)
{
  // output
  for (int i = 0; i < msg.num_joints; i++) {
    qdd[i] = msg.qdd[i];
    trq[i] = msg.u[i]; // first 6 = zero
  }

  comdd = rs.J_com() * qdd + rs.Jdot_times_v_com();
  pelvdd = rs.pelv().J * qdd + rs.pelv().Jdot_times_v;
  torsodd = rs.torso().J * qdd + rs.torso().Jdot_times_v;
  for (int i = 0; i < 2; i++)
    footdd[i] = rs.foot(i).J * qdd + rs.foot(i).Jdot_times_v;

  // TODO: should not hard code this
  Vector3d grf_loc[2];
  for (size_t i = 0; i < msg.contact_output.size(); i++) {
    int side = 0;
    if (msg.contact_output[i].body_name.compare("leftFoot") == 0)
      side = 0;
    else if (msg.contact_output[i].body_name.compare("rightFoot") == 0)
      side = 1;
    else
      continue;

    for (int j = 0; j < 6; j++)
      grf_w[side][j] = msg.contact_output[i].wrench[j];
    for (int j = 0; j < 3; j++)
      grf_loc[side][j] = msg.contact_output[i].ref_point[j];
  }

  // transform this into the ft sensor location
  for (int i = 0; i < 2; i++) {
    Isometry3d H(Isometry3d::Identity());
    H.translation() = grf_loc[i] - rs.foot_sensor(i).pose.translation();

    grf_w[i] = transformSpatialForce(H, grf_w[i]);
  }

  // computes cop
  Vector2d cop_w[2];
  for (int i = 0; i < 2; i++) {
    Isometry3d H = rs.foot_sensor(i).pose.inverse();
    H.translation() = Vector3d::Zero();
    grf_b[i] = transformSpatialForce(H, grf_w[i]);
    cop_b[i][0] = -grf_b[i][1] / grf_b[i][5];
    cop_b[i][1] = grf_b[i][0] / grf_b[i][5];

    cop_w[i][0] = -grf_w[i][1] / grf_w[i][5] + rs.foot_sensor(i).pose.translation()[0];
    cop_w[i][1] = grf_w[i][0] / grf_w[i][5] + rs.foot_sensor(i).pose.translation()[1];
  }
  this->cop_w = (cop_w[Side::LEFT]*grf_w[Side::LEFT][5] + cop_w[Side::RIGHT]*grf_w[Side::RIGHT][5]) / (grf_w[Side::RIGHT][5] + grf_w[Side::LEFT][5]);

  // input
  for (int i = 0; i < msg.desired_body_motions.size(); i++) {
    const drc::qp_desired_body_motion_t &vdot_d = msg.desired_body_motions[i];

    if (vdot_d.body_name.compare("pelvis") == 0)
      pelv.ParseMsg(vdot_d);
    if (vdot_d.body_name.compare("torso") == 0)
      torso.ParseMsg(vdot_d);
    if (vdot_d.body_name.compare("leftFoot") == 0)
      foot[Side::LEFT].ParseMsg(vdot_d);
    if (vdot_d.body_name.compare("rightFoot") == 0)
      foot[Side::RIGHT].ParseMsg(vdot_d);
  }

  // comdd_d
  if (_hasZMPInput) {
    Vector4d xlimp, x_bar;
    Matrix2d R_DQyD_ls = R_ls + D_ls.transpose() * Qy * D_ls;

    xlimp.head(2) = rs.com().head(2);
    xlimp.tail(2) = rs.comd().head(2);
    x_bar = xlimp - x0;

    Matrix<double,4,2> N = C_ls.transpose()*Qy*D_ls;
    Matrix<double,2,4> N_B = N.transpose() + B_ls.transpose()*S;
    Matrix<double,2,1> y_d_bar = y0 - x0.head(2);
    Matrix<double,2,1> r_2 = -2*D_ls.transpose()*Qy*y_d_bar;
    Matrix<double,2,1> r_s = 0.5*(r_2 + B_ls.transpose()*s1);

    comdd_d.head(2) = -R_DQyD_ls.inverse() * (N_B*x_bar + r_s);

    Matrix<double,1,2> lin = -(C_ls*xlimp-y0).transpose() * Qy * D_ls
      //+ u0.transpose() * R_ls
      - (S*x_bar+0.5*s1).transpose() * B_ls;
    comdd_d1.head(2) = R_DQyD_ls.inverse() * lin.transpose();

    com_d.head(2) = x0.head(2);
    comd_d.head(2) = x0.tail(2);
    cop_d = y0;
  }
  else {
    com_d.setZero();
    comd_d.setZero();
    comdd_d.setZero();
  }
}

void QPIO::AddToLog(MRDLogger &logger, const HumanoidStatus &rs) const
{
  logger.AddChannel("QP_d.com[x]", "m/s2", com_d.data()+0);
  logger.AddChannel("QP_d.com[y]", "m/s2", com_d.data()+1);
  logger.AddChannel("QP_d.com[z]", "m/s2", com_d.data()+2);

  logger.AddChannel("QP_d.cop[x]", "m/s2", cop_d.data()+0);
  logger.AddChannel("QP_d.cop[y]", "m/s2", cop_d.data()+1);
  logger.AddChannel("QP.cop[x]", "m/s2", cop_w.data()+0);
  logger.AddChannel("QP.cop[y]", "m/s2", cop_w.data()+1);

  logger.AddChannel("QP_d.comd[x]", "m/s2", comd_d.data()+0);
  logger.AddChannel("QP_d.comd[y]", "m/s2", comd_d.data()+1);
  logger.AddChannel("QP_d.comd[z]", "m/s2", comd_d.data()+2);

  logger.AddChannel("QP_d.comdd_d[x]", "m/s2", comdd_d.data()+0);
  logger.AddChannel("QP_d.comdd_d[y]", "m/s2", comdd_d.data()+1);
  logger.AddChannel("QP_d.comdd_d[z]", "m/s2", comdd_d.data()+2);

  logger.AddChannel("QP_d.comdd_d1[x]", "m/s2", comdd_d1.data()+0);
  logger.AddChannel("QP_d.comdd_d1[y]", "m/s2", comdd_d1.data()+1);
  logger.AddChannel("QP_d.comdd_d1[z]", "m/s2", comdd_d1.data()+2);

  logger.AddChannel("QP.comdd[x]", "m/s2", comdd.data()+0);
  logger.AddChannel("QP.comdd[y]", "m/s2", comdd.data()+1);
  logger.AddChannel("QP.comdd[z]", "m/s2", comdd.data()+2);

  pelv.AddToLog(std::string("QP_d."), logger);
  logger.AddChannel("QP.pelvdd[x]", "m/s2", pelvdd.data()+3);
  logger.AddChannel("QP.pelvdd[y]", "m/s2", pelvdd.data()+4);
  logger.AddChannel("QP.pelvdd[z]", "m/s2", pelvdd.data()+5);
  logger.AddChannel("QP.pelvdd[wx]", "rad/s2", pelvdd.data()+0);
  logger.AddChannel("QP.pelvdd[wy]", "rad/s2", pelvdd.data()+1);
  logger.AddChannel("QP.pelvdd[wz]", "rad/s2", pelvdd.data()+2);

  torso.AddToLog(std::string("QP_d."), logger);
  logger.AddChannel("QP.torsodd[x]", "m/s2", torsodd.data()+3);
  logger.AddChannel("QP.torsodd[y]", "m/s2", torsodd.data()+4);
  logger.AddChannel("QP.torsodd[z]", "m/s2", torsodd.data()+5);
  logger.AddChannel("QP.torsodd[wx]", "rad/s2", torsodd.data()+0);
  logger.AddChannel("QP.torsodd[wy]", "rad/s2", torsodd.data()+1);
  logger.AddChannel("QP.torsodd[wz]", "rad/s2", torsodd.data()+2);

  foot[Side::LEFT].AddToLog(std::string("QP_d."), logger);
  logger.AddChannel("QP.footdd[L][x]", "m/s2", footdd[Side::LEFT].data()+3);
  logger.AddChannel("QP.footdd[L][y]", "m/s2", footdd[Side::LEFT].data()+4);
  logger.AddChannel("QP.footdd[L][z]", "m/s2", footdd[Side::LEFT].data()+5);
  logger.AddChannel("QP.footdd[L][wx]", "rad/s2", footdd[Side::LEFT].data()+0);
  logger.AddChannel("QP.footdd[L][wy]", "rad/s2", footdd[Side::LEFT].data()+1);
  logger.AddChannel("QP.footdd[L][wz]", "rad/s2", footdd[Side::LEFT].data()+2);

  foot[Side::LEFT].AddToLog(std::string("QP_d."), logger);
  logger.AddChannel("QP.footdd[R][x]", "m/s2", footdd[Side::RIGHT].data()+3);
  logger.AddChannel("QP.footdd[R][y]", "m/s2", footdd[Side::RIGHT].data()+4);
  logger.AddChannel("QP.footdd[R][z]", "m/s2", footdd[Side::RIGHT].data()+5);
  logger.AddChannel("QP.footdd[R][wx]", "rad/s2", footdd[Side::RIGHT].data()+0);
  logger.AddChannel("QP.footdd[R][wy]", "rad/s2", footdd[Side::RIGHT].data()+1);
  logger.AddChannel("QP.footdd[R][wz]", "rad/s2", footdd[Side::RIGHT].data()+2);

  for (int i = 0; i < qdd.size(); i++)
    logger.AddChannel("QP.qdd["+rs.robot().getPositionName(i)+"]", "rad/s2", qdd.data()+i);

  for (int i = 0; i < trq.size(); i++)
    logger.AddChannel("QP.trq["+rs.robot().getPositionName(i)+"]", "Nm", trq.data()+i);

  logger.AddChannel("QP.F[L][x]", "N", grf_w[Side::LEFT].data()+3);
  logger.AddChannel("QP.F[L][y]", "N", grf_w[Side::LEFT].data()+4);
  logger.AddChannel("QP.F[L][z]", "N", grf_w[Side::LEFT].data()+5);
  logger.AddChannel("QP.M[L][x]", "Nm", grf_w[Side::LEFT].data()+0);
  logger.AddChannel("QP.M[L][y]", "Nm", grf_w[Side::LEFT].data()+1);
  logger.AddChannel("QP.M[L][z]", "Nm", grf_w[Side::LEFT].data()+2);
  logger.AddChannel("QP.F[R][x]", "N", grf_w[Side::RIGHT].data()+3);
  logger.AddChannel("QP.F[R][y]", "N", grf_w[Side::RIGHT].data()+4);
  logger.AddChannel("QP.F[R][z]", "N", grf_w[Side::RIGHT].data()+5);
  logger.AddChannel("QP.M[R][x]", "Nm", grf_w[Side::RIGHT].data()+0);
  logger.AddChannel("QP.M[R][y]", "Nm", grf_w[Side::RIGHT].data()+1);
  logger.AddChannel("QP.M[R][z]", "Nm", grf_w[Side::RIGHT].data()+2);

  logger.AddChannel("QP.cop_b[L][x]", "m/s2", cop_b[Side::LEFT].data()+0);
  logger.AddChannel("QP.cop_b[L][y]", "m/s2", cop_b[Side::LEFT].data()+1);
  logger.AddChannel("QP.cop_b[R][x]", "m/s2", cop_b[Side::RIGHT].data()+0);
  logger.AddChannel("QP.cop_b[R][y]", "m/s2", cop_b[Side::RIGHT].data()+1);
}
