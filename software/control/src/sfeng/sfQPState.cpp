#include "sfQPState.h"

void sfQPState::parseZMPInput(const drake::lcmt_qp_controller_input &msg)
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


void sfQPState::parseMsg(const drc::controller_state_t &msg, const sfRobotState &rs)
{
  /*
  // random stuf
  Vector3d FF(1,2,3);
  Vector3d pt(0,1,0);

  Vector3d torque = pt.cross(FF);
  std::cout << "torque: " << torque.transpose() << std::endl;
  Isometry3d H(Isometry3d::Identity());
  H.translation() = pt;
  Vector6d WW(Vector6d::Zero());
  WW.tail(3) = FF;
  std::cout << "WW: " << transformSpatialForce(H, WW).transpose() << std::endl;
  H.translation() = -pt;
  std::cout << "WW1: " << transformSpatialForce(H, WW).transpose() << std::endl;
  */

  // output
  for (int i = 0; i < msg.num_joints; i++) {
    this->qdd[i] = msg.qdd[i];
    this->trq[i] = msg.u[i]; // first 6 = zero
  }
  
  this->comdd = rs.J_com * this->qdd + rs.Jdv_com;
  this->pelvdd = rs.pelv.J * this->qdd + rs.pelv.Jdv;
  this->torsodd = rs.torso.J * this->qdd + rs.pelv.Jdv;
  for (int i = 0; i < 2; i++)
    this->footdd[i] = rs.foot[i]->J * this->qdd + rs.foot[i]->Jdv;
  
  // TODO: should not hard code this
  Vector3d grf_loc[2];
  for (int i = 0; i < 6; i++) {
    this->grf_w[Side::LEFT][i] = msg.contact_wrenches[1][i];
    if (i < 3)
      grf_loc[Side::LEFT][i] = msg.contact_ref_points[1][i];
  }
  for (int i = 0; i < 6; i++) {
    this->grf_w[Side::RIGHT][i] = msg.contact_wrenches[0][i];
    if (i < 3)
      grf_loc[Side::RIGHT][i] = msg.contact_ref_points[0][i];
  }

  // transform this into the ft sensor location 
  for (int i = 0; i < 2; i++) {
    Isometry3d H(Isometry3d::Identity());
    H.translation() = grf_loc[i] - rs.foot_sensor[i]->pose.translation();

    this->grf_w[i] = transformSpatialForce(H, this->grf_w[i]);
  }

  // computes cop
  Vector2d cop_w[2];
  for (int i = 0; i < 2; i++) {
    Isometry3d H = rs.foot_sensor[i]->pose.inverse();
    H.translation() = Vector3d::Zero();
    this->grf_b[i] = transformSpatialForce(H, this->grf_w[i]);
    this->cop_b[i][0] = -this->grf_b[i][1] / this->grf_b[i][5];
    this->cop_b[i][1] = this->grf_b[i][0] / this->grf_b[i][5];

    cop_w[i][0] = -this->grf_w[i][1] / this->grf_w[i][5] + rs.foot_sensor[i]->pose.translation()[0]; 
    cop_w[i][1] = this->grf_w[i][0] / this->grf_w[i][5] + rs.foot_sensor[i]->pose.translation()[1]; 
  }
  this->cop_w = (cop_w[Side::LEFT]*this->grf_w[Side::LEFT][5] + cop_w[Side::RIGHT]*this->grf_w[Side::RIGHT][5]) / (this->grf_w[Side::RIGHT][5] + this->grf_w[Side::LEFT][5]);

  // input
  for (int i = 0; i < msg.desired_body_vdots.size(); i++) {
    // desired acc are in body frame
    const drc::qp_desired_body_acceleration_t &vdot_d = msg.desired_body_vdots[i];
    if (vdot_d.body_name.compare("pelvis") == 0) {
      for (int j = 0; j < 6; j++)
        this->pelvdd_d[j] = vdot_d.body_vdot[j];
      this->pelvdd_d.head(3) = rs.pelv.pose.linear() * this->pelvdd_d.head(3);
      this->pelvdd_d.tail(3) = rs.pelv.pose.linear() * this->pelvdd_d.tail(3);
    }
    if (vdot_d.body_name.compare("leftFoot") == 0) {
      for (int j = 0; j < 6; j++)
        this->footdd_d[Side::LEFT][j] = vdot_d.body_vdot[j];
      this->footdd_d[Side::LEFT].head(3) = rs.l_foot.pose.linear() * this->footdd_d[Side::LEFT].head(3);
      this->footdd_d[Side::LEFT].tail(3) = rs.l_foot.pose.linear() * this->footdd_d[Side::LEFT].tail(3);
    }
    if (vdot_d.body_name.compare("rightFoot") == 0) {
      for (int j = 0; j < 6; j++)
        this->footdd_d[Side::RIGHT][j] = vdot_d.body_vdot[j];
      this->footdd_d[Side::RIGHT].head(3) = rs.r_foot.pose.linear() * this->footdd_d[Side::RIGHT].head(3);
      this->footdd_d[Side::RIGHT].tail(3) = rs.r_foot.pose.linear() * this->footdd_d[Side::RIGHT].tail(3);
    }
  }

  // comdd_d
  if (_hasZMPInput) {
    Vector4d xlimp, x_bar;
    Matrix2d R_DQyD_ls = R_ls + D_ls.transpose() * Qy * D_ls;

    xlimp.head(2) = rs.com.head(2);
    xlimp.tail(2) = rs.comd.head(2);
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

void sfQPState::addToLog(MRDLogger &logger, const sfRobotState &rs) const
{
  logger.addChannel("QP_d.com[x]", "m/s2", com_d.data()+0);
  logger.addChannel("QP_d.com[y]", "m/s2", com_d.data()+1);
  logger.addChannel("QP_d.com[z]", "m/s2", com_d.data()+2);
  
  logger.addChannel("QP_d.cop[x]", "m/s2", cop_d.data()+0);
  logger.addChannel("QP_d.cop[y]", "m/s2", cop_d.data()+1);
  logger.addChannel("QP.cop[x]", "m/s2", cop_w.data()+0);
  logger.addChannel("QP.cop[y]", "m/s2", cop_w.data()+1);

  logger.addChannel("QP_d.comd[x]", "m/s2", comd_d.data()+0);
  logger.addChannel("QP_d.comd[y]", "m/s2", comd_d.data()+1);
  logger.addChannel("QP_d.comd[z]", "m/s2", comd_d.data()+2);

  logger.addChannel("QP_d.comdd[x]", "m/s2", comdd_d.data()+0);
  logger.addChannel("QP_d.comdd[y]", "m/s2", comdd_d.data()+1);
  logger.addChannel("QP_d.comdd[z]", "m/s2", comdd_d.data()+2);

  logger.addChannel("QP_d.comdd1[x]", "m/s2", comdd_d1.data()+0);
  logger.addChannel("QP_d.comdd1[y]", "m/s2", comdd_d1.data()+1);
  logger.addChannel("QP_d.comdd1[z]", "m/s2", comdd_d1.data()+2);
  
  logger.addChannel("QP.comdd[x]", "m/s2", comdd.data()+0);
  logger.addChannel("QP.comdd[y]", "m/s2", comdd.data()+1);
  logger.addChannel("QP.comdd[z]", "m/s2", comdd.data()+2);

  logger.addChannel("QP_d.pelvdd[x]", "m/s2", pelvdd_d.data()+3);
  logger.addChannel("QP_d.pelvdd[y]", "m/s2", pelvdd_d.data()+4);
  logger.addChannel("QP_d.pelvdd[z]", "m/s2", pelvdd_d.data()+5);
  logger.addChannel("QP_d.pelvdd[wx]", "rad/s2", pelvdd_d.data()+0);
  logger.addChannel("QP_d.pelvdd[wy]", "rad/s2", pelvdd_d.data()+1);
  logger.addChannel("QP_d.pelvdd[wz]", "rad/s2", pelvdd_d.data()+2);
  
  logger.addChannel("QP.pelvdd[x]", "m/s2", pelvdd.data()+3);
  logger.addChannel("QP.pelvdd[y]", "m/s2", pelvdd.data()+4);
  logger.addChannel("QP.pelvdd[z]", "m/s2", pelvdd.data()+5);
  logger.addChannel("QP.pelvdd[wx]", "rad/s2", pelvdd.data()+0);
  logger.addChannel("QP.pelvdd[wy]", "rad/s2", pelvdd.data()+1);
  logger.addChannel("QP.pelvdd[wz]", "rad/s2", pelvdd.data()+2);

  logger.addChannel("QP_d.torsodd[x]", "m/s2", torsodd_d.data()+3);
  logger.addChannel("QP_d.torsodd[y]", "m/s2", torsodd_d.data()+4);
  logger.addChannel("QP_d.torsodd[z]", "m/s2", torsodd_d.data()+5);
  logger.addChannel("QP_d.torsodd[wx]", "rad/s2", torsodd_d.data()+0);
  logger.addChannel("QP_d.torsodd[wy]", "rad/s2", torsodd_d.data()+1);
  logger.addChannel("QP_d.torsodd[wz]", "rad/s2", torsodd_d.data()+2);
  
  logger.addChannel("QP.torsodd[x]", "m/s2", torsodd.data()+3);
  logger.addChannel("QP.torsodd[y]", "m/s2", torsodd.data()+4);
  logger.addChannel("QP.torsodd[z]", "m/s2", torsodd.data()+5);
  logger.addChannel("QP.torsodd[wx]", "rad/s2", torsodd.data()+0);
  logger.addChannel("QP.torsodd[wy]", "rad/s2", torsodd.data()+1);
  logger.addChannel("QP.torsodd[wz]", "rad/s2", torsodd.data()+2);

  logger.addChannel("QP_d.footdd[L][x]", "m/s2", footdd_d[Side::LEFT].data()+3);
  logger.addChannel("QP_d.footdd[L][y]", "m/s2", footdd_d[Side::LEFT].data()+4);
  logger.addChannel("QP_d.footdd[L][z]", "m/s2", footdd_d[Side::LEFT].data()+5);
  logger.addChannel("QP_d.footdd[L][wx]", "rad/s2", footdd_d[Side::LEFT].data()+0);
  logger.addChannel("QP_d.footdd[L][wy]", "rad/s2", footdd_d[Side::LEFT].data()+1);
  logger.addChannel("QP_d.footdd[L][wz]", "rad/s2", footdd_d[Side::LEFT].data()+2);
  
  logger.addChannel("QP.footdd[L][x]", "m/s2", footdd[Side::LEFT].data()+3);
  logger.addChannel("QP.footdd[L][y]", "m/s2", footdd[Side::LEFT].data()+4);
  logger.addChannel("QP.footdd[L][z]", "m/s2", footdd[Side::LEFT].data()+5);
  logger.addChannel("QP.footdd[L][wx]", "rad/s2", footdd[Side::LEFT].data()+0);
  logger.addChannel("QP.footdd[L][wy]", "rad/s2", footdd[Side::LEFT].data()+1);
  logger.addChannel("QP.footdd[L][wz]", "rad/s2", footdd[Side::LEFT].data()+2);
  
  logger.addChannel("QP_d.footdd[R][x]", "m/s2", footdd_d[Side::RIGHT].data()+3);
  logger.addChannel("QP_d.footdd[R][y]", "m/s2", footdd_d[Side::RIGHT].data()+4);
  logger.addChannel("QP_d.footdd[R][z]", "m/s2", footdd_d[Side::RIGHT].data()+5);
  logger.addChannel("QP_d.footdd[R][wx]", "rad/s2", footdd_d[Side::RIGHT].data()+0);
  logger.addChannel("QP_d.footdd[R][wy]", "rad/s2", footdd_d[Side::RIGHT].data()+1);
  logger.addChannel("QP_d.footdd[R][wz]", "rad/s2", footdd_d[Side::RIGHT].data()+2);
  
  logger.addChannel("QP.footdd[R][x]", "m/s2", footdd[Side::RIGHT].data()+3);
  logger.addChannel("QP.footdd[R][y]", "m/s2", footdd[Side::RIGHT].data()+4);
  logger.addChannel("QP.footdd[R][z]", "m/s2", footdd[Side::RIGHT].data()+5);
  logger.addChannel("QP.footdd[R][wx]", "rad/s2", footdd[Side::RIGHT].data()+0);
  logger.addChannel("QP.footdd[R][wy]", "rad/s2", footdd[Side::RIGHT].data()+1);
  logger.addChannel("QP.footdd[R][wz]", "rad/s2", footdd[Side::RIGHT].data()+2);

  for (int i = 0; i < qdd.size(); i++) 
    logger.addChannel("QP.qdd["+rs.robot->getPositionName(i)+"]", "rad/s2", qdd.data()+i);
  
  for (int i = 0; i < trq.size(); i++) 
    logger.addChannel("QP.trq["+rs.robot->getPositionName(i)+"]", "Nm", trq.data()+i);

  logger.addChannel("QP.F[L][x]", "N", grf_w[Side::LEFT].data()+3);
  logger.addChannel("QP.F[L][y]", "N", grf_w[Side::LEFT].data()+4);
  logger.addChannel("QP.F[L][z]", "N", grf_w[Side::LEFT].data()+5);
  logger.addChannel("QP.M[L][x]", "Nm", grf_w[Side::LEFT].data()+0);
  logger.addChannel("QP.M[L][y]", "Nm", grf_w[Side::LEFT].data()+1);
  logger.addChannel("QP.M[L][z]", "Nm", grf_w[Side::LEFT].data()+2);
  logger.addChannel("QP.F[R][x]", "N", grf_w[Side::RIGHT].data()+3);
  logger.addChannel("QP.F[R][y]", "N", grf_w[Side::RIGHT].data()+4);
  logger.addChannel("QP.F[R][z]", "N", grf_w[Side::RIGHT].data()+5);
  logger.addChannel("QP.M[R][x]", "Nm", grf_w[Side::RIGHT].data()+0);
  logger.addChannel("QP.M[R][y]", "Nm", grf_w[Side::RIGHT].data()+1);
  logger.addChannel("QP.M[R][z]", "Nm", grf_w[Side::RIGHT].data()+2);
  
  logger.addChannel("QP.cop_b[L][x]", "m/s2", cop_b[Side::LEFT].data()+0);
  logger.addChannel("QP.cop_b[L][y]", "m/s2", cop_b[Side::LEFT].data()+1);
  logger.addChannel("QP.cop_b[R][x]", "m/s2", cop_b[Side::RIGHT].data()+0);
  logger.addChannel("QP.cop_b[R][y]", "m/s2", cop_b[Side::RIGHT].data()+1);
} 
