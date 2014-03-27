#include "dubins-poly.hpp"

using namespace Eigen;
using namespace std;
using namespace eigen_utils;

#define DRAW_DS .01

DubinsPolySegment::DubinsPolySegment(DubinsPathPrimitive * dubins_primitive) :
    poly(Eigen::Vector3d::Zero()) //setup a 3rd order polynomial of 0's by default
{
  this->dubins_primitive = dubins_primitive;
  this->theta_end = this->dubins_primitive->getLength();
  if (this->dubins_primitive->type == 0) {
    this->dubins_line = (DubinsLine *) this->dubins_primitive;
    this->dubins_arc = NULL;
  }
  else {
    this->dubins_arc = (DubinsArc *) this->dubins_primitive;
    this->dubins_line = NULL;
  }
}

DubinsPolySegment::DubinsPolySegment(const dubins_poly_segment_t * msg) :
    poly(&msg->poly)
{
  if (msg->dubins_primitive.type == 0) {
    this->dubins_line = new DubinsLine(&msg->dubins_primitive);
    this->dubins_arc = NULL;
    this->dubins_primitive = this->dubins_line;
  }
  else {
    this->dubins_arc = new DubinsArc(&msg->dubins_primitive);
    this->dubins_line = NULL;
    this->dubins_primitive = this->dubins_arc;
  }
  this->theta_end = this->dubins_primitive->getLength();
}

void DubinsPolySegment::to_dubins_poly_segment_t(dubins_poly_segment_t * msg)
{
  if (this->dubins_line != NULL) {
    this->dubins_line->to_dubins_primitive_t(&msg->dubins_primitive);
  }
  else {
    this->dubins_arc->to_dubins_primitive_t(&msg->dubins_primitive);
  }
  poly.to_planning_polynomial_t(&msg->poly);
}

void DubinsPolySegment::getTransAxialDerivatives(double theta, Eigen::MatrixXd & trans_axial_derivatives)
{
  int D = trans_axial_derivatives.cols();
  assert(D <= 4);

  Eigen::VectorXd Pderiv(D);
  this->poly.eval(theta, Pderiv);

  trans_axial_derivatives(0, 0) = Pderiv(0);

  double P, Pd, Pdd, Pddd;

  int Psign;
  if (this->dubins_primitive->type > 0) {
    Psign = -1;
  }
  else {
    Psign = 1;
  }

  Pderiv *= Psign;

  P = Pderiv(0);

  if (D > 1)
    Pd = Pderiv(1);
  else
    Pd = 0;

  if (D > 2)
    Pdd = Pderiv(2);
  else
    Pdd = 0;

  if (D > 3)
    Pddd = Pderiv(3);
  else
    Pddd = 0;

  double Pd2, Pd3, Pdd2;

  double L;
  double rhod_rhat, rhodd_rhat, rhoddd_rhat, rhod_that, rhodd_that, rhoddd_that;
  double rho_rhat_P, drho_ds_rhat, drho2_ds2_rhat, drho3_ds3_rhat;
  double L2, L3, L4, L5, L7;
  double thetad, thetadd, thetaddd;
  if (this->dubins_primitive->type != 0) {
    double R_0 = fabs(1.0 / this->dubins_primitive->curvature);

    Pd *= R_0;
    Pdd *= R_0 * R_0;
    Pddd *= R_0 * R_0 * R_0;

    Pd2 = Pd * Pd;
    Pd3 = Pd2 * Pd;
    Pdd2 = Pdd * Pdd;

    double R = R_0 + P;
    double R2 = R * R;
    double PddR2 = pow(Pdd + R, 2);
    L = sqrt(R2 + Pd2);
    L2 = pow(L, 2);
    L3 = L2 * L;
    L4 = L3 * L;
    L5 = L4 * L;
    L7 = L2 * L5;

    thetad = 1.0 / L;
    thetadd = -Pd * (Pdd + R) / L4;
    thetaddd = -(L2 * (Pdd * (Pdd + R) + Pd * (Pddd + Pd)) - 4 * Pd2 * PddR2) / L7;

    rhod_rhat = Pd;
    rhodd_rhat = Pdd - R;
    rhoddd_rhat = Pddd - 3 * Pd;

    rhod_that = R;
    rhodd_that = 2 * Pd;
    rhoddd_that = 3 * Pdd - R;

  }
  else {

    Pd2 = Pd * Pd;
    Pd3 = Pd2 * Pd;
    Pdd2 = Pdd * Pdd;

    L = sqrt(1 + Pd2);
    L2 = pow(L, 2);
    L3 = L2 * L;
    L4 = L3 * L;
    L5 = L4 * L;
    L7 = L2 * L5;

    thetad = 1.0 / L;
    thetadd = -Pd * Pdd / L4;
    thetaddd = -(L2 * (Pdd2 + Pd * Pddd) - 4 * Pd2 * Pdd2) / L7;

    rhod_rhat = Pd;
    rhodd_rhat = Pdd;
    rhoddd_rhat = Pddd;

    rhod_that = 1;
    rhodd_that = 0;
    rhoddd_that = 0;

  }

  double thetad2 = thetad * thetad;
  double thetad3 = thetad2 * thetad;

  trans_axial_derivatives(0, 0) = P;
  trans_axial_derivatives(1, 0) = 0;

  if (D > 1) {
    trans_axial_derivatives(0, 1) = rhod_rhat * thetad;
    trans_axial_derivatives(1, 1) = rhod_that * thetad;
    if (D > 2) {
      trans_axial_derivatives(0, 2) = rhodd_rhat * thetad2 + rhod_rhat * thetadd;
      trans_axial_derivatives(1, 2) = rhodd_that * thetad2 + rhod_that * thetadd;
      if (D > 3) {
        trans_axial_derivatives(0, 3) = rhoddd_rhat * thetad3 + 3 * rhodd_rhat * thetadd * thetad
            + rhod_rhat * thetaddd;
        trans_axial_derivatives(1, 3) = rhoddd_that * thetad3 + 3 * rhodd_that * thetadd * thetad
            + rhod_that * thetaddd;
      }
    }
  }

  trans_axial_derivatives.topRows(1) *= Psign;
}

void DubinsPolySegment::getStartTransAxialDerivatives(Eigen::MatrixXd & trans_axial_derivatives)
{
  getTransAxialDerivatives(0, trans_axial_derivatives);
}

void DubinsPolySegment::getEndTransAxialDerivatives(Eigen::MatrixXd & trans_axial_derivatives)
{
  getTransAxialDerivatives(this->theta_end, trans_axial_derivatives);
}

double DubinsPolySegment::getArcLength(double theta)
{

  Eigen::VectorXd P_vec(2);
  this->poly.eval(theta, P_vec);

  double L;
  if (this->dubins_primitive->type == 0) {
    L = sqrt(1 + pow(P_vec(1), 2));
  }
  else {
    double R_0 = this->dubins_arc->R;
    double R = R_0 - this->dubins_arc->ccw * P_vec(0);
    L = sqrt(pow(R / R_0, 2) + pow(P_vec(1), 2));
  }
  return L;
}

void DubinsPolySegment::getPolyDerivatives(const Eigen::VectorXd & signed_trans_derivatives, Eigen::VectorXd & Pderiv)
{

  int D = signed_trans_derivatives.rows();

  int Psign;
  if (this->dubins_primitive->type > 0) {
    Psign = -1;
  }
  else {
    Psign = 1;
  }

  Eigen::VectorXd trans_derivatives = Psign * signed_trans_derivatives;

  assert(D <= 4);

  double rho = trans_derivatives(0);
  double drho_ds, drho2_ds2, drho3_ds3;

  if (D > 1)
    drho_ds = trans_derivatives(1);
  else
    drho_ds = 0;

  double drho_ds2 = drho_ds * drho_ds;

  if (D > 2)
    drho2_ds2 = trans_derivatives(2);
  else
    drho2_ds2 = 0;

  if (D > 3)
    drho3_ds3 = trans_derivatives(3);
  else
    drho3_ds3 = 0;

  double P, Pd, Pdd, Pddd;
  if (this->dubins_primitive->type != 0) {
    double R_0 = fabs(1.0 / this->dubins_primitive->curvature);
    double R = R_0 + rho;
    double R2 = R * R;
    P = rho;

    Pd = drho_ds * R / sqrt(1 - drho_ds2);
    double Pd2 = Pd * Pd;
    double Pd3 = Pd * Pd2;
    double L = sqrt(R2 + Pd2);
    double L2 = pow(L, 2);
    double L3 = L2 * L;
    double L4 = L3 * L;
    double L5 = L4 * L;
    double L7 = L2 * L5;

    Pdd = (drho2_ds2 + R / L2 + R * Pd2 / L4) / (1 / L2 - Pd2 / L4);

    double Pdd2 = Pdd * Pdd;
    double PddR2 = pow(Pdd + R, 2);

    Pddd = (drho3_ds3
        - (-3 * Pd / L3 - 4 * Pd * Pdd2 / L5 + 3 * Pd * R2 / L5 - Pd * Pdd * R / L5 - Pd3 / L5 + 4 * Pd3 * PddR2 / L7))
        / (1 / L3 - Pd2 / L5);

    Pd /= R_0;
    Pdd /= R_0 * R_0;
    Pddd /= R_0 * R_0 * R_0;
  }
  else {
    P = rho;

    Pd = drho_ds / sqrt(1 - drho_ds2);
    double Pd2 = Pd * Pd;
    double L = sqrt(1.0 + Pd2);
    double L2 = pow(L, 2);
    double L3 = L2 * L;
    double L4 = L3 * L;
    double L5 = L4 * L;
    double L7 = L2 * L5;
    double Pd3 = Pd * Pd2;

    Pdd = drho2_ds2 / (1 / L2 - Pd2 / L4);
    double Pdd2 = Pdd * Pdd;

    Pddd = (drho3_ds3 - (-4 * Pd * Pdd2 / L5 + 4 * Pd3 * Pdd2 / L7)) / (1 / L3 - Pd2 / L5);
  }

  Pderiv(0) = P;
  if (D > 1) {
    Pderiv(1) = Pd;
    if (D > 2) {
      Pderiv(2) = Pdd;
      if (D > 3) {
        Pderiv(3) = Pddd;
      }
    }
  }
  Pderiv *= Psign;

}

Eigen::Vector2d DubinsPolySegment::getAxialUnitVec(double theta)
{
  if (this->dubins_primitive->type == 0) {
    return this->dubins_line->unit_vec;
  }
  else {
    double heading = this->dubins_arc->ccw * (theta / this->dubins_arc->R + M_PI / 2.0) + this->dubins_arc->start_angle;
    return eigen_utils::angleToVec(heading);
  }
}

/**
 * transverse unit vector to left of nominal dubins path
 */
Eigen::Vector2d DubinsPolySegment::getTransUnitVec(double theta)
{
  if (this->dubins_primitive->type == 0) {
    return this->dubins_line->left_unit_vec;
  }
  else {
    double total_theta = this->dubins_arc->ccw * theta / this->dubins_arc->R + this->dubins_arc->start_angle;
    return -this->dubins_arc->ccw * eigen_utils::angleToVec(total_theta);
  }
}

/**
 * unit vectors composed into rotation matrix
 */
Eigen::Matrix2d DubinsPolySegment::getTransAxialRotationMatrix(double theta)
{
  Eigen::Matrix2d rot_mat;
  rot_mat.col(0) = this->getTransUnitVec(theta);
  rot_mat.col(1) = this->getAxialUnitVec(theta);
  return rot_mat;
}

/**
 * origin of trans axial frame in xy coordinates
 */
Eigen::Vector2d DubinsPolySegment::getTransAxialOrigin(double theta)
{
  if (this->dubins_primitive->type == 0) {
    return this->dubins_line->unit_vec * theta + this->dubins_line->start_pt;
  }
  else {
    return this->dubins_arc->R * (-this->dubins_arc->ccw * this->getTransUnitVec(theta)) + this->dubins_arc->center;
  }
}

DubinsPolySegment::~DubinsPolySegment()
{
  delete this->dubins_primitive;
}

std::ostream& operator<<(std::ostream& output, const DubinsPolySegment & seg)
{
  output << "DubinsPolySegment:\n";
  if (seg.dubins_arc != NULL) {
    output << *seg.dubins_arc << std::endl;
  }
  else if (seg.dubins_line != NULL) {
    output << *seg.dubins_line << std::endl;
  }

  output << "theta_end: " << seg.theta_end << std::endl;
  output << "polynomial coefficients:\n" << seg.poly << std::endl;
  output << "\n\n";
  return output;
}

double dubinsPolyGetPath(const std::list<DubinsPathPrimitive *> & dubins_path, const Eigen::VectorXd & der_costs,
    const Eigen::VectorXd & start_trans_derivatives, const Eigen::VectorXd & end_trans_derivatives, int D,
    std::list<DubinsPolySegment *> & path)
{
  int K = dubins_path.size();
  list<DubinsPathPrimitive *>::const_iterator dubins_path_it;
  list<DubinsPolySegment *>::iterator dubins_poly_path_start; //corresponds to the first element in the dubins path, thus where our optimization starts

  VectorXd curvatures(K);
  VectorXd taus(K);
  MatrixXd der_offsets;
  der_offsets.setZero(D, K - 1);

  VectorXd start_Pder(start_trans_derivatives.rows());
  VectorXd end_Pder(end_trans_derivatives.rows());

  Polynomial ** polys = (Polynomial **) calloc(K, sizeof(Polynomial *));

  int kk = 0;
  for (dubins_path_it = dubins_path.begin(); dubins_path_it != dubins_path.end(); dubins_path_it++) {
    DubinsPathPrimitive * dubins_prim = *dubins_path_it;

    DubinsPolySegment * poly_segment = new DubinsPolySegment(dubins_prim);
    path.push_back(poly_segment);
    polys[kk] = &(poly_segment->poly);

    if (kk == 0) {
      dubins_poly_path_start = path.end();
      dubins_poly_path_start--;
    }

    if (kk == 0) {
      poly_segment->getPolyDerivatives(start_trans_derivatives, start_Pder);
    }
    else if (kk == K - 1) {
      poly_segment->getPolyDerivatives(end_trans_derivatives, end_Pder);
    }

    curvatures(kk) = dubins_prim->curvature;

    taus(kk) = dubins_prim->getLength();

    kk++;
  }

  if (D > 2) {
    for (kk = 0; kk < (K - 1); kk++) {
      der_offsets(2, kk) = -curvatures(kk + 1) + curvatures(kk);
    }
  }
  else {
    return 0;
  }

  double cost;

//  eigen_dump(taus);
//  eigen_dump(start_Pder);
//  eigen_dump(end_Pder);
//  eigen_dump(der_costs);
//  eigen_dump(der_offsets);

  polyQuadDerOptPiecewise(taus, start_Pder, end_Pder, der_costs, der_offsets, polys, &cost);

  bool re_opt = true;
  if (re_opt) {
    Eigen::MatrixXd trans_axial_derivatives(2, D); //trans_axial derivatives for a segment (beginning or end, just used as argument)
    Eigen::MatrixXd segment_start_trans_derivatives(D, K); //matrix where each column is the trans derivative at the beginning of the segment
    Eigen::MatrixXd segment_end_trans_derivatives(D, K); //matrix where each column is the trand derivative at the end of the segment

    list<DubinsPolySegment *>::const_iterator dubins_poly_path_it;

    kk = 0;
    for (dubins_poly_path_it = dubins_poly_path_start; dubins_poly_path_it != path.end(); dubins_poly_path_it++) {
      DubinsPolySegment * dubins_poly_segment = *dubins_poly_path_it;
//      eigen_dump(*polys[kk]);
      dubins_poly_segment->getStartTransAxialDerivatives(trans_axial_derivatives);
      segment_start_trans_derivatives.col(kk) = trans_axial_derivatives.row(0).transpose();

      dubins_poly_segment->getEndTransAxialDerivatives(trans_axial_derivatives);
      segment_end_trans_derivatives.col(kk) = trans_axial_derivatives.row(0).transpose();
      kk++;
    }

    Eigen::MatrixXd drho_dtheta_joints = Eigen::MatrixXd::Zero(D, K - 1);
    for (kk = 0; kk < K - 1; kk++) {
      drho_dtheta_joints.col(kk) = (segment_start_trans_derivatives.col(kk + 1) + segment_end_trans_derivatives.col(kk))
          / 2;
    }

//    eigen_dump(segment_start_trans_derivatives);
//    eigen_dump(segment_end_trans_derivatives);
//    eigen_dump(drho_dtheta_joints);

    VectorXd der_costs_scaled;
    VectorXd j_single(K);

    VectorXd der_0, der_final;

    kk = 0;
    for (dubins_poly_path_it = dubins_poly_path_start; dubins_poly_path_it != path.end(); dubins_poly_path_it++) {
      DubinsPolySegment * dubins_poly_segment = *dubins_poly_path_it;

      if (kk > 0) {
        der_0.resize(D);
        dubins_poly_segment->getPolyDerivatives(drho_dtheta_joints.col(kk - 1), der_0);
      }
      else {
        der_0.resize(start_trans_derivatives.rows());
        dubins_poly_segment->getPolyDerivatives(start_trans_derivatives, der_0);
      }

      if (kk < K - 1) {
        der_final.resize(D);
        dubins_poly_segment->getPolyDerivatives(drho_dtheta_joints.col(kk), der_final);
      }
      else {
        der_final.resize(end_trans_derivatives.rows());
        dubins_poly_segment->getPolyDerivatives(end_trans_derivatives, der_final);
      }

      dubins_poly_segment->poly = polyQuadDerOpt(taus(kk), der_0, der_final, der_costs, &j_single(kk));
//      eigen_dump(taus(kk));
//      eigen_dump(der_0);
//      eigen_dump(der_final);
//      eigen_dump(dubins_poly_segment->poly);
      kk++;
    }

    cost = j_single.sum();
  }

  free(polys);
  return cost;

}

double dubinsPolyGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, const Eigen::VectorXd & der_costs, const Eigen::VectorXd & start_trans_derivatives,
    const Eigen::VectorXd & end_trans_derivatives, int D, std::list<DubinsPolySegment *> & path)
{
  list<DubinsPathPrimitive *> dubins_path = list<DubinsPathPrimitive *>();
  dubinsGetPath(R, xy_start, theta_start, xy_end, theta_end, dubins_path);

  return dubinsPolyGetPath(dubins_path, der_costs, start_trans_derivatives, end_trans_derivatives, D, path);
}

double dubinsPolyGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, const Eigen::VectorXd & der_costs, int D, std::list<DubinsPolySegment *> & path)
{
  Eigen::VectorXd start_trans_derivatives = Eigen::VectorXd::Zero(D);
  Eigen::VectorXd end_trans_derivatives = Eigen::VectorXd::Zero(D);

  return dubinsPolyGetPath(R, xy_start, theta_start, xy_end, theta_end, der_costs, start_trans_derivatives,
      end_trans_derivatives, D, path);
}

//------------- Utility Functions for DubinsPoly Paths ---------------------
double dubinsPolyGetPathLength(const std::list<DubinsPolySegment *> & path)
{
  double length = 0;
  std::list<DubinsPolySegment *>::const_iterator path_it = path.begin();
  DubinsPolySegment * cur_prim;
  while (!path.empty()) {
    cur_prim = *path_it;
    length += cur_prim->dubins_primitive->getLength();
  }
}

void dubinsPolyFreePath(std::list<DubinsPolySegment *> & path)
{
  std::list<DubinsPolySegment *>::iterator path_it = path.begin();
  DubinsPolySegment * cur_prim;
  while (!path.empty()) {
    cur_prim = *path_it;
    delete cur_prim;
    path_it = path.erase(path_it);
  }
}

dubins_poly_segment_list_t * to_dubins_poly_segment_list(const std::list<DubinsPolySegment *> & path)
{
  dubins_poly_segment_list_t * msg = (dubins_poly_segment_list_t *) calloc(1, sizeof(dubins_poly_segment_list_t));
  msg->num_segments = path.size();
  msg->path_list = (dubins_poly_segment_t *) calloc(msg->num_segments, sizeof(dubins_poly_segment_t));

  std::list<DubinsPolySegment *>::const_iterator path_it;
  int ii = 0;
  for (path_it = path.begin(); path_it != path.end(); path_it++) {
    (*path_it)->to_dubins_poly_segment_t(&msg->path_list[ii]);
    ii++;
  }
  return msg;
}

void toDubinsPolyPath(const dubins_poly_segment_list_t * msg, std::list<DubinsPolySegment *> & path)
{
  for (int ii = 0; ii < msg->num_segments; ii++) {
    path.push_back(new DubinsPolySegment(&msg->path_list[ii]));
  }
}

DubinsPolyPath::DubinsPolyPath(std::list<DubinsPolySegment *> & path, bool loop_path_, bool path_owner_)
{
  this->path = &path;
  this->loop_path = loop_path_;
  this->path_owner = path_owner_;

  if (this->path_owner)
    this->path_nonconst = &path;

  this->reset(); //should setup the iterators and all

}

DubinsPolyPath::DubinsPolyPath(const std::list<DubinsPolySegment *> & path, bool loop_path_arg)
{
  this->path = &path;
  this->loop_path = loop_path_arg;
  this->path_owner = false;
  this->path_nonconst = NULL
  ;

  this->reset(); //should setup the iterators and all
}

DubinsPolyPath::DubinsPolyPath(const dubins_poly_segment_list_t * msg)
{
  DubinsPolyPathList * path_list = new DubinsPolyPathList();
  toDubinsPolyPath(msg, *path_list);
  this->path = path_list;
  this->loop_path = msg->loop_path;

  this->path_owner = true;
  if (this->path_owner)
    this->path_nonconst = path_list;

  this->reset(); //should setup the iterators and all
}

DubinsPolyPath::DubinsPolyPath(const DubinsPolyPath & other)
{
  this->path = other.path;
  this->loop_path = other.loop_path;
  this->path_owner = false;
  this->path_nonconst = NULL;
}

DubinsPolyPath::~DubinsPolyPath()
{
  if (this->path_owner) {
    dubinsPolyFreePath(*this->path_nonconst);
    delete this->path_nonconst;
  }
}

/**
 * get heading along current point
 */
double DubinsPolyPath::getHeading() const
{
  return eigen_utils::atan2Vec(this->cur_seg->getAxialUnitVec(this->theta));
}

void DubinsPolyPath::spliceAtBeginning(std::list<DubinsPolySegment *> & other_list)
{
  if (!this->path_owner) {
    fprintf(stderr, "error: can't splice dubins poly path if not owner\n");
    return;
  }
  this->path_nonconst->splice(this->path_nonconst->begin(), other_list);
  this->path = this->path_nonconst;
}
void DubinsPolyPath::spliceAtEnd(std::list<DubinsPolySegment *> & other_list)
{
  if (!this->path_owner) {
    fprintf(stderr, "error: can't splice dubins poly path if not owner\n");
    return;
  }
  this->path_nonconst->splice(this->path_nonconst->end(), other_list);
  this->path = this->path_nonconst;
}

dubins_poly_segment_list_t * DubinsPolyPath::to_dubins_poly_segment_list_t() const
{
  dubins_poly_segment_list_t * msg = to_dubins_poly_segment_list(*this->path); //kind of ugly copy
  msg->loop_path = this->loop_path;
  return msg;
}

bool DubinsPolyPath::empty()
{
  return this->path->empty();
}

bool DubinsPolyPath::step(double ds)
{
  bool ended = false;

  if (this->it == path->end()) {
    if (this->loop_path) {
      this->it = path->begin();
      this->cur_seg = *this->it;
    }
    else {
      ended = true;
      this->it--;
      this->cur_seg = *this->it;
      ds = 0;
      this->theta = cur_seg->dubins_primitive->getLength();
      return ended;
    }
  }

  this->cur_seg = *this->it;
  double theta_max = cur_seg->dubins_primitive->getLength();

  double L = cur_seg->getArcLength(this->theta);

  //emergency stop
  if (isnan(L)) {
    fprintf(stderr, "error: nan encountered in DubinsPolyPath::step while evaluating arc length");
    eigen_dump(*cur_seg);
    eigen_dump(theta);
    return false;
  }

  double dtheta = ds / L;
  this->theta += dtheta;

  if (this->theta > theta_max) {
    double ds_remainder = (this->theta - theta_max) * L;
    this->it++;
    this->theta = 0;
    return this->step(ds_remainder);
  }

  return ended;
}

double DubinsPolyPath::transOffset() const
{
  return cur_seg->poly.eval(theta);
}

bool DubinsPolyPath::isValidPoint() const
{
  if (cur_seg->dubins_primitive->type != 0) {
    double R_0 = cur_seg->dubins_arc->R - cur_seg->dubins_arc->ccw * transOffset();
    if (R_0 < 0)
      return false;
  }

  return true;
}

void DubinsPolyPath::reset()
{
  this->it = this->path->begin();
  if (this->path->empty()) {
    this->cur_seg = NULL;
  }
  else {
    this->cur_seg = *this->it;
  }
  this->theta = 0;
}

void DubinsPolyPath::goToEnd()
{
  this->it = this->path->end();
  this->it--;
  this->cur_seg = *this->it;
  this->theta = this->cur_seg->theta_end;
}

Eigen::Vector2d DubinsPolyPath::getPoint() const
{
  Eigen::Vector2d cur_point = cur_seg->getTransAxialOrigin(this->theta)
      + cur_seg->getTransUnitVec(this->theta) * cur_seg->poly.eval(this->theta);
  return cur_point;
}

void DubinsPolyPath::getDerivatives(Eigen::MatrixXd & derivatives) const
    {
  Eigen::MatrixXd trans_axial_derivatives;
  trans_axial_derivatives.resizeLike(derivatives);
  this->cur_seg->getTransAxialDerivatives(this->theta, trans_axial_derivatives);
  derivatives = this->cur_seg->getTransAxialRotationMatrix(this->theta) * trans_axial_derivatives;
  derivatives.col(0) += this->cur_seg->getTransAxialOrigin(this->theta);
}

double DubinsPolyPath::getCurvature() const
{
  Eigen::MatrixXd trans_axial_derivatives(2, 3);
  this->cur_seg->getTransAxialDerivatives(this->theta, trans_axial_derivatives);

  //from: http://mathworld.wolfram.com/Curvature.html
  // where x_hat = axial_hat, y_hat = trans_hat (still right handed coordinate system)
  double transd = trans_axial_derivatives(0, 1);
  double transdd = trans_axial_derivatives(0, 2);

  double axiald = trans_axial_derivatives(1, 1);
  double axialdd = trans_axial_derivatives(1, 2);

  double curvature = axiald * transdd - transd * axialdd;
  return curvature;
}

void DubinsPolyPath::getCurvatureDerivatives(double * curvature, double * dcurvature) const
    {
  Eigen::MatrixXd trans_axial_derivatives(2, 4);
  this->cur_seg->getTransAxialDerivatives(this->theta, trans_axial_derivatives);

  //from: http://mathworld.wolfram.com/Curvature.html
  // where x_hat = axial_hat, y_hat = trans_hat (still right handed coordinate system)
  double transd = trans_axial_derivatives(0, 1);
  double transdd = trans_axial_derivatives(0, 2);
  double transddd = trans_axial_derivatives(0, 3);

  double axiald = trans_axial_derivatives(1, 1);
  double axialdd = trans_axial_derivatives(1, 2);
  double axialddd = trans_axial_derivatives(1, 3);

  *curvature = axiald * transdd - transd * axialdd;
  *dcurvature = axialdd * transdd + axiald * transddd - transdd * axialdd - transd * axialddd;
}

double DubinsPolyPath::getTheta()
{
  return theta; //parameter state for the current segment
}

DubinsPolySegment * DubinsPolyPath::getCurSeg()
{
  return cur_seg;
}

void DubinsPolyPath::lcmgl_print(bot_lcmgl_t * lcmgl, double ds, double z) const
    {
  DubinsPolyPath print_path = DubinsPolyPath(*this->path, false);
  bool ended = print_path.empty();

  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  Eigen::Vector2d cur_point;
  while (!ended) {
    cur_point = print_path.getPoint();
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), z);
    ended = print_path.step(ds);
  }

  bot_lcmgl_end(lcmgl);
}

void DubinsPolyPath::lcmgl_print_up_to_cur(bot_lcmgl_t * lcmgl, double ds, double z) const
    {
  DubinsPolyPath print_path = DubinsPolyPath(*this->path, false);
  bool ended = print_path.empty();

  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  Eigen::Vector2d cur_point;
  while (!ended) {
    if (print_path.cur_seg == this->cur_seg && print_path.theta > this->theta)
      break;

    cur_point = print_path.getPoint();
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), z);
    ended = print_path.step(ds);
  }

  bot_lcmgl_end(lcmgl);
}

void DubinsPolyPath::lcmgl_print_dubins(bot_lcmgl_t * lcmgl, double ds, double z) const
    {
  DubinsPolyPath print_path = DubinsPolyPath(*this->path, false);
  bool ended = print_path.empty();

  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  Eigen::Vector2d cur_point;
  while (!ended) {
    cur_point = print_path.cur_seg->getTransAxialOrigin(print_path.theta);
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), z);
    ended = print_path.step(ds);
  }

  bot_lcmgl_end(lcmgl);
}

void DubinsPolyPath::lcmgl_print_derivative_vec(bot_lcmgl_t * lcmgl, int der, double scale, double ds) const
    {
  DubinsPolyPath print_path = DubinsPolyPath(*this->path, false);
  bool ended = print_path.empty();

  bot_lcmgl_begin(lcmgl, GL_LINES);
  Eigen::Vector2d cur_point;
  Eigen::Vector2d der_point;
  Eigen::MatrixXd derivatives(2, der + 1);

  while (!ended) {

    print_path.getDerivatives(derivatives);
    cur_point = derivatives.col(0);
    der_point = derivatives.col(der) * scale + cur_point;
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), 0);
    bot_lcmgl_vertex3d(lcmgl, der_point(0), der_point(1), 0);
    ended = print_path.step(ds);
  }

  bot_lcmgl_end(lcmgl);
}

void DubinsPolyPath::lcmgl_print_trans_axial_coords(bot_lcmgl_t * lcmgl, double scale, double ds) const
    {
  DubinsPolyPath print_path = DubinsPolyPath(*this->path, false);
  bool ended = print_path.empty();

  bot_lcmgl_begin(lcmgl, GL_LINES);
  Eigen::Vector2d cur_point;
  Eigen::Vector2d trans_point;
  Eigen::Vector2d axial_point;

  while (!ended) {
    cur_point = print_path.cur_seg->getTransAxialOrigin(print_path.theta);
    trans_point = print_path.cur_seg->getTransUnitVec(print_path.theta) * scale + cur_point;
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), 0);
    bot_lcmgl_vertex3d(lcmgl, trans_point(0), trans_point(1), 0);

    axial_point = print_path.cur_seg->getAxialUnitVec(print_path.theta) * scale + cur_point;
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), 0);
    bot_lcmgl_vertex3d(lcmgl, axial_point(0), axial_point(1), 0);

    ended = print_path.step(ds);
  }

  bot_lcmgl_end(lcmgl);
}

