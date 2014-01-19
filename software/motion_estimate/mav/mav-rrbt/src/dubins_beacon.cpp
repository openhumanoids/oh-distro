#include "dubins_beacon.hpp"

using namespace eigen_utils;

DubinsBeacon::DubinsBeacon(X x_init, A aug_init, RegionCSpace<2> * c_space, Region<2> * goal_region, double nsigma) :
    beacons()
{
  this->x_init = x_init;
  this->aug_init = aug_init;
  this->c_space = c_space;
  this->goal_region = goal_region;

  this->r_min = 2;
  this->v_0 = 1;
  this->omega_0 = this->v_0 / this->r_min;
  this->length = 1;
  this->dt = .1;
  this->dl = this->dt * this->v_0;
  this->nsigma = nsigma;

  this->K = Matrix23d::Zero();
  double Kxv = 1;
  double Kyomega = this->v_0 * 1;
  double Kthetaomega = 1;
  this->K(0, 0) = Kxv;
  this->K(1, 1) = Kyomega;
  this->K(1, 2) = Kthetaomega;

  this->Mcbase = Eigen::Matrix2d::Zero();
  this->Mcbase(0, 0) = .001;
  this->Mcbase(1, 1) = .001;
  this->Mcbase /= dt;

  this->Rc_beacon_base = Eigen::Matrix2d::Zero();
  this->Rc_beacon_base(0, 0) = .005;
  this->Rc_beacon_base(1, 1) = .005;
  this->Rc_beacon_base /= dt;

  this->prop_demo = true;

  if (this->prop_demo) {
    this->K *= .1;
  }

  //  this->prop_demo *= .5;

}

DubinsBeacon::~DubinsBeacon()
{
  //    freeList(lqg_regions);
}

void DubinsBeacon::addSquareWithBeaconCorners(Box * box, double beacon_radius)
{
  this->c_space->obstacles.push_back(box);
  Circle * ll = new Circle(box->minCorner, beacon_radius);
  Circle * ur = new Circle(box->maxCorner, beacon_radius);
  Eigen::Vector2d lr_center, ul_center;
  lr_center << box->minCorner(0), box->maxCorner(1);
  ul_center << box->maxCorner(0), box->minCorner(1);
  Circle * lr = new Circle(lr_center, beacon_radius);
  Circle * ul = new Circle(ul_center, beacon_radius);
  this->beacons.push_back(ll);
  this->beacons.push_back(ur);
  this->beacons.push_back(lr);
  this->beacons.push_back(ul);
}

void DubinsBeacon::getBeaconsML(const X & x, std::list<Circle *> & beacons_ml)
{
  std::list<Circle *>::iterator beacons_it;
  Eigen::Vector2d xy = x.block<2, 1>(0, 0);

  Circle * beacon;

  for (beacons_it = this->beacons.begin(); beacons_it != this->beacons.end(); beacons_it++) {
    beacon = *beacons_it;
    if (beacon->contains(xy)) {
      beacons_ml.push_back(beacon);
    }
  }
}

void DubinsBeacon::getBeaconsCC(const X & x, const StateCov & sigma_total, std::list<Circle *> & beacons_cc)
{
  std::list<Circle *>::iterator beacons_it;
  Eigen::Vector2d xy = x.block<2, 1>(0, 0);
  Eigen::Matrix2d xy_sigma = sigma_total.block<2, 2>(0, 0);

  Circle * beacon;

  for (beacons_it = this->beacons.begin(); beacons_it != this->beacons.end(); beacons_it++) {
    beacon = *beacons_it;
    if (beacon->contains_cc(xy, xy_sigma, this->nsigma)) {
      beacons_cc.push_back(beacon);
    }
  }
}

void DubinsBeacon::getAB(const X & x, const U & u, StateCov & A, Matrix32d & B)
{
  double v = u(0);
  double omega = u(1);

  double xs = x(0);
  double ys = x(1);
  double thetas = x(2);

  double dt2 = pow(dt, 2);
  double omega2 = pow(omega, 2);

  if (fabs(omega) < .001) {
    A = StateCov::Identity();
    A(0, 2) = -v * sin(thetas) * dt;
    A(1, 2) = v * cos(thetas) * dt;
    B << cos(thetas) * dt, -1 / 2 * v * dt2 * sin(thetas), sin(thetas) * dt, 1 / 2 * v * dt2 * cos(thetas), 0, dt;
  }
  else {
    A << 1, 0, v / omega * (-cos(thetas) + cos(thetas + omega * dt)), 0, 1, v / omega * (-sin(thetas) + sin(thetas
        + omega * dt)), 0, 0, 1;
    B << (-sin(thetas) + sin(thetas + omega * dt)) / omega, v * (sin(thetas) - sin(thetas + omega * dt)) / omega2 + v
        * cos(thetas + omega * dt) * dt / omega, (cos(thetas) - cos(thetas + omega * dt)) / omega, -v * (cos(thetas)
        - cos(thetas + omega * dt)) / omega2 + v * sin(thetas + omega * dt) * dt / omega, 0, dt;
  }
}

void DubinsBeacon::step(const X & x, const U & u, StateCov & Sigma, StateCov & Lambda)
{

  StateCov A;
  Matrix32d B;

  getAB(x, u, A, B);

  //Kalman Prediction
  StateCov Sigma_bar = A * Sigma * A.transpose() + B * this->Mcbase * B.transpose();

  StateCov Global_to_Body = StateCov::Identity();
  Eigen::Rotation2Dd R_g_to_b = Eigen::Rotation2Dd(-x(2));
  Global_to_Body.block<2, 2>(0, 0) = R_g_to_b.matrix();

  StateCov Lambda_bar = (A - B * K * Global_to_Body) * Lambda * (A - B * K * Global_to_Body).transpose();

  std::list<Circle *> beacons_sensed = std::list<Circle *>();
  StateCov sigma_total = Sigma + Lambda;

  if (this->prop_demo) {
    getBeaconsCC(x, sigma_total, beacons_sensed);
  }
  else {
    getBeaconsCC(x, sigma_total, beacons_sensed);
  }

  std::list<Circle *>::iterator beac_it;
  Circle * beac;
  Eigen::Vector2d c2c;
  Matrix23d C;
  Eigen::Matrix2d S;
  Matrix32d L;
  double r, r2;
  StateCov CovDelta;
  for (beac_it = beacons_sensed.begin(); beac_it != beacons_sensed.end(); beac_it++) {
    beac = *beac_it;
    c2c = beac->center - x.block<2, 1>(0, 0);
    r = c2c.norm();
    r2 = pow(r, 2);
    C << -c2c(0) / r, -c2c(1) / r, 0, c2c(1) / r2, -c2c(0) / r2, -1;
    S = C * Sigma_bar * C.transpose() + Rc_beacon_base;
    L = Sigma_bar * C.transpose() * S.inverse();
    CovDelta = L * C * Sigma_bar;
    Sigma_bar -= CovDelta;
    Lambda_bar += CovDelta;
  }

  Sigma = Sigma_bar;
  Lambda = Lambda_bar;

  if (this->prop_demo) {
    Lambda = StateCov::Zero();
  }
}

bool DubinsBeacon::steerApprox(const X & initial_state, X & final_state, std::list<P *> & path_data,
    bot_lcmgl_t * lcmgl, int animate_usleep)
{
  return steer(initial_state, final_state, path_data, lcmgl, animate_usleep);
}

bool DubinsBeacon::steer(const X & initial_state, const X & final_state, std::list<P *> & path_data,
    bot_lcmgl_t * lcmgl, int animate_usleep)
{
  Eigen::Vector2d xy_cur;

  dubinsGetPath(this->r_min, initial_state, final_state, path_data);

  //  if (final_state(0) < -4.0 && final_state(0) > -6.0 && final_state(1) > 20) {
  //    if (initial_state(0) < -2.0 && initial_state(0) > -4.0 && initial_state(1) > 14 && initial_state(1) < 16) {
  //      std::list<DubinsPathPrimitive *>::iterator path_it;
  //      eigen_dump(initial_state);
  //      eigen_dump(final_state);
  //      for (path_it = path_data.begin(); path_it != path_data.end(); path_it++) {
  //        DubinsPathPrimitive * prim_path = *path_it;
  //        int type = prim_path->type;
  //        if (type == 0) {
  //          DubinsLine * line_path = (DubinsLine *) (*path_it);
  //          std::cout << *line_path << std::endl;
  //        }
  //        else {
  //          DubinsArc * arc_path = (DubinsArc *) (*path_it);
  //          std::cout << *arc_path << std::endl;
  //        }
  //      }
  //    }
  //  }

  bool collision;

  X x_carrot = path_data.front()->getStartPose();
  U u_step;
  std::list<P *>::const_iterator path_it = path_data.begin();
  int step_ccw;
  while (dubinsAdvanceCarrot(path_data, path_it, x_carrot, &step_ccw, this->dl)) {
    u_step << v_0, step_ccw * this->omega_0;
    xy_cur = x_carrot.block<2, 1>(0, 0);
    collision = this->c_space->collides(xy_cur);

    if (lcmgl != NULL) {
      std::list<P *> temp_path_data = std::list<P *>();
      dubinsGetPath(this->r_min, initial_state, x_carrot, temp_path_data);
      this->lcmgl_drawSteer(x_carrot, temp_path_data, collision, lcmgl, animate_usleep);
      dubinsFreePath(temp_path_data);
    }
    if (collision) {
      dubinsFreePath(path_data);
      return false;
    }
  }

  return true;
}

bool DubinsBeacon::propagate(const std::list<P *> & path_data, const A & aug_init, A & aug_final, double * cost,
    bot_lcmgl_t * lcmgl, int animate_usleep)
{
  Eigen::Vector2d xy_cur;
  *cost = dubinsGetPathLength(path_data);

  aug_final = aug_init;

  //turn it into rrt* by skipping propagation
  if (false) {
    aug_final.cost += *cost;
    return true;
  }

  Eigen::Matrix2d Sigma_total2d;

  StateCov Sigma_total;
  bool collision;

  X x_carrot = path_data.front()->getStartPose();
  U u_step;
  std::list<P *>::const_iterator path_it = path_data.begin();
  int step_ccw;
  while (dubinsAdvanceCarrot(path_data, path_it, x_carrot, &step_ccw, this->dl)) {
    u_step << v_0, step_ccw * this->omega_0;
    step(x_carrot, u_step, aug_final.Sigma, aug_final.Lambda);
    Sigma_total = aug_final.Sigma + aug_final.Lambda;

    Sigma_total2d = Sigma_total.block<2, 2>(0, 0);
    xy_cur = x_carrot.block<2, 1>(0, 0);
    collision = this->c_space->collides_cc(xy_cur, Sigma_total2d, this->nsigma);

    if (lcmgl != NULL) {
      this->lcmgl_drawPropagate(x_carrot, aug_final, path_data, collision, lcmgl, animate_usleep);
    }
    if (collision) {
      return false;
    }
  }

  aug_final.cost += *cost;
  aug_final.trace = (aug_final.Sigma + aug_final.Lambda).trace();

  return true;

}

bool DubinsBeacon::simulate(const std::list<P *> & path_data, bot_lcmgl_t * lcmgl)
{

  bool animate_lcmgl = false;

  if (lcmgl != NULL) {
    initDepthTest(lcmgl);
    if (!animate_lcmgl) {
      bot_lcmgl_color3f(lcmgl, BT_SIMULATE_COLOR);
      bot_lcmgl_line_width(lcmgl, BT_SIMULATE_WIDTH);
      bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    }
    else {
      bot_lcmgl_line_width(lcmgl, 5);
      bot_lcmgl_point_size(lcmgl, 10);
    }
  }

  X x_hat = x_init + randn(this->aug_init.Lambda);
  X x = x_hat + randn(this->aug_init.Sigma);
  StateCov Sigma = this->aug_init.Sigma;
  StateCov L, S;
  StateCov I = StateCov::Identity();

  std::list<P *>::const_iterator path_it = path_data.begin();

  U u_step, u_act, u_control;
  u_control = U::Zero();
  X z;

  bool collision;

  bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_SAMPLE_TRAJ);

  StateCov A;
  Matrix32d B;

  X x_carrot = path_data.front()->getStartPose();
  int step_ccw;
  while (dubinsAdvanceCarrot(path_data, path_it, x_carrot, &step_ccw, this->dl)) {
    u_step << this->v_0, step_ccw * this->omega_0;
    u_step += u_control;
    u_act = u_step + randn(this->Mcbase);
    getAB(x_hat, u_step, A, B);

    if (fabs(u_step(1)) < .0001) {
      x_hat(0) += u_step(0) * cos(x_hat(2)) * this->dt;
      x_hat(1) += u_step(0) * sin(x_hat(2)) * this->dt;
      x_hat(2) += u_step(1) * this->dt;
    }
    else {
      x_hat(0) += u_step(0) / u_step(1) * (-sin(x_hat(2)) + sin(x_hat(2) + u_step(1) * this->dt));
      x_hat(1) += u_step(0) / u_step(1) * (cos(x_hat(2)) - cos(x_hat(2) + u_step(1) * this->dt));
      x_hat(2) += u_step(1) * this->dt;

    }

    if (fabs(u_act(1)) < .0001) {
      x(0) += u_act(0) * cos(x(2)) * this->dt;
      x(1) += u_act(0) * sin(x(2)) * this->dt;
      x(2) += u_act(1) * this->dt;
    }
    else {
      x(0) += u_act(0) / u_act(1) * (-sin(x(2)) + sin(x(2) + u_act(1) * this->dt));
      x(1) += u_act(0) / u_act(1) * (cos(x(2)) - cos(x(2) + u_act(1) * this->dt));
      x(2) += u_act(1) * this->dt;
    }

    //Kalman Prediction
    StateCov Sigma_bar = A * Sigma * A.transpose() + B * this->Mcbase * B.transpose();

    std::list<Circle *> beacons_sensed = std::list<Circle *>();
    getBeaconsML(x, beacons_sensed);

    std::list<Circle *>::iterator beac_it;
    Circle * beac;
    Eigen::Vector2d c2c_hat, c2c;
    Matrix23d C;
    Eigen::Matrix2d S;
    Matrix32d L;
    Z z_hat;
    Z z;
    Z z_diff;
    double r, r_hat, r2_hat;
    for (beac_it = beacons_sensed.begin(); beac_it != beacons_sensed.end(); beac_it++) {
      beac = *beac_it;
      c2c = beac->center - x.block<2, 1>(0, 0);
      c2c_hat = beac->center - x_hat.block<2, 1>(0, 0);
      r = c2c.norm();
      r_hat = c2c_hat.norm();
      r2_hat = pow(r_hat, 2);
      z << r, atan2Vec(c2c) - x(2);
      z_hat << r_hat, atan2Vec(c2c_hat) - x_hat(2);
      C << -c2c(0) / r_hat, -c2c(1) / r_hat, 0, c2c(1) / r2_hat, -c2c(0) / r2_hat, -1;
      S = C * Sigma_bar * C.transpose() + Rc_beacon_base;
      L = Sigma_bar * C.transpose() * S.inverse();
      z_diff = z - z_hat;
      z_diff(1) = bot_mod2pi(z_diff(1));
      x_hat += L * (z_diff);
      Sigma_bar -= L * C * Sigma_bar;
    }

    Sigma = Sigma_bar;

    x_hat(2) = bot_mod2pi(x_hat(2));
    x_carrot(2) = bot_mod2pi(x_carrot(2));
    X x_error = x_hat - x_carrot;
    x_error(2) = bot_mod2pi(x_error(2));

    StateCov Global_to_Body = StateCov::Identity();
    Global_to_Body.block<2, 2>(0, 0) = Eigen::Rotation2Dd(-x_hat(2)).matrix(); //-theta since we're going from global to body
    X x_error_body = Global_to_Body * x_error;
    u_control = -K * x_error_body;

    if (lcmgl != NULL) {
      if (animate_lcmgl) {
        bot_lcmgl_color3f(lcmgl, 1, 0, 0);
        this->lcmgl_state(lcmgl, x_carrot);

        bot_lcmgl_color3f(lcmgl, 0, 1, 0);
        this->lcmgl_state(lcmgl, x);

        bot_lcmgl_color3f(lcmgl, 0, 0, 1);
        this->lcmgl_state(lcmgl, x_hat);
        bot_lcmgl_switch_buffer(lcmgl);
        usleep(10000);
      }
      else {
        bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_SAMPLE_TRAJ);
      }
    }

    Eigen::Vector2d xy = x.block<2, 1>(0, 0);

    if (this->c_space->collides(xy)) {
      if (lcmgl != NULL) {
        if (!animate_lcmgl) {
          bot_lcmgl_end(lcmgl);
          bot_lcmgl_color3f(lcmgl, BT_STEER_COLOR_COLLISION);
          bot_lcmgl_point_size(lcmgl, BT_SIMULATE_SIZE);
          bot_lcmgl_begin(lcmgl, GL_POINTS);
          bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_SAMPLE_TRAJ);
          bot_lcmgl_end(lcmgl);
          endDepthTest(lcmgl);
        }
      }
      return false;
    }
  }

  //    if (this->onStateGoal(xy))
  //      return true;
  //    else
  //      return false;

  if (lcmgl != NULL) {
    bot_lcmgl_end(lcmgl);
    endDepthTest(lcmgl);
  }
  return true;

}

void DubinsBeacon::lcmgl_drawPropagate(const X & x, const A & aug, const std::list<P *> path_data, bool collision,
    bot_lcmgl_t * lcmgl, int animate_usleep)
{
  bot_lcmgl_line_width(lcmgl, 3);

  if (collision)
    bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR_COLLISION);
  else
    bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR);

  initDepthTest(lcmgl);

  aug.lcmgl_print(lcmgl, x, this->nsigma);

  this->lcmgl_state(lcmgl, x);
  bot_lcmgl_line_width(lcmgl, BT_EDGE_WIDTH);
  dubins_lcmgl_printPath(path_data, lcmgl);

  endDepthTest(lcmgl);

  bot_lcmgl_switch_buffer(lcmgl);

  if (animate_usleep > 0) {
    usleep(animate_usleep);
  }
}

void DubinsBeacon::lcmgl_drawSteer(const X & x_cur, const std::list<P *> path_data, bool collision,
    bot_lcmgl_t * lcmgl, int animate_usleep)
{
  if (collision)
    bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR_COLLISION);
  else
    bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR);

  initDepthTest(lcmgl);

  this->lcmgl_state(lcmgl, x_cur);
  bot_lcmgl_line_width(lcmgl, BT_EDGE_WIDTH);
  dubins_lcmgl_printPath(path_data, lcmgl);

  endDepthTest(lcmgl);
  bot_lcmgl_switch_buffer(lcmgl);
  if (animate_usleep > 0) {
    usleep(animate_usleep);
  }

}

void DubinsBeacon::sample(X & x_samp)
{
  Eigen::Vector2d xy_samp;
  this->c_space->sample(xy_samp);
  x_samp.block<2, 1>(0, 0) = xy_samp;
  x_samp.block<1, 1>(2, 0) = Eigen::MatrixXd::Random(1, 1) * M_PI;
}

int DubinsBeacon::getStateDimension()
{
  return 3;
}

bool DubinsBeacon::onStateGoal(const X & x)
{
  return this->goal_region->contains(x.block<2, 1>(0, 0));
}

bool DubinsBeacon::onAugGoal(const X & x, const A & aug)
{
  StateCov sigma_total = aug.Sigma + aug.Lambda;
  return this->goal_region->contains_cc(x.block<2, 1>(0, 0), sigma_total.block<2, 2>(0, 0), this->nsigma);
}

bool DubinsBeacon::compareAugPartial(const A & aug1, const A & aug2, double epsilon)
{
  return aug1.comparePartial(aug2, epsilon);
}

bool DubinsBeacon::compareAugTotal(const A & aug1, const A & aug2)
{
  return aug1.compareTotal(aug2);
}

double DubinsBeacon::getAugCost(const A & aug)
{
  return aug.cost;
}

double DubinsBeacon::distance(const X & state1, const X & state2)
{
  double d1 = dubinsGetDistance(this->r_min, state1, state2);
  double d2 = dubinsGetDistance(this->r_min, state2, state1);
  if (d1 < d2)
    return d1;
  else
    return d2;
}

void DubinsBeacon::lcmgl_system(bot_lcmgl_t * lcmgl)
{
  //begin lcmgl
  initDepthTest(lcmgl);

  //lqg regions
  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_color3f(lcmgl, BT_REGION_COLOR);
  std::list<Circle *>::iterator beac_it;
  for (beac_it = this->beacons.begin(); beac_it != this->beacons.end(); beac_it++) {
    bot_lcmgl_begin(lcmgl, GL_POLYGON);
    (*beac_it)->lcmgl_print(lcmgl);
    bot_lcmgl_end(lcmgl);
  }

  //goal box
  bot_lcmgl_color3f(lcmgl, BT_GOAL_COLOR);
  bot_lcmgl_begin(lcmgl, GL_POLYGON);
  this->goal_region->lcmgl_print(lcmgl);
  bot_lcmgl_end(lcmgl);

  //initial belief
  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR);
  aug_init.lcmgl_print(lcmgl, x_init, this->nsigma);

  this->c_space->lcmgl_print(lcmgl);

  //end lcmgl
  endDepthTest(lcmgl);
}

void DubinsBeacon::lcmgl_state(bot_lcmgl_t * lcmgl, const X & x)
{
  bot_lcmgl_line_width(lcmgl, 5);
  bot_lcmgl_point_size(lcmgl, 10);

  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_LINES);
  bot_lcmgl_vertex3d(lcmgl, x(0) + this->length * cos(x(2)), x(1) + this->length * sin(x(2)), Z_LINES);
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_begin(lcmgl, GL_POINTS);
  bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_LINES);
  bot_lcmgl_end(lcmgl);
}

void DubinsBeacon::lcmgl_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data)
{
  dubins_lcmgl_printPath(path_data, lcmgl);
}

void DubinsBeacon::lcmgl_aug(bot_lcmgl_t * lcmgl, const X & x, const A & aug)
{
  aug.lcmgl_print(lcmgl, x, this->nsigma);
}

void DubinsBeacon::lcmgl_aug_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_init,
    const A & aug_final)
{

  double dl_scale = 5.0; //speed things up for drawing
  double draw_dl = dl_scale * this->dl;

  double L = dubinsGetPathLength(path_data);
  double total_dist = L / this->v_0;

  double start_offset = aug_init.trace * DRAW_ENTROPY_SCALE_DUB;
  double end_offset = aug_final.trace * DRAW_ENTROPY_SCALE_DUB;

  X x_carrot = path_data.front()->getStartPose();
  U u_step;
  std::list<P *>::const_iterator path_it = path_data.begin();

  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), start_offset);

  double cur_dist = 0;
  double cur_offset;

  int step_ccw;
  while (dubinsAdvanceCarrot(path_data, path_it, x_carrot, &step_ccw, draw_dl)) {
    cur_dist += draw_dl;
    cur_offset = start_offset * (1 - cur_dist / total_dist) + (cur_dist / total_dist) * end_offset;
    bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), cur_offset);
  }
  bot_lcmgl_end(lcmgl);
}

void DubinsBeacon::lcmgl_aug_along_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_begin)
{
  //hack the nsigma to make it less conservative since the propogations along the optimal path might not exactly match the propagation when the path was built
  double nsigma_scale = 2.0 / 4.0;
  double nsigma_scale_inv = 1.0 / nsigma_scale;
  this->nsigma *= nsigma_scale;

  this->prop_demo = false;

  A aug_cur = aug_begin;

  X x_carrot = path_data.front()->getStartPose();
  U u_step;
  std::list<P *>::const_iterator path_it = path_data.begin();
  int step_ccw;
  int ii = 0;

  bot_lcmgl_color3f(lcmgl, BT_OPTIMAL_PATH_COLOR);
  bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_PATH_WIDTH);
  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), aug_cur.trace * DRAW_ENTROPY_SCALE);

  while (dubinsAdvanceCarrot(path_data, path_it, x_carrot, &step_ccw, this->dl)) {
    u_step << v_0, step_ccw * this->omega_0;
    step(x_carrot, u_step, aug_cur.Sigma, aug_cur.Lambda);

    bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), (aug_cur.Sigma + aug_cur.Lambda).trace() * DRAW_ENTROPY_SCALE);

    if (ii % 10 == 0) {
      bot_lcmgl_end(lcmgl);
      bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_AUG_WIDTH);
      aug_cur.setTrace();
      aug_cur.lcmgl_print(lcmgl, x_carrot, this->nsigma * nsigma_scale_inv);
      bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
      bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1),
          (aug_cur.Sigma + aug_cur.Lambda).trace() * DRAW_ENTROPY_SCALE);

    }
    ii++;
  }

  bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), (aug_cur.Sigma + aug_cur.Lambda).trace() * DRAW_ENTROPY_SCALE);
  bot_lcmgl_end(lcmgl);
  bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_AUG_WIDTH);
  aug_cur.setTrace();
  aug_cur.lcmgl_print(lcmgl, x_carrot, this->nsigma * nsigma_scale_inv);

  //this is the repair of a crazy hack
  this->nsigma *= nsigma_scale_inv;
}

DubinsBeacon::A DubinsBeacon::getAugInit()
{
  return aug_init;
}

DubinsBeacon::X DubinsBeacon::getXInit()
{
  return x_init;
}

//
//using namespace std;
//using namespace Eigen;
//
//int main(int argc, char **argv)
//{
//  lcm_t * lcm = bot_lcm_get_global(NULL);
//  bot_lcmgl_t * lcmgl_1 = bot_lcmgl_init(lcm, "lcmgl_1");
//  bot_lcmgl_t * lcmgl_2 = bot_lcmgl_init(lcm, "lcmgl_2");
//  bot_lcmgl_t * lcmgl_3 = bot_lcmgl_init(lcm, "lcmgl_3");
//
//  double sigma0 = .0001;
//  int animate_usleep = 1000;
//  A aug_init = A(sigma0, 0, 0);
//  //  aug_init.Lambda(0, 0) = 0.001;
//  //  aug_init.Lambda(1, 1) = 1;
//  //  aug_init.Lambda(2, 2) = 0.001;
//
//
//  Vector2d env_center, env_size, goal_center, goal_size;
//  env_center << 0, 0;
//  env_size << 20, 20;
//  RegionCSpace<2> * c_space = new RegionCSpace<2> (new Box(env_center, env_size));
//
//  goal_center << 8, 8;
//  goal_size << 2, 2;
//  Region<2> * goal_region = new Box(goal_center, goal_size);
//  X x_init, x_final, x_error, x_path_init;
//  x_init << 0, 0, 0;
//  x_final << 8, 8, 0;
//  x_error << 0, 0, 0;
//  x_path_init = x_init + x_error;
//
//  DubinsBeacon * dub = new DubinsBeacon(x_init, aug_init, c_space, goal_region);
//  Vector2d beacon_center;
//  beacon_center << 6, 4;
//  dub->beacons.push_back(new Circle(beacon_center, 3));
//
//  dub->lcmgl_system(lcmgl_1);
//  bot_lcmgl_switch_buffer(lcmgl_1);
//
//  bot_lcmgl_line_width(lcmgl_2, 20);
//  bot_lcmgl_point_size(lcmgl_2, 20);
//  bot_lcmgl_color3f(lcmgl_2, 1, 0, 0);
//  list<P *> path_data = list<P *> ();
//  dub->steer(x_path_init, x_final, path_data, lcmgl_2, animate_usleep);
//  bot_lcmgl_line_width(lcmgl_2, 3);
//  bot_lcmgl_color3f(lcmgl_2, BT_EDGE_COLOR);
//  dubins_lcmgl_printPath(path_data, lcmgl_2);
//  dub->lcmgl_aug_along_path(lcmgl_2, path_data, aug_init);
//  bot_lcmgl_switch_buffer(lcmgl_2);
//
//  bot_lcmgl_line_width(lcmgl_3, 20);
//  bot_lcmgl_point_size(lcmgl_3, 20);
//  bot_lcmgl_color3f(lcmgl_3, 0, 0, 1);
//  for (int ii = 0; ii < 1000; ii++) {
//    dub->simulate(path_data, lcmgl_3);
//  }
//  bot_lcmgl_switch_buffer(lcmgl_3);
//
//  return 0;
//}

