#ifndef _LQG_PLANNING_HPP
#define _LQG_PLANNING_HPP
#include <Eigen/Dense>
#include <bot_lcmgl_client/lcmgl.h>
#include "system_defs.hpp"
#include "drawing_defs.hpp"
#include <bot_core/lcm_util.h>
#include <GL/gl.h>
#include <eigen_utils/eigen_utils.hpp>
#include "environment_defs.hpp"

using namespace eigen_utils;

/*
 * straight line path between two points
 *
 * start - starting point
 * end - ending point
 * dpdt - vector from start to end with magnitude equal to the speed of movement
 */

template<int N>
class Line {
public:
  typedef Eigen::Matrix<double, N, 1> X;EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  X start;
  X end;
  X vec;
  X unit_vec;
  double length;
  Line(const X & start, const X & end)
  {
    this->start = start;
    this->end = end;
    this->vec = this->end - this->start;
    this->length = this->vec.norm();
    this->unit_vec = this->vec / this->length;

  }

  void lcmgl_print(bot_lcmgl_t * lcmgl) const
      {
    bot_lcmgl_begin(lcmgl, GL_LINES);
    bot_lcmgl_vertex3d(lcmgl, start(0), start(1), Z_LINES);
    bot_lcmgl_vertex3d(lcmgl, end(0), end(1), Z_LINES);
    bot_lcmgl_end(lcmgl);
  }
};

/*
 * the augmented state for a gaussian pomdp system
 *
 * Sigma - Kalman filter covariance
 * Lambda - Predicted covariance of the state estimate mean
 * cost - accumulated cost up to this state
 * trace - the trace of Simga+Lambda, stored for convenience
 */

template<int N>
class LQGAug {
public:
  typedef Eigen::Matrix<double, N, 1> X;
  typedef Eigen::Matrix<double, N, N> StateCov;EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateCov Sigma;
  StateCov Lambda;
  double cost;
  double trace;

  LQGAug(StateCov & Sigma, StateCov & Lambda, double cost)
  {
    this->Sigma = Sigma;
    this->Lambda = Lambda;
    this->cost = cost;
    updateTrace();
  }

  LQGAug(double sigma, double lambda, double cost)
  {
    this->Sigma = sigma * StateCov::Identity();
    this->Lambda = lambda * StateCov::Identity();

    this->cost = cost;

    StateCov total_sig = Sigma + Lambda;
    this->trace = total_sig.trace();
  }

  double updateTrace()
  {
    StateCov total_sig = Sigma + Lambda;
    this->trace = total_sig.trace();
    return this->trace;
  }

  LQGAug()
  {

  }

  /*
   * returns true if this dominates (i.e. is less than) other
   */
  bool comparePartial(const LQGAug & other, double epsilon) const
      {
    if (epsilon == 0) {
      epsilon = .00000001;
    }

    double cost_diff = other.cost - this->cost;
    if (cost_diff < 0)
      return false;

    StateCov Sigma_diff = other.Sigma - this->Sigma;
    double min_eig_Sigma;

    //do a special case eigenvalue check for 2x2 diagonal matrices for speed
    if (N == 2 && (Sigma_diff(1, 0) < 0.001 * (Sigma_diff(1, 1) * Sigma_diff(0, 0)))) {
      if (Sigma_diff(0, 0) < Sigma_diff(1, 1))
        min_eig_Sigma = Sigma_diff(0, 0);
      else
        min_eig_Sigma = Sigma_diff(1, 1);
    }
    else { //normal case
      Eigen::SelfAdjointEigenSolver<StateCov> eigen_solver(Sigma_diff);
      X eig_Sigma_diff = eigen_solver.eigenvalues();
      min_eig_Sigma = eig_Sigma_diff.minCoeff();
    }
    if (min_eig_Sigma < -epsilon) // * cost_diff
      return false;

    StateCov Lambda_diff = other.Lambda - this->Lambda;
    double min_eig_Lambda;
    if (N == 2 && (Lambda_diff(1, 0) < 0.001 * (Lambda_diff(1, 1) * Lambda_diff(0, 0)))) {
      if (Lambda_diff(0, 0) < Lambda_diff(1, 1))
        min_eig_Lambda = Lambda_diff(0, 0);
      else
        min_eig_Lambda = Lambda_diff(1, 1);
    }
    else {
      Eigen::SelfAdjointEigenSolver<StateCov> eigen_solver(Lambda_diff);
      X eig_Lambda_diff = eigen_solver.eigenvalues();
      min_eig_Lambda = eig_Lambda_diff.minCoeff();
    }
    if (min_eig_Lambda < -epsilon) // * cost_diff
      return false;

    return true;
  }

  /*
   * returns true if this has lower cost
   */
  bool compareTotal(const LQGAug & other) const
      {
    return this->cost < other.cost;
  }

  virtual double getCost()
  {
    return cost;
  }

  void setTrace()
  {
    this->trace = (this->Sigma + this->Lambda).trace();
  }

  void lcmgl_print(bot_lcmgl_t * lcmgl, const X & xyz_vec, double rho) const
      {
    StateCov sigma_total = this->Sigma + this->Lambda;

    Eigen::Matrix2d sigma_total2d;
    sigma_total2d << sigma_total(0, 0), sigma_total(0, 1), sigma_total(1, 0), sigma_total(1, 1);

    Eigen::Matrix2d Lambda2d;
    Lambda2d << this->Lambda(0, 0), this->Lambda(0, 1), this->Lambda(1, 0), this->Lambda(1, 1);

    //    Eigen::Vector2d mu2d;
    //    mu2d << xyz_vec(0, 0), xyz_vec(1, 0);

    Eigen::Vector3d mu3d;

    mu3d << xyz_vec(0), xyz_vec(1), this->trace * DRAW_ENTROPY_SCALE;

    bot_lcmgl_cov_ellipse(lcmgl, sigma_total2d, mu3d, rho, false);
    bot_lcmgl_cov_ellipse(lcmgl, Lambda2d, mu3d, rho, true);

  }
};

template<int N>
std::ostream& operator<<(std::ostream& output, const LQGAug<N> & aug)
{
  output << "cost: " << aug.cost << "\nSigma:\n" << aug.Sigma << "\nLambda:\n" << aug.Lambda << std::endl;
  return output; // for multiple << operators.
}

template<int N>
class LQGProperties {
public:
  typedef Eigen::Matrix<double, N, 1> X;
  typedef Eigen::Matrix<double, N, N> StateCov;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateCov Q;
  StateCov A;
  StateCov B;
  StateCov C;
  StateCov K;
  StateCov R;

  LQGProperties(const StateCov & A, const StateCov & B, const StateCov & C, const StateCov & Q, const StateCov & R,
      const StateCov & K)
  {
    this->A = A;
    this->B = B;
    this->C = C;
    this->Q = Q;
    this->K = K;
    this->R = R;

  }

  LQGProperties()
  {
    StateCov I = StateCov::Identity();
    A = B = C = Q = K = R = I;
  }

  void scale(double scale)
  {
    this->Q *= scale;
    this->R /= scale;
  }

};

template<int N>
class LQGHyperCubeRegion: public LQGProperties<N>, public HyperBox<N> {
public:
  LQGHyperCubeRegion(const LQGProperties<N> lqg_prop, HyperBox<N> box) :
      HyperBox<N>(box), LQGProperties<N>(lqg_prop)
  {

  }
};

template<int N>
class LQGBoxSystem: public AugSystemInterface<Eigen::Matrix<double, N, 1>, Line<N> , LQGAug<N> > {
public:
  typedef Eigen::Matrix<double, N, 1> X;
  typedef Eigen::Matrix<double, N, N> StateCov;
  typedef Line<N> P;
  typedef LQGAug<N> A;

  double nsigma;
  double dxdt;
  std::list<LQGHyperCubeRegion<N> *> lqg_regions;
  LQGProperties<N> lqg_base;
  Region<N> * goal_region;
  CSpaceInterface<N> * c_space;
  A aug_init;
  X x_init;

  A getAugInit()
  {
    return aug_init;
  }

  X getXInit()
  {
    return x_init;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LQGBoxSystem(double nsigma) :
      lqg_base()
  {
    this->nsigma = nsigma;
  }

  ~LQGBoxSystem()
  {
    freeList(lqg_regions);
  }

  virtual LQGProperties<N> * getMLProperties(const X & x)
  {
    typename std::list<LQGHyperCubeRegion<N> *>::iterator prop_it;

    for (prop_it = this->lqg_regions.begin(); prop_it != this->lqg_regions.end(); prop_it++) {
      if ((*prop_it)->contains(x)) {
        return *prop_it;
      }
    }
    return &this->lqg_base;
  }

  virtual LQGProperties<N> * getCCProperties(const X & x, const StateCov & sigma_total)
  {
    LQGProperties<N> *cc_prop = &this->lqg_base;
    double cc_trace = cc_prop->R.trace();

    double region_trace;
    LQGHyperCubeRegion<N> * lqg_region;
    typename std::list<LQGHyperCubeRegion<N> *>::iterator prop_it;
    //note that this assumes all the regions are disjoint
    for (prop_it = this->lqg_regions.begin(); prop_it != this->lqg_regions.end(); prop_it++) {
      lqg_region = *prop_it;
      region_trace = lqg_region->R.trace();
      if (lqg_region->contains_cc(x, sigma_total, this->nsigma)) {
        if (region_trace < cc_trace) {
          cc_prop = lqg_region;
          cc_trace = region_trace;
        }
      }
    }
    return cc_prop;
  }

  void step(const X & x, StateCov & Sigma, StateCov & Lambda, double scale = 1)
  {
    //    LQGProperties<N> * prop = getMLProperties(x);
    LQGProperties<N> * prop = getCCProperties(x, Sigma + Lambda);
    LQGProperties<N> * scaled_prop;

    bool scaled = false;

    if (scale < (1 - 1e-15)) {
      scaled_prop = new LQGProperties<N>(*prop);
      scaled_prop->scale(scale);
      prop = scaled_prop;
      scaled = true;
    }

    //Kalman Prediction
    StateCov Sigma_bar = prop->A * Sigma * prop->A.transpose() + prop->Q;

    //Innovation Covariance
    StateCov S = prop->C * Sigma_bar * prop->C.transpose() + prop->R;

    //Kalman Gain
    StateCov L = Sigma_bar * prop->C.transpose() * S.inverse();

    //Kalman Update
    Sigma = (Eigen::MatrixXd::Identity(N, N) - L * prop->C.transpose()) * Sigma_bar;

    //Prediction Distribution
    Lambda = (prop->A - prop->B * prop->K) * Lambda * (prop->A - prop->B * prop->K).transpose() + Sigma_bar
        * prop->C.transpose() * L;

    if (scaled)
      delete scaled_prop;
  }

  bool steerApprox(const X & initial_state, X & final_state, std::list<P *> & path_data, bot_lcmgl_t * lcmgl,
      int animate_usleep)
  {
    return steer(initial_state, final_state, path_data, lcmgl, animate_usleep);
  }

  bool steer(const X & initial_state, const X & final_state, std::list<P *> & path_data, bot_lcmgl_t * lcmgl,
      int animate_usleep)
  {
    Line<N> * path_el = new Line<N>(initial_state, final_state);

    X to_go = path_el->vec;
    X dpdt = path_el->vec * (this->dxdt / path_el->length);

    X current_state = initial_state;

    bool collision;
    while (to_go.dot(dpdt) > 0) {
      collision = this->c_space->collides(current_state);

      if (lcmgl != NULL) {
        Line<N> path_el_draw = Line<N>(initial_state, current_state);
        std::list<P *> path_data_draw = std::list<P *>(1, &path_el_draw);
        this->lcmgl_drawSteer(path_data_draw, collision, lcmgl, animate_usleep);
      }
      if (collision) {
        delete path_el;
        return false;
      }
      current_state += dpdt;
      to_go = final_state - current_state;
    }

    path_data.push_back(path_el);
    return true;
  }

  bool propagate(const std::list<P *> & path_data, const A & aug_init, A & aug_final, double * cost,
      bot_lcmgl_t * lcmgl, int animate_usleep)
  {

    Line<N> * path_el = path_data.front();
    X current_state = path_el->start;
    X dpdt = path_el->vec * (this->dxdt / path_el->length);
    current_state += dpdt; //step forward since through each loop we want to calculate the system changes to get to the next state

    *cost = path_el->length;
    aug_final = aug_init;
    StateCov Sigma_total = aug_final.Sigma + aug_final.Lambda;
    bool chance_constrained_collision = this->c_space->collides_cc(current_state, Sigma_total, this->nsigma);

    //    return !chance_constrained_collision;

    X to_go = path_el->end - current_state;

    while (to_go.dot(dpdt) > 0) {
      step(current_state, aug_final.Sigma, aug_final.Lambda);

      //      if (animate_usleep == -5) {
      //        std::cout << aug_final << std::endl;
      //        usleep(1000000);
      //      }

      Sigma_total = aug_final.Sigma + aug_final.Lambda;

      chance_constrained_collision = this->c_space->collides_cc(current_state, Sigma_total, this->nsigma);

      if (lcmgl != NULL) {
        this->lcmgl_drawPropagate(current_state, aug_final, chance_constrained_collision, lcmgl, animate_usleep);
      }

      if (chance_constrained_collision)
        return false;

      current_state += dpdt;
      to_go = path_el->end - current_state;
    }

    //    double fraction_step = to_go.norm() / this->dxdt;
    //    if (fraction_step > .0001) {
    //      step(current_state, aug_final.Sigma, aug_final.Lambda, fraction_step);
    //    }

    Sigma_total = aug_final.Sigma + aug_final.Lambda;
    current_state = path_el->end;
    chance_constrained_collision = this->c_space->collides_cc(current_state, Sigma_total, this->nsigma);

    if (lcmgl != NULL) {
      this->lcmgl_drawPropagate(current_state, aug_final, chance_constrained_collision, lcmgl, animate_usleep);
    }

    aug_final.cost += path_el->length;
    aug_final.trace = Sigma_total.trace();

    if (chance_constrained_collision)
      return false;
    else
      return true;

  }

  bool simulate(const std::list<P *> & path_data, bot_lcmgl_t * lcmgl)
  {

    if (lcmgl != NULL) {
      initDepthTest(lcmgl);
      bot_lcmgl_color3f(lcmgl, BT_SIMULATE_COLOR);
      bot_lcmgl_line_width(lcmgl, BT_SIMULATE_WIDTH);
      bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    }

    X x_carrot = x_init;
    X x_hat = x_init + randn(this->aug_init.Lambda);
    X x = x_hat + randn(this->aug_init.Sigma);
    StateCov Sigma = this->aug_init.Sigma;
    StateCov L, S;
    StateCov I = StateCov::Identity();

    typename std::list<P *>::const_iterator path_it = path_data.begin();

    X u_step;
    X z;

    bool collision;
    LQGProperties<N> * prop;

    bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_SAMPLE_TRAJ);

    while (advanceCarrot(path_data, path_it, x_carrot, u_step)) {
      x_hat += u_step;
      x += u_step;
      prop = getMLProperties(x);
      x += randn(prop->Q);
      Sigma += prop->Q;
      z = x + randn(prop->R);
      S = Sigma + prop->R;
      L = Sigma * S.inverse();
      x_hat = x_hat + L * (z - x_hat);
      Sigma = (I - L) * Sigma;

      x -= prop->K * (x_hat - x_carrot);
      x_hat -= prop->K * (x_hat - x_carrot);

      if (lcmgl != NULL) {
        bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_SAMPLE_TRAJ);
      }

      if (this->c_space->collides(x)) {
        if (lcmgl != NULL) {
          bot_lcmgl_end(lcmgl);
          bot_lcmgl_color3f(lcmgl, BT_STEER_COLOR_COLLISION);
          bot_lcmgl_point_size(lcmgl, BT_SIMULATE_SIZE);
          bot_lcmgl_begin(lcmgl, GL_POINTS);
          bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_SAMPLE_TRAJ);
          bot_lcmgl_end(lcmgl);
          endDepthTest(lcmgl);
        }
        return false;
      }
    }

    //    if (this->onStateGoal(x))
    //      return true;
    //    else
    //      return false;

    if (lcmgl != NULL) {
      bot_lcmgl_end(lcmgl);
      endDepthTest(lcmgl);
    }
    return true;

  }

  bool advanceCarrot(const std::list<P *> & path_data, typename std::list<P*>::const_iterator & path_it, X & x_carrot,
      X & u_step)
  {
    P * path_el = *path_it;
    X dpdt = path_el->vec * (this->dxdt / path_el->length);
    X x_next = x_carrot + dpdt;
    X to_go = path_el->end - x_next;
    double remaining_path_length = to_go.dot(dpdt);
    if (remaining_path_length > 0) {
      u_step = dpdt;
      x_carrot = x_next;
      return true;
    }

    path_it++;
    if (path_it == path_data.end())
      return false;

    while (-remaining_path_length > (*path_it)->length) {
      remaining_path_length += (*path_it)->length;
      path_it++;
      if (path_it == path_data.end())
        return false;
    }

    path_el = *path_it;
    dpdt = path_el->vec * (this->dxdt / path_el->length);
    X x_carrot_old = x_carrot;
    x_carrot = path_el->start + dpdt * (-remaining_path_length);
    u_step = x_carrot - x_carrot_old;
    return true;
  }

  void lcmgl_drawPropagate(const X & x, const A & aug, bool collision, bot_lcmgl_t * lcmgl, int animate_usleep)
  {
    bot_lcmgl_line_width(lcmgl, 3);

    if (collision)
      bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR_COLLISION);
    else
      bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR);

    initDepthTest(lcmgl);

    aug.lcmgl_print(lcmgl, x, this->nsigma);

    endDepthTest(lcmgl);

    bot_lcmgl_switch_buffer(lcmgl);

    if (animate_usleep > 0) {
      usleep(animate_usleep);
    }
  }

  void lcmgl_drawSteer(const std::list<P *> & path_data_draw, bool collision, bot_lcmgl_t * lcmgl, int animate_usleep)
  {
    bot_lcmgl_line_width(lcmgl, 3);
    if (collision)
      bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR_COLLISION);
    else
      bot_lcmgl_color3f(lcmgl, BT_PROPAGATE_COLOR);

    initDepthTest(lcmgl);

    typename std::list<P *>::const_iterator path_it;
    for (path_it = path_data_draw.begin(); path_it != path_data_draw.end(); path_it++) {
      (*path_it)->lcmgl_print(lcmgl);
    }
    endDepthTest(lcmgl);

    bot_lcmgl_switch_buffer(lcmgl);
    if (animate_usleep > 0) {
      usleep(animate_usleep);
    }

  }

  void sample(X & x_samp)
  {
    this->c_space->sample(x_samp);
  }

  int getStateDimension()
  {
    return N;
  }

  bool onStateGoal(const X & x)
  {
    return this->goal_region->contains(x);
  }

  bool onAugGoal(const X & x, const A & aug)
  {
    StateCov sigma_total = aug.Sigma + aug.Lambda;
    return this->goal_region->contains_cc(x, sigma_total, this->nsigma);
  }

  bool compareAugPartial(const A & aug1, const A & aug2, double epsilon)
  {
    return aug1.comparePartial(aug2, epsilon);
  }

  virtual bool compareAugTotal(const A & aug1, const A & aug2)
  {
    return aug1.compareTotal(aug2);
  }

  virtual double getAugCost(const A & aug)
  {
    return aug.cost;
  }

  double distance(const X & state1, const X & state2)
  {
    X diff = state1 - state2;
    return diff.norm();
  }

  void lcmgl_system(bot_lcmgl_t * lcmgl)
  {
    //begin lcmgl
    initDepthTest(lcmgl);

    //lqg regions
    bot_lcmgl_line_width(lcmgl, 3);
    bot_lcmgl_color3f(lcmgl, BT_REGION_COLOR);
    typename std::list<LQGHyperCubeRegion<N> *>::iterator prop_it;
    for (prop_it = this->lqg_regions.begin(); prop_it != this->lqg_regions.end(); prop_it++) {
      bot_lcmgl_begin(lcmgl, GL_POLYGON);
      (*prop_it)->lcmgl_print(lcmgl);
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

  void lcmgl_state(bot_lcmgl_t * lcmgl, const X & x)
  {
    bot_lcmgl_begin(lcmgl, GL_POINT);
    bot_lcmgl_vertex3d(lcmgl, x(0), x(1), Z_LINES);
    bot_lcmgl_end(lcmgl);
  }

  void lcmgl_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data)
  {
    typename std::list<P *>::const_iterator path_it;

    for (path_it = path_data.begin(); path_it != path_data.end(); path_it++) {
      (*path_it)->lcmgl_print(lcmgl);
    }
  }

  virtual void lcmgl_aug(bot_lcmgl_t * lcmgl, const X & x, const A & aug)
  {
    aug.lcmgl_print(lcmgl, x, this->nsigma);
  }

  virtual void lcmgl_aug_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_init,
      const A & aug_final)
  {

    typename std::list<P *>::const_iterator path_it;
    P * path_el;

    double L_total = 0;
    for (path_it = path_data.begin(); path_it != path_data.end(); path_it++) {
      path_el = *path_it;
      L_total += path_el->length;
    }

    double L_cur = 0;

    path_it = path_data.begin();
    path_el = *path_it;
    double start_offset = aug_init.trace * DRAW_ENTROPY_SCALE;

    double end_offset = aug_final.trace * DRAW_ENTROPY_SCALE;

    double cur_offset;
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, path_el->start(0), path_el->start(1), start_offset);

    for (; path_it != path_data.end(); path_it++) {
      path_el = *path_it;
      L_cur += path_el->length;
      cur_offset = start_offset * (1 - L_cur / L_total) + (L_cur / L_total) * end_offset;
      bot_lcmgl_vertex3d(lcmgl, path_el->end(0), path_el->end(1), cur_offset);
    }
    bot_lcmgl_end(lcmgl);
  }

  void lcmgl_aug_along_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_begin)
  {
    //hack the nsigma to make it less conservative since the propogations along the optimal path might not exactly match the propagation when the path was built
    double nsigma_scale = 2.0 / 4.0;
    double nsigma_scale_inv = 1.0 / nsigma_scale;
    this->nsigma *= nsigma_scale;

    typename std::list<P *>::const_iterator path_it = path_data.begin();

    X x_carrot = (*path_it)->start;

    X x_line_start = x_carrot;
    X x_line_end, x_line_end_tol;

    A aug_cur = aug_begin;
    //        aug_cur.lcmgl_print(lcmgl, x_line_start, this->nsigma);

    X u_step;
    P * path_segment;
    std::list<P *> path_prop = std::list<P *>();

    double cost;

    //initialize to draw path segment
    bot_lcmgl_color3f(lcmgl, BT_OPTIMAL_PATH_COLOR);
    bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_PATH_WIDTH);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), aug_cur.trace * DRAW_ENTROPY_SCALE);

    LQGAug<N> aug_prev;
    int ii = 0;
    while (advanceCarrot(path_data, path_it, x_carrot, u_step)) {
      x_line_end = x_carrot;
      x_line_end_tol = x_line_end + .001 * (x_carrot - x_line_start);
      path_segment = new Line<N>(x_line_start, x_line_end_tol);
      path_prop.push_front(path_segment);
      aug_prev = aug_cur;

      propagate(path_prop, aug_prev, aug_cur, &cost, NULL, -5);

      bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), aug_cur.trace * DRAW_ENTROPY_SCALE);

      path_prop.pop_front();
      delete path_segment;
      x_line_start = x_line_end;

      ii++;

      if (ii > 20) {
        // end path segment drawing to draw augs
        bot_lcmgl_end(lcmgl);
        bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_AUG_WIDTH);
        aug_cur.lcmgl_print(lcmgl, x_line_end, this->nsigma * nsigma_scale_inv);

        //restart path segment drawing
        bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_PATH_WIDTH);
        bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
        bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), aug_cur.trace * DRAW_ENTROPY_SCALE);
        ii = 0;
      }

    }
    x_line_end = x_carrot;
    path_segment = new Line<N>(x_line_start, x_line_end);
    path_prop.push_front(path_segment);
    propagate(path_prop, aug_cur, aug_cur, &cost, NULL, 0);
    path_prop.pop_front();
    delete path_segment;

    bot_lcmgl_vertex3d(lcmgl, x_carrot(0), x_carrot(1), aug_cur.trace * DRAW_ENTROPY_SCALE);
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_line_width(lcmgl, BT_OPTIMAL_AUG_WIDTH);
    aug_cur.lcmgl_print(lcmgl, x_line_end, this->nsigma * nsigma_scale_inv);

    //this is the repair of a crazy hack
    this->nsigma *= nsigma_scale_inv;
  }

};

#endif
