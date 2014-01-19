#include <iostream>
#include <Eigen/Dense>
#include "drawing_defs.hpp"
#include "environment_defs.hpp"
#include "lqg_planning.hpp"
#include <getopt.h>
#include <bot_core/bot_core.h>
#include <lcm/lcm.h>
#include <vector>
#include <eigen_utils/eigen_utils.hpp>
#include <mav_planning_utils/dubins.hpp>

#define DRAW_ENTROPY_SCALE_DUB DRAW_ENTROPY_SCALE

class DubinsBeacon: public AugSystemInterface<Eigen::Vector3d, DubinsPathPrimitive, LQGAug<3> > {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Vector3d X;
  typedef Eigen::Matrix3d StateCov;
  typedef Eigen::Matrix<double, 3, 2> Matrix32d;
  typedef Eigen::Matrix<double, 2, 3> Matrix23d;
  typedef Eigen::Vector2d U;
  typedef Eigen::Vector2d Z;
  typedef DubinsPathPrimitive P;
  typedef LQGAug<3> A;

  double v_0;
  double omega_0;
  double r_min;
  double dt;
  double dl;
  double length;

  double nsigma;
  Eigen::Matrix2d Mcbase;
  Eigen::Matrix2d Rc_beacon_base;
  Eigen::Matrix3d Rc_gps_base;
  Matrix23d K;

  std::list<Circle *> beacons;

  Region<2> * goal_region;
  RegionCSpace<2> * c_space;
  A aug_init;
  X x_init;

  bool prop_demo;

  DubinsBeacon(X x_init, A aug_init, RegionCSpace<2> * c_space, Region<2> * goal_region, double nsigma);

  ~DubinsBeacon();

  void addSquareWithBeaconCorners(Box * box, double r_beacon);

  virtual void getBeaconsML(const X & x, std::list<Circle *> & beacons_ml);

  virtual void getBeaconsCC(const X & x, const StateCov & sigma_total, std::list<Circle *> & beacons_cc);

  void getAB(const X & x, const U & u, StateCov & A, Matrix32d & B);

  void step(const X & x, const U & u, StateCov & Sigma, StateCov & Lambda);

  bool steer(const X & initial_state, const X & final_state, std::list<P *> & path_data, bot_lcmgl_t * lcmgl,
      int animate_usleep);

  bool steerApprox(const X & initial_state, X & final_state, std::list<P *> & path_data, bot_lcmgl_t * lcmgl,
        int animate_usleep);


  bool propagate(const std::list<P *> & path_data, const A & aug_init, A & aug_final, double * cost,
      bot_lcmgl_t * lcmgl, int animate_usleep);

  bool simulate(const std::list<P *> & path_data, bot_lcmgl_t * lcmgl);

  void lcmgl_drawPropagate(const X & x, const A & aug, const std::list<P *> path_data, bool collision,
      bot_lcmgl_t * lcmgl, int animate_usleep);

  void lcmgl_drawSteer(const X & x_cur, const std::list<P *> path_data, bool collision, bot_lcmgl_t * lcmgl,
      int animate_usleep);

  virtual void sample(X & x_samp);

  int getStateDimension();

  bool onStateGoal(const X & x);

  bool onAugGoal(const X & x, const A & aug);

  bool compareAugPartial(const A & aug1, const A & aug2, double epsilon);

  virtual bool compareAugTotal(const A & aug1, const A & aug2);

  virtual double getAugCost(const A & aug);

  double distance(const X & state1, const X & state2);

  void lcmgl_system(bot_lcmgl_t * lcmgl);

  void lcmgl_state(bot_lcmgl_t * lcmgl, const X & x);

  void lcmgl_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data);

  virtual void lcmgl_aug(bot_lcmgl_t * lcmgl, const X & x, const A & aug);

  virtual void lcmgl_aug_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_init,
      const A & aug_final);

  void lcmgl_aug_along_path(bot_lcmgl_t * lcmgl, const std::list<P *> & path_data, const A & aug_begin);

  A getAugInit();

  X getXInit();

};
