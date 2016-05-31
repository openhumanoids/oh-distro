#include "QPReactiveRecoveryPlan.hpp"
#include <unsupported/Eigen/Polynomials>
#include <Eigen/StdVector>
#include "drake/util/drakeGeometryUtil.h"
#include "drake/solvers/qpSpline/splineGeneration.h"
#include "drake/util/lcmUtil.h"
#include "drake/core/Gradient.h"
#include "lcmtypes/drc/reactive_recovery_debug_t.hpp"
extern "C" {
  #include "iris/solver.h"
}

#define CVXGEN_MAX_ROWS 3
#define CVXGEN_MAX_PTS 8

#define DEBUG

using namespace Eigen;

Vars vars;
Params params;
Workspace work;
Settings settings;

VectorXd QPReactiveRecoveryPlan::closestPointInConvexHull(const Ref<const VectorXd> &x, const Ref<const MatrixXd> &V) {

  int dim = x.size();

  Matrix<double, CVXGEN_MAX_ROWS, 1> x_resized;
  Matrix<double, CVXGEN_MAX_ROWS, CVXGEN_MAX_PTS> V_resized;

  if (dim > CVXGEN_MAX_ROWS) {
    fprintf(stderr, "x can not be more than %d elements\n", CVXGEN_MAX_ROWS);
    exit(1);
  }
  if (V.rows() != dim) {
    fprintf(stderr, "V must have same number of rows as x\n");
    exit(1);
  }
  if (V.cols() > CVXGEN_MAX_PTS) {
    fprintf(stderr, "V can not be larger than %d x %d\n", CVXGEN_MAX_ROWS, CVXGEN_MAX_PTS);
    exit(1);
  }

  x_resized.head(dim) = x;
  V_resized.block(0, 0, dim, V.cols()) = V;

  if (dim < CVXGEN_MAX_ROWS) {
    x_resized.tail(CVXGEN_MAX_ROWS - dim) = VectorXd::Zero(CVXGEN_MAX_ROWS-dim);
    V_resized.block(dim, 0, CVXGEN_MAX_ROWS - dim, V_resized.cols()) = MatrixXd::Zero(CVXGEN_MAX_ROWS-dim, V_resized.cols());
  }

  for (int i=V.cols(); i < CVXGEN_MAX_PTS; i++) {
    for (int j=0; j < CVXGEN_MAX_ROWS; j++) {
      V_resized(j,i) = V_resized(j,i-1);
    }
  }

  for (int j=0; j < CVXGEN_MAX_PTS; j++) {
    V_resized.col(j) = V_resized.col(j) - x_resized;
  }

  set_defaults();
  setup_indexing();
  settings.verbose = 0;


  double *src = V_resized.data();
  double *dest = params.Y;
  for (int i=0; i < CVXGEN_MAX_ROWS * CVXGEN_MAX_PTS; i++) {
    *dest++ = *src++;
  }

  solve();


  Map<VectorXd>y(vars.v, CVXGEN_MAX_ROWS);
  Map<VectorXd>w(vars.w, CVXGEN_MAX_PTS);

  y.head(dim) = y.head(dim) + x.head(dim);
  return y.head(dim);
}

Isometry3d QPReactiveRecoveryPlan::closestPoseInConvexHull(const Isometry3d &pose, const Ref<const MatrixXd> &V) {
  if (V.rows() < 2 || V.rows() > 3) {
    fprintf(stderr, "Vertices should have dimension 2 or 3\n");
    exit(1);
  }
  const int dim = V.rows();
  Isometry3d new_pose = pose;
  new_pose.translation().head(dim) = QPReactiveRecoveryPlan::closestPointInConvexHull(pose.translation().head(dim), V);
  return new_pose;
}

Polynomial<double> QPReactiveRecoveryPlan::bangBangPolynomial(double x0, double xd0, double u) {
  VectorXd coefs(3);
  coefs << x0 - 0.25*xd0*xd0/u,
           0.5*xd0,
           0.25*u;
  Polynomial<double> p(coefs);
  return p;
}

std::vector<double> realRoots(Polynomial<double> p) {
  VectorXd coefs = p.getCoefficients();
  double order = p.getDegree();
  std::vector<double> roots;
  if (order == 1) {
    // c0 + c1*t = 0;
    // t = -c0/c1
    roots.push_back(-coefs(0) / coefs(1));
  } else if (order == 2) {
    // c0 + c1*t + c2*t^2 = 0;
    // t = (-c1 +- sqrt(c1^2 - 4*c2*c0)) / (2*c2)
    double discriminant = pow(coefs(1), 2) - 4*coefs(2)*coefs(0);
    if (discriminant >= 0) {
      roots.push_back((-coefs(1) + sqrt(discriminant)) / (2*coefs(2)));
      roots.push_back((-coefs(1) - sqrt(discriminant)) / (2*coefs(2)));
    }
  } else {
    PolynomialSolver<double, Dynamic> poly_solver(coefs);
    poly_solver.realRoots(roots);
  }
  return roots;
}

std::vector<double> QPReactiveRecoveryPlan::expIntercept(const ExponentialForm &expform, double l0, double ld0, double u, int degree) {
  // Find the t >= 0 solutions to a*e^(b*t) + c == l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
  // using a taylor expansion up to power [degree]

  Polynomial<double> p_taylor = expform.taylorExpand(degree);
  Polynomial<double> p_bang = QPReactiveRecoveryPlan::bangBangPolynomial(l0, ld0, u);
  Polynomial<double> p_int = p_taylor - p_bang;

  std::vector<double> roots = realRoots(p_int);
  std::vector<double> nonneg_roots;

  for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
    if (*it > 0) {
      nonneg_roots.push_back(*it);
    } 
  }

  return nonneg_roots;
}

std::vector<BangBangIntercept> QPReactiveRecoveryPlan::bangBangIntercept(double x0, double xd0, double xf, double u_max) {
  std::vector<BangBangIntercept> intercepts;

  double us[2] = {u_max, -u_max};
  for (int i=0; i<2; i++) {
    double u = us[i];
    Polynomial<double> p = QPReactiveRecoveryPlan::bangBangPolynomial(x0 - xf, xd0, u);
    std::vector<double> roots = realRoots(p);
    for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
      double t = *it;
      if (t >= std::abs(xd0 / u)) {
        BangBangIntercept inter;
        inter.tf = t;
        inter.tswitch = 0.5 * (t - xd0 / u);
        inter.u = u;
        intercepts.push_back(inter);
        break;
      }
    }
  }
  return intercepts;
}


double QPReactiveRecoveryPlan::icpError(const Ref<const Vector2d> &r_ic, const FootStateMap &foot_states, const VertMap &foot_vertices) {
  if (foot_states.size() != 2) {
    throw std::runtime_error("isICPCaptured only supports 2 feet");
  }
  Matrix<double, 3, 8> all_vertices_in_world;

  int foot_count = 0;
  for (std::map<FootID, FootState>::const_iterator state = foot_states.begin(); state != foot_states.end(); ++state) {
    if (state->second.contact || 
        (state->second.pose.translation()(2) - state->second.terrain_height < this->capture_max_flyfoot_height)) {
      auto vert_it = foot_vertices.find(state->first);
      if (vert_it == foot_vertices.end()) {
        std::cout << footIDToName[state->first] << std::endl;
        throw std::runtime_error("Cannot find foot name in foot_vertices");
      }

      Matrix<double, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT> foot_vertices_in_world = state->second.pose * (this->capture_shrink_factor * vert_it->second);
      all_vertices_in_world.block(0, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT*foot_count, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT) = foot_vertices_in_world.block(0,0,3,QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT);
      ++foot_count;
    } 
  }

  Matrix<double, 2, Dynamic> active_vertices_in_world = all_vertices_in_world.block(0, 0, 2, foot_count * QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT);

  VectorXd r_ic_near = QPReactiveRecoveryPlan::closestPointInConvexHull(r_ic, active_vertices_in_world);
  return (r_ic - r_ic_near).norm();
}


bool QPReactiveRecoveryPlan::isICPCaptured(const Ref<const Vector2d> &r_ic, const FootStateMap &foot_states, const VertMap &foot_vertices) {
  if (foot_states.size() != 2) {
    throw std::runtime_error("isICPCaptured only supports 2 feet");
  }
  Matrix<double, 3, 8> all_vertices_in_world;

  int foot_count = 0;
  for (std::map<FootID, FootState>::const_iterator state = foot_states.begin(); state != foot_states.end(); ++state) {
    if (state->second.contact || 
        (state->second.pose.translation()(2) - state->second.terrain_height < this->capture_max_flyfoot_height)) {
      auto vert_it = foot_vertices.find(state->first);
      if (vert_it == foot_vertices.end()) {
        std::cout << footIDToName[state->first] << std::endl;
        throw std::runtime_error("Cannot find foot name in foot_vertices");
      }

      Matrix<double, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT> foot_vertices_in_world = state->second.pose * (this->capture_shrink_factor * vert_it->second);
      all_vertices_in_world.block(0, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT*foot_count, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT) = foot_vertices_in_world.block(0,0,3,QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT);
      ++foot_count;
    } else {
      return false;
    }
  }

  Matrix<double, 2, Dynamic> active_vertices_in_world = all_vertices_in_world.block(0, 0, 2, foot_count * QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT);

  VectorXd r_ic_near = QPReactiveRecoveryPlan::closestPointInConvexHull(r_ic, active_vertices_in_world);
  return (r_ic - r_ic_near).norm() < 1e-2; // threshold set by the accuracy of the cvxgen qp solver

}

ExponentialForm QPReactiveRecoveryPlan::icpTrajectory(double x_ic, double x_cop, double omega) {
  ExponentialForm icptraj((x_ic - x_cop), omega, x_cop);
  return icptraj;
}

bool tswitchComp(const BangBangIntercept int0, const BangBangIntercept int1) {
  return int0.tswitch < int1.tswitch;
}

bool errorCompare(const InterceptPlan plan0, const InterceptPlan plan1) {
  return plan0.error < plan1.error;
}

Isometry3d QPReactiveRecoveryPlan::getTWorldToLocal(const Isometry3d &icp, const Isometry3d &cop) {
  Isometry3d T_world_to_local = Isometry3d::Identity();
  T_world_to_local.rotate(AngleAxis<double>(-std::atan2(icp.translation().y() - cop.translation().y(), icp.translation().x() - cop.translation().x()), Vector3d(0, 0, 1)));
  T_world_to_local.translate(-cop.translation());
  return T_world_to_local;
}

double QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(const FootState foot_state, const BipedDescription &biped, Isometry3d &T_world_to_local) {
  std::vector<BangBangIntercept> xprime_axis_intercepts = 
    QPReactiveRecoveryPlan::bangBangIntercept((T_world_to_local * foot_state.pose).translation().y(),
                                              (T_world_to_local.linear() * foot_state.velocity.head(3))(1),
                                              0,
                                              biped.u_max);

  VectorXd times_to_xprime_axis(xprime_axis_intercepts.size());
  for (int i=0; i < xprime_axis_intercepts.size(); i++) {
    times_to_xprime_axis(i) = xprime_axis_intercepts[i].tf;
  }
  return times_to_xprime_axis.minCoeff();
}

double interceptPlanError(const InterceptPlan &plan, const std::map<FootID, FootState> &foot_states, const BipedDescription &biped) {
  Matrix<double, 3, 2*QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT> foot_vertices_in_world;
  foot_vertices_in_world.block(0, 0, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT) = plan.pose_next * biped.foot_vertices.find(plan.swing_foot)->second;
  foot_vertices_in_world.block(0, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT, 3, QP_REACTIVE_RECOVERY_VERTICES_PER_FOOT) = foot_states.find(plan.stance_foot)->second.pose * biped.foot_vertices.find(plan.stance_foot)->second;

  Vector2d icp_projected_onto_support_polygon = QPReactiveRecoveryPlan::closestPointInConvexHull(plan.icp_next.translation().head(2),
                                                                                                 foot_vertices_in_world.topRows(2));
  return (icp_projected_onto_support_polygon - plan.icp_next.translation().head(2)).norm();

  // return (plan.pose_next.translation().head(2) - plan.icp_plus_offset_next.translation().head(2)).norm();
}

Isometry3d snapToTerrain(const Isometry3d &pose, const double terrain_height, const Ref<const Vector3d> &terrain_normal) {
  Isometry3d pose_snapped = Isometry3d(Translation<double, 3>(Vector3d(pose.translation().x(), pose.translation().y(), terrain_height)));
  pose_snapped.linear() = pose.linear();

  Vector3d axis = (pose.rotation() * Vector3d(0, 0, 1)).cross(terrain_normal);
  double sin_theta = axis.norm();
  if (sin_theta > 1e-14) {
    pose_snapped.rotate(AngleAxis<double>(std::asin(sin_theta), axis));
  }
  return pose_snapped;
}

std::vector<InterceptPlan> QPReactiveRecoveryPlan::getInterceptsWithCoP(const FootID &swing_foot, const std::map<FootID, FootState> &foot_states, const Isometry3d &icp, const Isometry3d &cop) {

  Isometry3d T_world_to_local = QPReactiveRecoveryPlan::getTWorldToLocal(icp, cop);
  FootID stance_foot = otherFoot[swing_foot];

  // std::cerr << "reach verts in stance: " << this->biped.reachable_vertices.find(swing_foot)->second << std::endl;
  Matrix<double, 3, 4> reachable_vertices_in_world = foot_states.find(stance_foot)->second.pose * this->biped.reachable_vertices.find(swing_foot)->second;
  // std::cerr << "stance pose: " << foot_states.find(stance_foot)->second.pose.translation() << std::endl;
  // std::cerr << "reach verts in world: " << reachable_vertices_in_world << std::endl;
  // std::cerr << "foot name: " << footIDToName[swing_foot] << std::endl;

  double t_min_to_xprime = QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(foot_states.find(swing_foot)->second, this->biped, T_world_to_local);
  t_min_to_xprime = std::max(t_min_to_xprime, this->min_step_duration);

  double x0 = (T_world_to_local * foot_states.find(swing_foot)->second.pose).translation().x();
  double xd0 = (T_world_to_local.linear() * foot_states.find(swing_foot)->second.velocity.head(3))(0);

  std::vector<InterceptPlan> intercept_plans;

  double x_ic = (T_world_to_local * icp).translation()(0);
  double x_cop = 0; // by the definition of our local frame

  ExponentialForm icp_traj_in_local = QPReactiveRecoveryPlan::icpTrajectory(x_ic, x_cop, this->biped.omega);
  double x_ic_int = icp_traj_in_local.value(t_min_to_xprime) + this->desired_icp_offset;
  // Don't narrow our stance to intercept if possible
  // double x_ic_target = std::max(x_ic_int, x0);
  double x_ic_target = x_ic_int;

  Polynomial<double> x_foot_poly_plus = QPReactiveRecoveryPlan::bangBangPolynomial(x0, xd0, this->biped.u_max);
  Polynomial<double> x_foot_poly_minus = QPReactiveRecoveryPlan::bangBangPolynomial(x0, xd0, -this->biped.u_max);
  Vector2d x_foot_int(x_foot_poly_plus.value(t_min_to_xprime), x_foot_poly_minus.value(t_min_to_xprime));

  if ((x_ic_target >= x_foot_int.minCoeff()) && (x_ic_target <= x_foot_int.maxCoeff())) {
    // std::cerr << "xprime dominates" << std::endl;
    // The time to get onto the xprime axis dominates, and we can hit the ICP (plus offset) as soon as we get to that axis
    std::vector<BangBangIntercept> intercepts = QPReactiveRecoveryPlan::bangBangIntercept(x0, xd0, x_ic_target, this->biped.u_max);
    if (intercepts.size() > 0) {
      // std::cerr << "x_ic_target: " << x_ic_target << std::endl;
      // if there are multiple options, take the one that switches sooner
      std::vector<BangBangIntercept>::iterator it_min = std::min_element(intercepts.begin(), intercepts.end(), tswitchComp);
      Isometry3d intercept_pose_in_world = T_world_to_local.inverse() * Isometry3d(Translation<double, 3>(Vector3d(x_ic_target, 0, 0)));
      // std::cerr << "intercept before reach: " << intercept_pose_in_world.translation() << std::endl;
      intercept_pose_in_world = QPReactiveRecoveryPlan::closestPoseInConvexHull(intercept_pose_in_world, reachable_vertices_in_world.topRows(2));
      // std::cerr << "intercept after reach: " << intercept_pose_in_world.translation() << std::endl;
      InterceptPlan intercept_plan;
      intercept_plan.tf = t_min_to_xprime;
      intercept_plan.tswitch = it_min->tswitch;
      intercept_plan.pose_next = intercept_pose_in_world;
      intercept_plan.icp_next = T_world_to_local.inverse() * Isometry3d(Translation<double, 3>(Vector3d(x_ic_int, 0, 0)));
      intercept_plan.cop = cop;
      intercept_plan.swing_foot = swing_foot;
      intercept_plan.stance_foot = otherFoot[swing_foot];
      intercept_plan.error = 0; // to be filled in later
      intercept_plan.stance_pose = foot_states.at(stance_foot).pose;
      intercept_plans.push_back(intercept_plan);
    }
  } else {
    // std::cerr << "xprime does not dominate" << std::endl;
    std::vector<double> us = {this->biped.u_max, -this->biped.u_max};
    for (std::vector<double>::iterator u = us.begin(); u != us.end(); ++u) {
      std::vector<double> t_int = QPReactiveRecoveryPlan::expIntercept(icp_traj_in_local + this->desired_icp_offset, x0, xd0, *u, 7);
      std::vector<double> t_int_feasible;
      for (std::vector<double>::iterator it = t_int.begin(); it != t_int.end(); ++it) {
        if (*it >= t_min_to_xprime && *it >= std::abs(xd0 / (*u))) {
          t_int_feasible.push_back(*it);
        }
      }
      std::vector<Isometry3d, aligned_allocator<Isometry3d>> reachable_poses;
      if (t_int_feasible.size() == 0) {
        // If there are no intercepts, get as close to our desired capture as possible within the reachable set. 
        // note: this might be off the xcop->xic line
        double x_ic_future = icp_traj_in_local.value(t_min_to_xprime);
        Isometry3d reachable_pose_in_world = T_world_to_local.inverse() * Isometry3d(Translation<double, 3>(Vector3d(x_ic_future + this->desired_icp_offset, 0, 0)));
          // std::cerr << "intercept before reach: " << reachable_pose_in_world.translation() << std::endl;
        reachable_pose_in_world = QPReactiveRecoveryPlan::closestPoseInConvexHull(reachable_pose_in_world, reachable_vertices_in_world.topRows(2));
          // std::cerr << "intercept after reach: " << reachable_pose_in_world.translation() << std::endl;
        reachable_poses.push_back(reachable_pose_in_world);
      } else {
        for (std::vector<double>::iterator t = t_int_feasible.begin(); t != t_int_feasible.end(); ++t) {
          Isometry3d reachable_pose_in_world = T_world_to_local.inverse() * Isometry3d(Translation<double, 3>(Vector3d(icp_traj_in_local.value(*t) + this->desired_icp_offset, 0, 0)));
          // std::cerr << "intercept before reach: " << reachable_pose_in_world.translation() << std::endl;
          reachable_pose_in_world = QPReactiveRecoveryPlan::closestPoseInConvexHull(reachable_pose_in_world, reachable_vertices_in_world.topRows(2));
          // std::cerr << "intercept after reach: " << reachable_pose_in_world.translation() << std::endl;
          reachable_poses.push_back(reachable_pose_in_world);
        }
      }
      for (std::vector<Isometry3d, aligned_allocator<Isometry3d>>::iterator reachable_pose = reachable_poses.begin(); reachable_pose != reachable_poses.end(); ++ reachable_pose) {
        Isometry3d reachable_pose_in_local = T_world_to_local * (*reachable_pose);

        std::vector<BangBangIntercept> intercepts = QPReactiveRecoveryPlan::bangBangIntercept(x0, xd0, reachable_pose_in_local.translation().x(), this->biped.u_max);
        if (intercepts.size() > 0) {
          // if there are multiple options, take the one that switches sooner
          std::vector<BangBangIntercept>::iterator it_min = std::min_element(intercepts.begin(), intercepts.end(), tswitchComp);

          InterceptPlan intercept_plan;
          intercept_plan.tf = std::max(it_min->tf, t_min_to_xprime);
          intercept_plan.tswitch = it_min->tswitch;
          intercept_plan.pose_next = *reachable_pose;
          intercept_plan.icp_next = T_world_to_local.inverse() * Isometry3d(Translation<double, 3>(Vector3d(icp_traj_in_local.value(it_min->tf), 0, 0)));
          intercept_plan.cop = cop;
          intercept_plan.swing_foot = swing_foot;
          intercept_plan.stance_foot = otherFoot[swing_foot];
          intercept_plan.error = 0; // to be filled in later
          intercept_plan.stance_pose = foot_states.at(stance_foot).pose;
          intercept_plans.push_back(intercept_plan);
        }
      }
    }
  }

  for (std::vector<InterceptPlan>::iterator it = intercept_plans.begin(); it != intercept_plans.end(); ++it) {
    it->pose_next.linear() = foot_states.find(stance_foot)->second.pose.linear();
    it->pose_next = snapToTerrain(it->pose_next, foot_states.find(stance_foot)->second.terrain_height, foot_states.find(stance_foot)->second.terrain_normal);
    it->error = interceptPlanError(*it, foot_states, this->biped);
  }

  return intercept_plans;
}

std::vector<InterceptPlan> QPReactiveRecoveryPlan::getInterceptPlansForFoot(const FootID &swing_foot, const std::map<FootID, FootState> &foot_states, const Isometry3d &icp) {
  FootID stance_foot = otherFoot.find(swing_foot)->second;

  // Find the center of pressure, which we'll place as close as possible to the ICP
  Isometry3d cop = QPReactiveRecoveryPlan::closestPoseInConvexHull(icp, 
                       (foot_states.find(stance_foot)->second.pose * (this->foot_hull_cop_shrink_factor * this->biped.foot_vertices.find(stance_foot)->second)).topRows(2));
  return this->getInterceptsWithCoP(swing_foot, foot_states, icp, cop);
}

std::vector<InterceptPlan> QPReactiveRecoveryPlan::getInterceptPlans(const std::map<FootID, FootState> &foot_states, const Isometry3d &icp) {
  std::vector<InterceptPlan> all_intercept_plans;
  std::vector<FootID> available_swing_feet;

  if (foot_states.find(RIGHT)->second.contact && foot_states.find(LEFT)->second.contact) {
    available_swing_feet.push_back(LEFT);
    available_swing_feet.push_back(RIGHT);
  } else if (!foot_states.find(RIGHT)->second.contact) {
    available_swing_feet.push_back(RIGHT);
  } else {
    available_swing_feet.push_back(LEFT);
  }

  for (std::vector<FootID>::iterator swing_foot = available_swing_feet.begin(); swing_foot != available_swing_feet.end(); ++swing_foot) {
    if (foot_states.find(*swing_foot)->second.velocity.head(3).squaredNorm() / this->biped.u_max / 2.0 < this->max_considerable_foot_swing) {
      std::vector<InterceptPlan> foot_plans = this->getInterceptPlansForFoot(*swing_foot, foot_states, icp);
      all_intercept_plans.insert(all_intercept_plans.end(), foot_plans.begin(), foot_plans.end());
    }
  }
  return all_intercept_plans;
}

std::unique_ptr<PiecewisePolynomial<double>> QPReactiveRecoveryPlan::freeKnotTimesSpline(double t0, double tf, const Ref<const MatrixXd> &xs, const Ref<const VectorXd> xd0, const Ref<const VectorXd> xdf) {
  const int grid_steps = 10;

  const size_t num_segments = xs.cols() - 1;
  const size_t ndof = xs.rows();
  const size_t num_knots = num_segments - 1;

  if (xs.rows() != xd0.rows() || xs.rows() != xdf.rows()) {
    throw std::runtime_error("size of xs and xd0 (or xdf) don't match");
  }

  std::vector<double> segment_times;
  segment_times.resize(num_segments + 1);
  segment_times[0] = t0;
  segment_times[static_cast<size_t>(num_segments)] = tf;
  std::vector<double> best_segment_times = segment_times;
  double t_step = (tf - t0) / grid_steps;
  double min_objective_value = std::numeric_limits<double>::infinity();

  // assemble the knot point locations for input to nWaypointCubicSpline
  MatrixXd xi = xs.block(0, 1, ndof, num_knots);

  int t_indices[num_knots];
  if (grid_steps <= num_knots){
    // If we have have too few grid steps, then by pigeonhole it's
    // impossible to give each a unique time in our grid search.
    throw std::runtime_error("Drake:freeKnotTimesSpline:TooManyKnotsForNumGridSteps");
  }
  for (int i=0; i<num_knots; i++)
    t_indices[i] = i+1; // assume knot point won't be the same time as the
          // initial state, or previous knot point
 
  while (t_indices[0] < grid_steps-num_knots+1){
    for (int i=0; i<num_knots; i++)
      segment_times[i+1] = t0 + t_indices[i]*t_step;

    bool valid_solution = true;
    double objective_value = 0.0;
    for (int dof = 0; dof < ndof && valid_solution; dof++) {
      try {
        PiecewisePolynomial<double> spline = nWaypointCubicSpline(segment_times, xs(dof, 0), xd0(dof), xs(dof, num_segments), xdf(dof), xi.row(dof).transpose());
        PiecewisePolynomial<double> acceleration_squared = spline.derivative(2);
        acceleration_squared *= acceleration_squared;
        PiecewisePolynomial<double> acceleration_squared_integral = acceleration_squared.integral();
        objective_value += acceleration_squared_integral.scalarValue(spline.getEndTime()) - acceleration_squared_integral.scalarValue(spline.getStartTime());
      }
      catch (ConstraintMatrixSingularError&) {
        valid_solution = false;
      }
    }

    if (valid_solution && objective_value < min_objective_value) {
      best_segment_times = segment_times;
      min_objective_value = objective_value;
    }

    // Advance grid search counter or terminate, counting from
    // the latest t_index, and on overflow carrying to the
    // next lowest t_index and resetting to the new value of that
    // next lowest t_index. (since times must always be in order!)
    t_indices[num_knots-1]++;
    // carry, except for the lowest place, which we 
    // use to detect doneness.
    for (int i=num_knots-1; i>0; i--){
      if ((i==num_knots-1 && t_indices[i] >= grid_steps) || (i<num_knots-1 && t_indices[i] >= t_indices[i+1])){
        t_indices[i-1]++;
        t_indices[i] = t_indices[i-1]+1;
      }
    }
  }

  std::vector<Matrix<Polynomial<double>, Dynamic, Dynamic>> poly_matrix;
  poly_matrix.reserve(num_segments);
  for (int i=0; i < num_segments; ++i) {
    poly_matrix.push_back(Matrix<Polynomial<double>, Dynamic, Dynamic> (ndof, 1));
  }
  for (int dof=0; dof < ndof; ++dof) {
    PiecewisePolynomial<double> one_d_spline = nWaypointCubicSpline(best_segment_times, xs(dof, 0), xd0(dof), xs(dof, num_segments), xdf(dof), xi.row(dof).transpose());
    for (int i=0; i < num_segments; ++i) {
      poly_matrix[i](dof) = one_d_spline.getPolynomial(i);
    }
  }
  std::unique_ptr<PiecewisePolynomial<double>> spline(new PiecewisePolynomial<double>(poly_matrix, best_segment_times));
  return spline;
}

std::unique_ptr<PiecewisePolynomial<double>> QPReactiveRecoveryPlan::straightToGoalTrajectory(double t_global, const InterceptPlan &intercept_plan, const FootStateMap &foot_states) {
  const FootState state = foot_states.at(intercept_plan.swing_foot);

  std::cout << "case 1" << std::endl;

  const double fraction_first = 0.7;

  const double swing_height_first_in_world = state.terrain_height + (state.pose.translation().z() - state.terrain_height) * (1 - std::pow(fraction_first,2));

  Matrix<double, 6, 3> xs;
  Vector6d xd0 = Vector6d::Zero(); // don't try to continue current velocity (it just leads to unpredictable and weird splines)
  Vector6d xdf = Vector6d::Zero();

  Quaterniond quat;
  xs.block(0, 0, 3, 1) = state.pose.translation();
  quat = Quaterniond(state.pose.rotation());
  auto w = quat2expmap(Drake::initializeAutoDiff(Vector4d(quat.w(), quat.x(), quat.y(), quat.z())));
  xs.block(3, 0, 3, 1) = autoDiffToGradientMatrix(w);

  xs.block(0, 2, 3, 1) = intercept_plan.pose_next.translation();
  quat = Quaterniond(intercept_plan.pose_next.rotation());
  xs.block(3, 2, 3, 1) = quat2expmap(Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));

  auto w_unwrap = closestExpmap(Drake::initializeAutoDiff(xs.block<3, 1>(3, 0)), Drake::initializeAutoDiff(xs.block<3, 1>(3, 2)));
  xs.block(3, 2, 3, 1) = autoDiffToValueMatrix(w_unwrap);
  xd0.tail<3>() = autoDiffToGradientMatrix(w_unwrap) * xd0.tail<3>();

  xs.block(0, 1, 6, 1) = (1 - fraction_first) * xs.block(0, 0, 6, 1) + fraction_first * xs.block(0, 2, 6, 1);
  xs(2, 1) = swing_height_first_in_world;

  return QPReactiveRecoveryPlan::freeKnotTimesSpline(t_global, intercept_plan.tf + t_global, xs, xd0, xdf);
}

std::unique_ptr<PiecewisePolynomial<double>> QPReactiveRecoveryPlan::upOverAndDownTrajectory(double t_global, const InterceptPlan &intercept_plan, const FootStateMap &foot_states) {
  const FootState state = foot_states.at(intercept_plan.swing_foot);

  std::cout << "case 2" << std::endl;

  const double fraction_first = 0.15;
  const double fraction_second = 1 - fraction_first;

  double swing_height_first_in_world = state.terrain_height + this->swing_height_above_terrain;
  double swing_height_second_in_world = state.terrain_height + this->swing_height_above_terrain;

  if (state.pose.translation().z() > swing_height_first_in_world) {
    swing_height_first_in_world = swing_height_second_in_world * (fraction_first / fraction_second) + state.pose.translation().z() * (1 - fraction_first / fraction_second);
  }

  Matrix<double, 6, 4> xs;
  Vector6d xd0 = Vector6d::Zero(); // don't try to continue current velocity (it just leads to unpredictable and weird splines)
  Vector6d xdf = Vector6d::Zero();

  Quaterniond quat;
  xs.block(0, 0, 3, 1) = state.pose.translation();
  quat = Quaterniond(state.pose.rotation());
  auto w = quat2expmap(Drake::initializeAutoDiff(Vector4d(quat.w(), quat.x(), quat.y(), quat.z())));
  xs.block(3, 0, 3, 1) = autoDiffToGradientMatrix(w);

  xs.block(0, 3, 3, 1) = intercept_plan.pose_next.translation();
  quat = Quaterniond(intercept_plan.pose_next.rotation());
  xs.block(3, 3, 3, 1) = quat2expmap(Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));

  auto w_unwrap = closestExpmap(Drake::initializeAutoDiff(xs.block<3, 1>(3, 0)), Drake::initializeAutoDiff(xs.block<3,1>(3, 3)));
  xs.block(3, 3, 3, 1) = autoDiffToValueMatrix(w_unwrap);
  xd0.tail<3>() = autoDiffToGradientMatrix(w_unwrap) * xd0.tail<3>();

  xs.block(0, 1, 2, 1) = (1 - fraction_first) * xs.block(0, 0, 2, 1) + fraction_first * xs.block(0, 3, 2, 1);
  xs(2, 1) = swing_height_first_in_world;
  xs.block(3, 1, 3, 1) = xs.block(3, 0, 3, 1);

  xs.block(0, 2, 2, 1) = (1 - fraction_second) * xs.block(0, 0, 2, 1) + fraction_second * xs.block(0, 3, 2, 1);
  xs(2, 2) = swing_height_second_in_world;
  xs.block(3, 2, 3, 1) = xs.block(3, 3, 3, 1);

  return QPReactiveRecoveryPlan::freeKnotTimesSpline(t_global, t_global + intercept_plan.tf, xs, xd0, xdf);
}

std::unique_ptr<PiecewisePolynomial<double>> QPReactiveRecoveryPlan::swingTrajectory(double t_global, const InterceptPlan &intercept_plan, const std::map<FootID, FootState> &foot_states) {
  const FootState state = foot_states.at(intercept_plan.swing_foot);
  const double dist_to_goal = (intercept_plan.pose_next.translation().head(2) - state.pose.translation().head(2)).norm();
  // TODO: name the magic numbers here
  const double descend_coeff = std::pow(1.0 / 0.15, 2);

  std::cout << "planning swing with tf = " << intercept_plan.tf << std::endl;
  if (descend_coeff * std::pow(state.pose.translation().z() - state.terrain_height, 2) >= dist_to_goal) {
    // We're within a quadratic bowl around our target, so let's just descend straight there
    return this->straightToGoalTrajectory(t_global, intercept_plan, foot_states);
  } else {
    // We'll need to go up and then back down to get to the goal
    return this->upOverAndDownTrajectory(t_global, intercept_plan, foot_states);
  }
}

void QPReactiveRecoveryPlan::setRobot(RigidBodyTree *robot) {
  this->robot = robot;
  this->findFootSoleFrames();
  this->q_des.resize(robot->num_positions);
  this->bodyOrFrameNameToIdMap = computeBodyOrFrameNameToIdMap(*robot);
}

QPReactiveRecoveryPlan::QPReactiveRecoveryPlan(RigidBodyTree *robot, const RobotPropertyCache &rpc) {
  this->robot = robot;
  this->biped = getAtlasDefaults();
  this->robot_property_cache = rpc;
  if (this->robot) {
    this->setRobot(robot);
  }
  this->initLCM();
}

QPReactiveRecoveryPlan::QPReactiveRecoveryPlan(RigidBodyTree *robot, const RobotPropertyCache &rpc, BipedDescription biped) {
  this->robot = robot;
  this->biped = biped;
  this->robot_property_cache = rpc;
  if (this->robot) {
    this->setRobot(robot);
  }
  this->initLCM();
}

void QPReactiveRecoveryPlan::findFootSoleFrames() {
  std::map<FootID, bool> has_frame;
  has_frame[RIGHT] = false;
  has_frame[LEFT] = false;
  for (int i=0; i < robot->frames.size(); ++i) {
    if (this->robot->frames[i]->name == "r_foot_sole") {
      has_frame[RIGHT] = true;
      // frame_ind0 = -frameID - 2
      // i = -frameID - 2;
      // frameID = -i - 2;
      this->foot_frame_ids[RIGHT] = -i - 2;
      Isometry3d Tframe;
      int body_id = this->robot->parseBodyOrFrameID( this->foot_frame_ids[RIGHT], &Tframe);
      if (!Tframe.isApprox(this->robot->frames[i]->transform_to_body)) {
        throw std::runtime_error("somehow I got the frame ID/index logic wrong");
      }
      this->foot_body_ids[RIGHT] = body_id;
    } else if (this->robot->frames[i]->name == "l_foot_sole") {
      has_frame[LEFT] = true;
      this->foot_frame_ids[LEFT] = -i - 2;
      Isometry3d Tframe;
      int body_id = this->robot->parseBodyOrFrameID(this->foot_frame_ids[LEFT], &Tframe);
      if (!Tframe.isApprox(this->robot->frames[i]->transform_to_body)) {
        throw std::runtime_error("somehow I got the frame ID/index logic wrong");
      }
      this->foot_body_ids[LEFT] = body_id;
    }
  }

  if (!has_frame[RIGHT]) {
    throw std::runtime_error("could not find r_foot_sole frame");
  }
  if (!has_frame[LEFT]) {
    throw std::runtime_error("could not find l_foot_sole frame");
  }
}

void QPReactiveRecoveryPlan::resetInitialization() {
  this->initialized = false;
  this->last_swing_plan.reset(NULL);
}

drake::lcmt_qp_controller_input QPReactiveRecoveryPlan::getQPControllerInput(double t_global, const VectorXd &q, const VectorXd &v, const std::vector<bool>& contact_force_detected) {
  if (!this->initialized) {
    for (int i=0; i < this->robot_property_cache.position_indices.arms[Side::LEFT].size(); ++i) {
      int j = this->robot_property_cache.position_indices.arms[Side::LEFT][i];
      this->q_des(j) = q(j);
    }
    for (int i=0; i < this->robot_property_cache.position_indices.arms[Side::RIGHT].size(); ++i) {
      int j = this->robot_property_cache.position_indices.arms[Side::RIGHT][i];
      this->q_des(j) = q(j);
    }
    this->initialized = true;
    this->t_start = t_global;
  }

  KinematicsCache<double> cache = this->robot->doKinematics(q, v);


  Vector2d r_ic = this->getICP(cache, v);
  Isometry3d icp = Isometry3d(Translation<double, 3>(Vector3d(r_ic(0), r_ic(1), 0)));

  FootStateMap foot_states = this->getFootStates(cache, v, contact_force_detected);

  bool is_captured = this->isICPCaptured(icp.translation().head<2>(), foot_states, this->biped.foot_vertices);

  drake::lcmt_qp_controller_input qp_input;
  this->setupQPInputDefaults(t_global, qp_input);

  if (this->last_swing_plan && t_global < this->last_swing_plan->getEndTime()) {
    // std::cout << "continuing current plan" << std::endl;
    this->getInterceptInput(t_global, foot_states, qp_input);
  } else if (is_captured) {
    // std::cout << "is captured" << std::endl;
    this->getCaptureInput(t_global, foot_states, icp, qp_input);
  } else if (this->last_swing_plan && t_global < this->last_swing_plan->getEndTime() + this->post_execution_delay) {
    // std::cout << "in delay after plan end" << std::endl;
    this->getCaptureInput(t_global, foot_states, icp, qp_input);
  } else {
    std::cout << "replanning" << std::endl;
    std::vector<InterceptPlan> intercept_plans = this->getInterceptPlans(foot_states, icp);
    if (intercept_plans.size() == 0) {
      std::cout << "recovery is not possible" << std::endl;
      this->getCaptureInput(t_global, foot_states, icp, qp_input);
    } else {
      std::vector<InterceptPlan>::iterator best_plan = std::min_element(intercept_plans.begin(), intercept_plans.end(), errorCompare);
      this->last_intercept_plan = *best_plan;
      this->last_swing_plan.reset(this->swingTrajectory(t_global, this->last_intercept_plan, foot_states).release());
      this->t_start = t_global;
      this->getInterceptInput(t_global, foot_states, qp_input);
    }
  }

  this->publishForVisualization(cache, t_global, icp);
  verifySubtypeSizes(qp_input);
  return qp_input;
}

void QPReactiveRecoveryPlan::publishQPControllerInput(double t_global, const VectorXd &q, const VectorXd &v, const std::vector<bool>& contact_force_detected) {
  drake::lcmt_qp_controller_input qp_input = this->getQPControllerInput(t_global, q, v, contact_force_detected);
  this->LCMHandle->publish("QP_CONTROLLER_INPUT", &qp_input);
}

void QPReactiveRecoveryPlan::setupQPInputDefaults(double t_global, drake::lcmt_qp_controller_input &qp_input) {
  qp_input.be_silent = false;
  qp_input.timestamp = static_cast<int64_t> (t_global * 1e6);
  qp_input.num_support_data = 0;
  qp_input.num_tracked_bodies = 0;
  qp_input.num_external_wrenches = 0;
  qp_input.num_joint_pd_overrides = 0;

  qp_input.zmp_data.timestamp = 0;

  Matrix4d A = Matrix4d::Zero();
  A.block(0, 2, 2, 2) = Matrix2d::Identity();
  eigenToCArrayOfArrays(A, qp_input.zmp_data.A);

  Matrix<double, 4, 2> B = Matrix<double, 4, 2>::Zero();
  B.block(2, 0, 2, 2) = Matrix2d::Identity();
  eigenToCArrayOfArrays(B, qp_input.zmp_data.B);

  Matrix<double, 2, 4> C = Matrix<double, 2, 4>::Zero();
  C.block(0, 0, 2, 2) = Matrix2d::Identity();
  eigenToCArrayOfArrays(C, qp_input.zmp_data.C);

  Matrix2d D = Matrix2d::Identity();
  D *= -(1.0 / pow(this->biped.omega, 2));
  eigenToCArrayOfArrays(D, qp_input.zmp_data.D);

  // x0 and y0 will be filled in from the plan

  Vector2d u0 = Vector2d::Zero();
  eigenToCArrayOfArrays(u0, qp_input.zmp_data.u0);

  Matrix2d R = Matrix2d::Zero();
  eigenToCArrayOfArrays(R, qp_input.zmp_data.R);

  Matrix2d Qy = Matrix2d::Identity();
  Qy *= 0.8;
  eigenToCArrayOfArrays(Qy, qp_input.zmp_data.Qy);

  eigenToCArrayOfArrays(this->S, qp_input.zmp_data.S);

  Vector4d s1 = Vector4d::Zero();
  eigenToCArrayOfArrays(s1, qp_input.zmp_data.s1);

  Vector4d s1dot = Vector4d::Zero();
  eigenToCArrayOfArrays(s1dot, qp_input.zmp_data.s1dot);

  qp_input.zmp_data.s2 = 0;
  qp_input.zmp_data.s2dot = 0;

  qp_input.whole_body_data.num_positions = this->robot->num_positions;
  PiecewisePolynomial<double> qdesPolynomial = PiecewisePolynomial<double>(this->q_des); // create a constant PiecewisePolynomial

  drake::lcmt_piecewise_polynomial qtrajSplineMsg;
  encodePiecewisePolynomial(qdesPolynomial, qtrajSplineMsg);
  qp_input.whole_body_data.spline = qtrajSplineMsg;


  for (int i=0; i < this->robot_property_cache.position_indices.arms[Side::LEFT].size(); i++) {
    qp_input.whole_body_data.constrained_dofs.push_back(this->robot_property_cache.position_indices.arms[Side::LEFT][i] + 1);
  }
  for (int i=0; i < this->robot_property_cache.position_indices.arms[Side::RIGHT].size(); i++) {
    qp_input.whole_body_data.constrained_dofs.push_back(this->robot_property_cache.position_indices.arms[Side::RIGHT][i] + 1);
  }
  for (int i=0; i < this->robot_property_cache.position_indices.neck.size(); i++) {
    qp_input.whole_body_data.constrained_dofs.push_back(this->robot_property_cache.position_indices.neck[i] + 1);
  }
  qp_input.whole_body_data.constrained_dofs.push_back(this->robot_property_cache.position_indices.back_bky + 1);
  qp_input.whole_body_data.constrained_dofs.push_back(this->robot_property_cache.position_indices.back_bkz + 1);

  qp_input.whole_body_data.num_constrained_dofs = qp_input.whole_body_data.constrained_dofs.size();

  qp_input.param_set_name = "recovery";

}


void QPReactiveRecoveryPlan::publishForVisualization(KinematicsCache<double>& cache, double t_global, const Isometry3d &icp) {
  std::shared_ptr<drc::reactive_recovery_debug_t> msg(new drc::reactive_recovery_debug_t());

  msg->utime = static_cast<int64_t> (t_global * 1e6);

  auto com = this->robot->centerOfMass<double>(cache);
  memcpy(msg->com, com.data(), 3*sizeof(double));
  memcpy(msg->icp, icp.translation().head<2>().data(), 2*sizeof(double));

  msg->num_spline_ts = 0;
  msg->num_spline_segments = 0;

  this->LCMHandle->publish("REACTIVE_RECOVERY_DEBUG", msg.get());
}
/*
Matrix3Xd QPReactiveRecoveryPlan::heelToeContacts(int body_id) {
  // no longer works due to changes to robot property cache
  Matrix3Xd toe_contacts = this->robot_property_cache.contact_groups[body_id].at("toe");
  Matrix3Xd heel_contacts = this->robot_property_cache.contact_groups[body_id].at("heel");
  Matrix3Xd all_contacts(3, toe_contacts.cols() + heel_contacts.cols());
  all_contacts.block(0, 0, 3, toe_contacts.cols()) = toe_contacts;
  all_contacts.block(0, toe_contacts.cols(), 3, heel_contacts.cols()) = heel_contacts;
  return all_contacts;
}
*/

std::map<SupportLogicType, std::vector<bool>> createSupportLogicMaps() {
  std::map<SupportLogicType, std::vector<bool> > ret;
  ret[REQUIRE_SUPPORT] = { {true, true, true, true} };
  ret[ONLY_IF_FORCE_SENSED] = { {false, false, true, true} };
  ret[KINEMATIC_OR_SENSED] = { {false, true, true, true} };
  ret[PREVENT_SUPPORT] = { {false, false, false, false} };
  return ret;
}

void QPReactiveRecoveryPlan::encodeSupportData(const int body_id, const FootState &foot_state, const SupportLogicType &support_logic, drake::lcmt_support_data &support_data) {
  support_data.timestamp = 0;
  support_data.body_name = this->robot->getBodyOrFrameName(body_id);
  support_data.contact_pts.resize(3);
  Matrix3Xd all_contacts = this->robot->bodies[body_id]->contact_pts;
  support_data.num_contact_pts = all_contacts.cols();
  for (int i=0; i < 3; i++) {
    support_data.contact_pts[i].resize(all_contacts.cols());
    for (int j=0; j < all_contacts.cols(); j++) {
      support_data.contact_pts[i][j] = all_contacts(i, j);
    }
  }
  std::map<SupportLogicType, std::vector<bool>> logic_maps = createSupportLogicMaps();
  std::vector<bool> logic = logic_maps.at(support_logic);
  for (int i=0; i < 4; ++i) {
    support_data.support_logic_map[i] = logic[i];
  }
  support_data.mu = this->mu;
  support_data.use_support_surface = true;
  for (int i=0; i < 3; ++i) {
    // 4-vector describing a support surface: [v; b] such that v' * [x;y;z] + b == 0
    // we have normal n and height h at x0,y0
    // n' * [x0;y0;h] + b == 0
    // b = -n' * [x0;y0;h]
    support_data.support_surface[i] = static_cast<float>(foot_state.terrain_normal(i));
  }
  support_data.support_surface[3] = static_cast<float> (-1 * foot_state.terrain_normal.transpose() * Vector3d(foot_state.pose.translation().x(), foot_state.pose.translation().y(), foot_state.terrain_height));
}

void QPReactiveRecoveryPlan::encodeBodyMotionData(int body_or_frame_id, PiecewisePolynomial<double> spline, drake::lcmt_body_motion_data &body_motion) {
  body_motion.timestamp = 0;
  body_motion.body_or_frame_name = this->robot->getBodyOrFrameName(body_or_frame_id);
  encodePiecewisePolynomial(spline, body_motion.spline);
  body_motion.in_floating_base_nullspace = false;
  body_motion.control_pose_when_in_contact = false;
  body_motion.quat_task_to_world[0] = 1;
  body_motion.quat_task_to_world[1] = 0;
  body_motion.quat_task_to_world[2] = 0;
  body_motion.quat_task_to_world[3] = 0;
  body_motion.translation_task_to_world[0] = 0;
  body_motion.translation_task_to_world[1] = 0;
  body_motion.translation_task_to_world[2] = 0;
  body_motion.xyz_kp_multiplier[0] = 1;
  body_motion.xyz_kp_multiplier[1] = 1;
  body_motion.xyz_kp_multiplier[2] = 1;
  body_motion.xyz_damping_ratio_multiplier[0] = 1;
  body_motion.xyz_damping_ratio_multiplier[1] = 1;
  body_motion.xyz_damping_ratio_multiplier[2] = 1;
  body_motion.expmap_kp_multiplier = 1;
  body_motion.expmap_damping_ratio_multiplier = 1;
  body_motion.weight_multiplier[0] = 1;
  body_motion.weight_multiplier[1] = 1;
  body_motion.weight_multiplier[2] = 1;
  body_motion.weight_multiplier[3] = 1;
  body_motion.weight_multiplier[4] = 1;
  body_motion.weight_multiplier[5] = 1;
}

double angleAverage(double theta1, double theta2) {
  // (Copied from drakeUtil.cpp to avoid a lot of extra dependencies)
  //
  // Computes the average between two angles by averaging points on the unit
  // circle and taking the arctan of the result.
  //   see: http://en.wikipedia.org/wiki/Mean_of_circular_quantities
  // theta1 is a scalar or column vector of angles (rad)
  // theta2 is a scalar or column vector of angles (rad)

  double x_mean = 0.5 * (std::cos(theta1) + std::cos(theta2));
  double y_mean = 0.5 * (std::sin(theta1) + std::sin(theta2));

  double angle_mean = atan2(y_mean, x_mean);

  return angle_mean;
}

PiecewisePolynomial<double> constantPoseCubicSpline(const Isometry3d &pose) {
  std::vector<Matrix<Polynomial<double>, Dynamic, Dynamic>> poly_matrix;
  poly_matrix.push_back(Matrix<Polynomial<double>, Dynamic, Dynamic>(6, 1));
  std::vector<double> ts = {0, 0};
  Matrix<double, 0, 1> xi;
  Vector6d xyzexp;
  xyzexp.head<3>() = pose.translation().head<3>();
  Quaterniond quat = Quaterniond(pose.rotation());
  xyzexp.tail<3>() = quat2expmap(Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
  for (int i=0; i < 6; ++i) {
    poly_matrix[0](i) = Polynomial<double>(Vector4d(xyzexp(i), 0, 0, 0));
  }
  return PiecewisePolynomial<double>(poly_matrix, ts);
}

void QPReactiveRecoveryPlan::getInterceptInput(double t_global, const FootStateMap &foot_states, drake::lcmt_qp_controller_input &qp_input) {

  PiecewisePolynomial<double>::CoefficientMatrix plan_shift(6, 1);
  plan_shift.topRows<3>() = this->last_intercept_plan.stance_pose.translation() - foot_states.at(this->last_intercept_plan.stance_foot).pose.translation();
  plan_shift.bottomRows<3>().setZero();
  // plan_shift = desired - measured;
  // measured = desired - plan_shift

  Vector4d x0 = Vector4d::Zero();
  x0.head<2>() = 0.5 * this->last_intercept_plan.stance_pose.translation().head<2>() + 0.5 * this->last_intercept_plan.pose_next.translation().head<2>() - plan_shift.topRows<2>();
  eigenToCArrayOfArrays(x0, qp_input.zmp_data.x0);
  eigenToCArrayOfArrays(this->last_intercept_plan.cop.translation().head<2>() - plan_shift.topRows<2>(), qp_input.zmp_data.y0);

  drake::lcmt_support_data support_data_stance;
  int stance_foot_id = this->foot_body_ids.at(this->last_intercept_plan.stance_foot);
  this->encodeSupportData(stance_foot_id, foot_states.at(this->last_intercept_plan.stance_foot), 
                          REQUIRE_SUPPORT, support_data_stance);
  qp_input.support_data.push_back(support_data_stance);

  drake::lcmt_support_data support_data_swing;
  if (t_global - this->t_start <= (this->last_swing_plan->getEndTime() - this->last_swing_plan->getStartTime()) / 2) {
    this->encodeSupportData(this->foot_body_ids.at(this->last_intercept_plan.swing_foot),
                            foot_states.at(this->last_intercept_plan.swing_foot),
                            PREVENT_SUPPORT, support_data_swing);
  } else {
    this->encodeSupportData(this->foot_body_ids.at(this->last_intercept_plan.swing_foot),
                            foot_states.at(this->last_intercept_plan.swing_foot),
                            ONLY_IF_FORCE_SENSED, support_data_swing);
  }
  qp_input.support_data.push_back(support_data_swing);
  qp_input.num_support_data = qp_input.support_data.size();

  drake::lcmt_body_motion_data body_motion;
  this->encodeBodyMotionData(this->foot_frame_ids.at(this->last_intercept_plan.swing_foot),
                             *this->last_swing_plan - plan_shift, body_motion);
  body_motion.in_floating_base_nullspace = true;
  qp_input.body_motion_data.push_back(body_motion);

  double pelvis_height = this->last_intercept_plan.stance_pose.translation().z() + this->pelvis_height_above_sole;
  // double pelvis_height = foot_states.at(this->last_intercept_plan.stance_foot).terrain_height + this->pelvis_height_above_sole;
  Quaterniond rfoot_quat = Quaterniond(foot_states.at(RIGHT).pose.rotation());
  Quaterniond lfoot_quat = Quaterniond(foot_states.at(LEFT).pose.rotation());
  double pelvis_yaw = angleAverage(quat2rpy(Vector4d(rfoot_quat.w(), rfoot_quat.x(), rfoot_quat.y(), rfoot_quat.z()))(2), 
                                quat2rpy(Vector4d(lfoot_quat.w(), lfoot_quat.x(), lfoot_quat.y(), lfoot_quat.z()))(2));
  Isometry3d pelvis_pose = Isometry3d(Translation<double, 3>(Vector3d(0, 0, pelvis_height)));
  pelvis_pose.rotate(AngleAxis<double>(pelvis_yaw, Vector3d(0, 0, 1)));
  this->encodeBodyMotionData(this->bodyOrFrameNameToIdMap["pelvis"],
                             constantPoseCubicSpline(pelvis_pose) - plan_shift,
                             body_motion);
  body_motion.weight_multiplier[3] = 0; // don't try to control x and y
  body_motion.weight_multiplier[4] = 0;
  qp_input.body_motion_data.push_back(body_motion);
  qp_input.num_tracked_bodies = qp_input.body_motion_data.size();
}

void QPReactiveRecoveryPlan::getCaptureInput(double t_global, const FootStateMap &foot_states, const Isometry3d &icp, drake::lcmt_qp_controller_input &qp_input) {

  Vector4d x0 = Vector4d::Zero();
  x0.head<2>() = 0.5 * foot_states.at(RIGHT).pose.translation().head<2>() + 0.5 * foot_states.at(LEFT).pose.translation().head<2>();
  eigenToCArrayOfArrays(x0, qp_input.zmp_data.x0);
  eigenToCArrayOfArrays(x0.head<2>(), qp_input.zmp_data.y0);

  std::vector<FootID> foot_ids = {RIGHT, LEFT};
  for (std::vector<FootID>::iterator foot = foot_ids.begin(); foot != foot_ids.end(); ++foot) {
    drake::lcmt_support_data support_data;
    drake::lcmt_body_motion_data body_motion;
    if ((foot_states.at(*foot).pose.translation().z() - foot_states.at(*foot).terrain_height) < this->capture_max_flyfoot_height) {
      this->encodeSupportData(this->foot_body_ids.at(*foot), foot_states.at(*foot),
                              REQUIRE_SUPPORT, support_data);
    } else {
      this->encodeSupportData(this->foot_body_ids.at(*foot), foot_states.at(*foot),
                              ONLY_IF_FORCE_SENSED, support_data);
    }
    qp_input.support_data.push_back(support_data);

    Isometry3d pose_on_terrain = snapToTerrain(foot_states.at(*foot).pose, foot_states.at(*foot).terrain_height, foot_states.at(*foot).terrain_normal);
    this->encodeBodyMotionData(this->foot_frame_ids.at(*foot), 
                               constantPoseCubicSpline(pose_on_terrain),
                               body_motion);
    body_motion.in_floating_base_nullspace = true;
    qp_input.body_motion_data.push_back(body_motion);
  }
  qp_input.num_support_data = qp_input.support_data.size();

  drake::lcmt_body_motion_data body_motion;
  double pelvis_height = 0.5 * (foot_states.at(LEFT).terrain_height + foot_states.at(RIGHT).terrain_height) + this->pelvis_height_above_sole;
  Quaterniond rfoot_quat = Quaterniond(foot_states.at(RIGHT).pose.rotation());
  Quaterniond lfoot_quat = Quaterniond(foot_states.at(LEFT).pose.rotation());
  double pelvis_yaw = angleAverage(quat2rpy(Vector4d(rfoot_quat.w(), rfoot_quat.x(), rfoot_quat.y(), rfoot_quat.z()))(2), 
                                quat2rpy(Vector4d(lfoot_quat.w(), lfoot_quat.x(), lfoot_quat.y(), lfoot_quat.z()))(2));
  Isometry3d pelvis_pose = Isometry3d(Translation<double, 3>(Vector3d(0, 0, pelvis_height)));
  pelvis_pose.rotate(AngleAxis<double>(pelvis_yaw, Vector3d(0, 0, 1)));
  this->encodeBodyMotionData(this->bodyOrFrameNameToIdMap["pelvis"],
                             constantPoseCubicSpline(pelvis_pose),
                             body_motion);
  body_motion.weight_multiplier[3] = 0; // don't try to control x and y
  body_motion.weight_multiplier[4] = 0;
  qp_input.body_motion_data.push_back(body_motion);
  qp_input.num_tracked_bodies = qp_input.body_motion_data.size();
}

Vector2d QPReactiveRecoveryPlan::getICP(KinematicsCache<double>& cache, const VectorXd &v) {
  Vector3d com_position = this->robot->centerOfMass(cache);
  Vector3d com_velocity = this->robot->centerOfMassJacobian(cache) * v;
  Vector2d icp = com_position.head(2) + com_velocity.head(2) / this->biped.omega;
  return icp;
}

FootStateMap QPReactiveRecoveryPlan::getFootStates(const KinematicsCache<double>& cache, const VectorXd &v, const std::vector<bool>& contact_force_detected) {
  std::vector<FootID> foot_ids = {RIGHT, LEFT};
  FootStateMap foot_states;
  double min_foot_height = std::numeric_limits<double>::infinity();
  for (std::vector<FootID>::iterator id = foot_ids.begin(); id != foot_ids.end(); ++id) {
    const int frame_id = this->foot_frame_ids[*id];
    Vector3d origin = Vector3d::Zero();
    foot_states[*id].pose = robot->relativeTransform(cache, 0, frame_id);
    auto twist = robot->relativeTwist(cache, 0, frame_id, frame_id);
    auto quat = rotmat2quat(foot_states[*id].pose.linear());
    foot_states[*id].velocity.topRows<3>() = foot_states[*id].pose.linear() * twist.bottomRows<3>();
    Matrix<double, QUAT_SIZE, SPACE_DIMENSION> omega_to_quatdot;
    angularvel2quatdotMatrix(quat, omega_to_quatdot, static_cast<Gradient<decltype(omega_to_quatdot), Dynamic>::type*>(nullptr));
    foot_states[*id].velocity.bottomRows<4>() = omega_to_quatdot * foot_states[*id].pose.linear() * twist.topRows<3>();
    int body_id = this->robot->parseBodyOrFrameID(this->foot_frame_ids[*id]);
    foot_states[*id].contact = contact_force_detected[body_id];
    if (foot_states[*id].pose.translation().z() < min_foot_height) {
      min_foot_height = foot_states[*id].pose.translation().z();
    }
  }

  for (std::vector<FootID>::iterator id = foot_ids.begin(); id != foot_ids.end(); ++id) {
    // NOTE: not using terrain maps at all, so we're just assuming the ground is flat under the lower foot
    foot_states[*id].terrain_height = min_foot_height;
    foot_states[*id].terrain_normal = Vector3d(0,0,1);
  }

  return foot_states;
}

void QPReactiveRecoveryPlan::initLCM() {
  this->LCMHandle = std::shared_ptr<lcm::LCM>(new lcm::LCM);
  if (!this->LCMHandle->good()) {
    throw std::runtime_error("lcm is not good");
  }
}

