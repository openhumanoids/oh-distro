#include <math.h>
#include "../QPReactiveRecoveryPlan.hpp"

using namespace Eigen;

int testClosestPointInConvexHull1() {
  // Try a simple rectangle in 2D
  MatrixXd V(2,4);
  V << 0, 1, 1, 0,
       0, 0, 1, 1;

  VectorXd x(2);
  x << 0.5, 0.5;

  VectorXd y_expected(2);
  y_expected << 0.5, 0.5;

  VectorXd y = QPReactiveRecoveryPlan::closestPointInConvexHull(x, V);
  if (!y.isApprox(y_expected, 1e-4)) {
    return 1;
  }

  return 0;
}

int testClosestPointInConvexHull2() {
  MatrixXd V(2,4);
  V << 0, 1, 1, 0,
       0, 0, 1, 1;

  VectorXd x(2);
  x << 10, 10;

  VectorXd y_expected(2);
  y_expected << 1, 1;

  VectorXd y = QPReactiveRecoveryPlan::closestPointInConvexHull(x, V);

  if (!y.isApprox(y_expected, 1e-4)) {
    return 1;
  }

  return 0;
}

int testClosestPointInConvexHull3() {
  MatrixXd V(2,4);
  V << 2, 1, 1, 2,
       2, 2, 1, 1;

  VectorXd x(2);
  x << 0, 0;

  VectorXd y_expected(2);
  y_expected << 1, 1;

  VectorXd y = QPReactiveRecoveryPlan::closestPointInConvexHull(x, V);

  if (!y.isApprox(y_expected, 1e-4)) {
    return 1;
  }

  return 0;
}

int testExpTaylor() {
  int d = 5;
  double a = 0.5;
  double b = 0.6;
  double c = 0.7;
  ExponentialForm expform(a, b, c);
  Polynomial<double> p = expform.taylorExpand(d);
  if (std::abs(expform.value(0.0) - (a + c)) > 1e-10) {
    std::cout << "expform value does not match" << std::endl;
    std::cout << expform.value(0.0) << std::endl;
    std::cout << a + c << std::endl;
    return 1;
  }
  if (std::abs(p.value(0.0) - (a + c)) > 1e-5) {
    std::cout << p.value(0.0) << std::endl;
    std::cout << a + c << std::endl;
    return 1;
  }
  if (std::abs(p.value(0.5) - expform.value(0.5)) > 1e-5) {
    return 1;
  }
  if (std::abs(p.value(1.5) - expform.value(1.5)) > 1e-3) {
    return 1;
  }
  if (std::abs(p.value(5.0) - expform.value(5.0)) > 1) {
    return 1;
  }
  return 0;
}

int testExpIntercept() {
  int d = 5;
  double a = 0.5;
  double b = 0.6;
  double c = 0.7;

  double l0 = 0.1;
  double ld0 = 0.5;

  double u = 10;

  ExponentialForm expform(a, b, c);
  std::vector<double> roots = QPReactiveRecoveryPlan::expIntercept(expform, l0, ld0, u, d);

  Polynomial<double> p_exp = expform.taylorExpand(d);
  // l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
  VectorXd coefs_int(3);
  coefs_int << l0 - 0.25*ld0*ld0/u, 0.5*ld0, 0.25*u;
  Polynomial<double> p_int(coefs_int);
  for (std::vector<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
    double val_exp = p_exp.value(*it);
    double val_int = p_int.value(*it);
    if (std::abs(val_exp - val_int) > 1e-3) return 1;
  }
  return 0;
}

int testBangBangIntercept() {
  double x0 = -1.5;
  double xd0 = 2.0;
  double xf = 5.0;
  double u_max = 10;

  std::vector<BangBangIntercept> intercepts = QPReactiveRecoveryPlan::bangBangIntercept(x0, xd0, xf, u_max);
  if (intercepts.size() != 1) {
    std::cout << "intercepts wrong size" << std::endl;
    return 1;
  }
  if (std::abs(intercepts[0].tf - 1.4371) > 1e-4) {
    std::cout << "tf wrong" << std::endl;
    return 1;
  }
  if (std::abs(intercepts[0].tswitch - 0.6185) > 1e-4) {
    std::cout << "tswitch wrong" << std::endl;
    return 1;
  }
  if (std::abs(intercepts[0].u - 10) > 1e-14) {
    std::cout << "u wrong" << std::endl;
    return 1;
  }

  x0 = -1.5;
  xd0 = 2.0;
  xf = -5.0;
  u_max = 10;

  intercepts = QPReactiveRecoveryPlan::bangBangIntercept(x0, xd0, xf, u_max);
  if (intercepts.size() != 1) {
    std::cout << "intercepts wrong size" << std::endl;
    return 1;
  }
  if (std::abs(intercepts[0].tf - 1.4166) > 1e-4) {
    std::cout << "tf wrong" << std::endl;
    return 1;
  }
  if (std::abs(intercepts[0].tswitch - 0.8083) > 1e-4) {
    std::cout << "tswitch wrong" << std::endl;
    return 1;
  }
  if (std::abs(intercepts[0].u - -10) > 1e-14) {
    std::cout << "u wrong: " << intercepts[0].u << std::endl;
    return 1;
  }
  return 0;
}

int testisICPCaptured() {
  RobotPropertyCache rpc;
  std::unique_ptr<QPReactiveRecoveryPlan> plan(new QPReactiveRecoveryPlan(NULL, rpc));
  plan->capture_shrink_factor = 0.8;
  plan->capture_max_flyfoot_height = 0.025;

  std::map<FootID, FootState> foot_states;
  std::map<FootID, Matrix<double, 3, 4>> foot_vertices;
  Vector2d r_ic;
  FootState rstate;
  rstate.pose.setIdentity();
  FootState lstate;
  lstate.pose.setIdentity();
  bool captured;

  Matrix<double, 3, 4> V;
  V << -.1, .1, .1, -.1,
        -.05, -.05, .05, .05,
        0, 0, 0, 0;
  foot_vertices[RIGHT] = V;
  foot_vertices[LEFT] = V;

  rstate.pose.translate(Vector3d(1.13, 2, 0)).rotate(AngleAxis<double>(M_PI/2, Vector3d(0, 0, 1)));
  rstate.contact = true;
  rstate.terrain_height = 0;
  foot_states[RIGHT] = rstate;

  lstate.pose.translate(Vector3d(0.87, 2, 0)).rotate(AngleAxis<double>(M_PI/2, Vector3d(0, 0, 1)));
  lstate.contact = true;
  lstate.terrain_height = 0;
  foot_states[LEFT] = lstate;

  r_ic << 1.0, 2.0;

  captured = plan->isICPCaptured(r_ic, foot_states, foot_vertices);
  if (!captured) {
    std::cout << "should be captured" << std::endl;
    return 1;
  }

  foot_states[LEFT].contact = false;
  captured = plan->isICPCaptured(r_ic, foot_states, foot_vertices);
  if (!captured) {
    // should still be captured because lfoot is close to the terrain
    std::cout << "should be captured" << std::endl;
    return 1;
  }

  foot_states[LEFT].terrain_height = -0.1;
  captured = plan->isICPCaptured(r_ic, foot_states, foot_vertices);
  if (captured) {
    // should not be captured, because terrain is too low 
    std::cout << "should not be captured" << std::endl;
    return 1;
  }

  foot_states[LEFT].contact = true;
  r_ic << 1.0, 2.1;
  captured = plan->isICPCaptured(r_ic, foot_states, foot_vertices);
  if (captured) {
    // should not be captured because r_ic is out of support
    std::cout << "should not be captured" << std::endl;
    return 1;
  }
  return 0;
}

int testGetTWorldToLocal() {
  Isometry3d icp = Isometry3d::Identity();
  Isometry3d cop = Isometry3d::Identity();
  Isometry3d T_world_to_local;

  icp.translate(Vector3d(1, 0, 0));
  T_world_to_local = QPReactiveRecoveryPlan::getTWorldToLocal(icp, cop);
  if (!T_world_to_local.isApprox(Isometry3d::Identity())) {
    std::cout << "transform 1 should be the identity" << std::endl;
    return 1;
  }

  cop.translate(Vector3d(0.5, 0, 0));
  T_world_to_local = QPReactiveRecoveryPlan::getTWorldToLocal(icp, cop);
  if (!(T_world_to_local * cop).isApprox(Isometry3d::Identity())) {
    std::cout << "cop should always be at the origin" << std::endl;
    return 1;
  }
  if (!(std::abs((T_world_to_local * icp).translation().y()) < 1e-10)) {
    std::cout << "icp should always be along the x axis in the local frame" << std::endl;
    return 1;
  }

  icp = Isometry3d::Identity();
  cop = Isometry3d::Identity();
  cop.translate(Vector3d(0.5, 0.5, 0));
  T_world_to_local = QPReactiveRecoveryPlan::getTWorldToLocal(icp, cop);
  if (!(T_world_to_local * cop).translation().isApprox(Vector3d(0,0,0))) {
    std::cout << (T_world_to_local * cop).matrix() << std::endl;
    std::cout << "cop should always be at the origin" << std::endl;
    return 1;
  }
  if (!(std::abs((T_world_to_local * icp).translation().y()) < 1e-10)) {
    std::cout << "icp should always be along the x axis in the local frame" << std::endl;
    return 1;
  }
  if (!(std::abs((T_world_to_local * icp).translation().x() - (1.0 / std::sqrt(2))) < 1e-10)) {
    std::cout << (T_world_to_local * icp).translation() << std::endl;
    std::cout << "icp should be along the positive x axis" << std::endl;
    return 1;
  }
  if (!(T_world_to_local.inverse() * T_world_to_local * cop).isApprox(cop)) {
    std::cout << "inverse should give us back the original pose" << std::endl;
    return 1;
  }

  return 0;
}

int testMinTimeToXprime() {
  double t_min;
  Isometry3d T_world_to_local;
  BipedDescription biped = getAtlasDefaults();
  FootState foot_state;
  foot_state.pose = Isometry3d::Identity();
  foot_state.velocity = XYZQuat::Zero();

  foot_state.pose.translate(Vector3d(1, 0, 0));

  Isometry3d icp = Isometry3d::Identity();
  Isometry3d cop = Isometry3d::Identity();

  icp.translate(Vector3d(0.1, 0, 0));

  T_world_to_local = QPReactiveRecoveryPlan::getTWorldToLocal(icp, cop);

  // xprime axis should be the same as the x axis here
  t_min = QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(foot_state, biped, T_world_to_local);
  if (std::abs(t_min) > 1e-10) {
    std::cout << "time to xprime should be zero here" << std::endl;
    return 1;
  }

  foot_state.velocity(0) = 10;
  t_min = QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(foot_state, biped, T_world_to_local);
  if (std::abs(t_min) > 1e-10) {
    std::cout << "velocity along xprime should have no effect on intercept time" << std::endl;
    return 1;
  }

  foot_state.velocity(0) = 10;
  t_min = QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(foot_state, biped, T_world_to_local);
  if (std::abs(t_min) > 1e-10) {
    std::cout << "velocity along xprime should have no effect on intercept time" << std::endl;
    return 1;
  }

  // x = x0 + xd0*t + 1/2*u*t^2
  // x = x0 - u*t + 1/2*u*t^2; let t=1;
  // x = x0 - u + 1/2*u
  foot_state.velocity = XYZQuat::Zero();
  foot_state.velocity(1) = -biped.u_max;
  foot_state.pose.translate(Vector3d(0, 0.5 * biped.u_max, 0));
  t_min = QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(foot_state, biped, T_world_to_local);
  if (std::abs(t_min - 1) > 1e-10) {
    std::cout << "should take exactly 1 second to get to xprime from this state" << std::endl;
    return 1;
  }

  icp = Isometry3d::Identity();
  cop = Isometry3d::Identity();
  foot_state.velocity = XYZQuat::Zero();
  foot_state.pose = Isometry3d::Identity();
  cop.translate(Vector3d(-1, 0.5, 0));
  icp.translate(Vector3d(-1, 1, 0));
  T_world_to_local = QPReactiveRecoveryPlan::getTWorldToLocal(icp, cop);

  foot_state.pose.translate(Vector3d(-0.5, 0, 0));

  // dx = 1/2 * u * (t/2)^2 + 1/2 * u * (t/2)^2 = 0.5;
  // u * t^2/4 = 0.5;
  // u * t^2 = 2;
  // t = sqrt(2 / u)
  t_min = QPReactiveRecoveryPlan::getMinTimeToXprimeAxis(foot_state, biped, T_world_to_local);
  if (std::abs(t_min - std::sqrt(2.0 / biped.u_max)) > 1e-10) {
    std::cout << "t_min: " << t_min << std::endl;
    std::cout << "sqrt(2/u): " << std::sqrt(2.0 / biped.u_max) << std::endl;
    std::cout << "should take sqrt(2/u) seconds to get to xprime axis" << std::endl;
    return 1;
  }

  return 0;
}

int testClosestPoseInConvexHull() {
  Isometry3d pose = Isometry3d::Identity();
  Isometry3d pose_closest;


  Matrix<double, 2, 4> verts;
  verts << -2, -1, -1, -2, 
           -3, -2, -2, -3;

  pose_closest = QPReactiveRecoveryPlan::closestPoseInConvexHull(pose, verts);
  if (!pose_closest.isApprox(Isometry3d(Translation<double, 3>(Vector3d(-1, -2, 0))), 1e-3)) {
    std::cout << pose_closest.matrix() << std::endl;
    std::cout << Isometry3d(Translation<double, 3>(Vector3d(-1, -2, 0))).matrix() << std::endl;
    std::cout << "should be at -1, -2, 0" << std::endl;
    return 1;
  }

  pose = Isometry3d(Translation<double, 3>(Vector3d(2, 4, 1)));
  pose.rotate(AngleAxis<double>(M_PI / 2, Vector3d(0, 0, 1)));
  pose_closest = QPReactiveRecoveryPlan::closestPoseInConvexHull(pose, verts);
  if ((pose_closest.translation() - Vector3d(-1, -2, 1)).norm() > 1e-2) {
    std::cerr << pose_closest.translation() << std::endl;
    fprintf(stderr, "Wrong pose\n");
    return 1;
  }
  if (!pose_closest.rotation().isApprox(pose.rotation())) {
    std::cerr << pose_closest.rotation() << std::endl;
    fprintf(stderr, "Rotation should be preserved\n");
    return 1;
  }
  return 0;
}

int testGetInterceptsWithCoP() {
  FootID swing_foot = RIGHT;
  std::map<FootID, FootState> foot_states;
  BipedDescription biped = getAtlasDefaults();
  Isometry3d icp = Isometry3d::Identity();
  Isometry3d cop = Isometry3d::Identity();

  foot_states[LEFT].pose = Isometry3d::Identity();
  foot_states[LEFT].pose.translate(Vector3d(-0.13, 1, 0.1));
  foot_states[LEFT].pose.rotate(AngleAxis<double>(M_PI / 2, Vector3d(0, 0, 1)));

  foot_states[RIGHT].pose = Isometry3d::Identity();
  foot_states[RIGHT].pose.translate(Vector3d(0.13, 1, 0.1));
  foot_states[RIGHT].pose.rotate(AngleAxis<double>(M_PI / 2, Vector3d(0, 0, 1)));

  cop.translate(Vector3d(-0.11, 1.05, 0.1));
  icp.translate(Vector3d(-0.1, 1.06, 0.1));

  RobotPropertyCache rpc;
  std::unique_ptr<QPReactiveRecoveryPlan> planner(new QPReactiveRecoveryPlan(NULL, rpc, biped));

  std::vector<InterceptPlan> intercept_plans = planner->getInterceptsWithCoP(swing_foot, foot_states, icp, cop);

  Matrix<double, 3, 4> reachable_verts_in_world = foot_states[LEFT].pose * biped.reachable_vertices[RIGHT];
  for (std::vector<InterceptPlan>::iterator plan = intercept_plans.begin(); plan != intercept_plans.end(); ++plan) {
    // std::cout << plan->tf << std::endl;
    // std::cout << plan->pose_next.matrix() << std::endl;
    if (plan->tf < 0.5 * planner->min_step_duration) {
      fprintf(stderr, "Min step duration violated\n");
      return 1;
    }
    Isometry3d closest_reachable_pose = QPReactiveRecoveryPlan::closestPoseInConvexHull(plan->pose_next, reachable_verts_in_world.topRows(2));
    if ((closest_reachable_pose.translation().head(2) - plan->pose_next.translation().head(2)).norm() > 1e-2) {
      std::cerr << plan->pose_next.translation() << std::endl;
      std::cerr << reachable_verts_in_world << std::endl;
      std::cerr << closest_reachable_pose.translation() << std::endl;
      fprintf(stderr, "Desired step is unreachable.\n");
      return 1;
    }
  }

  cop = Isometry3d(Translation<double, 3>(Vector3d(-0.11, 1, 0.1)));
  icp = Isometry3d(Translation<double, 3>(Vector3d(1, 1, 0.1)));
  intercept_plans = planner->getInterceptsWithCoP(swing_foot, foot_states, icp, cop);
  for (std::vector<InterceptPlan>::iterator plan = intercept_plans.begin(); plan != intercept_plans.end(); ++plan) {
    // std::cout << plan->tf << std::endl;
    // std::cout << plan->pose_next.matrix() << std::endl;
    if (std::abs(plan->pose_next.translation().y() - 1) > 1e-2) {
      fprintf(stderr, "intercept should be on the y=1 axis\n");
      return 1;
    }
  }

  icp = Isometry3d(Translation<double, 3>(Vector3d(0, 1, 0.1)));
  biped.u_max = 20;
  planner.reset(new QPReactiveRecoveryPlan(NULL, rpc, biped));
  intercept_plans = planner->getInterceptsWithCoP(swing_foot, foot_states, icp, cop);
  for (std::vector<InterceptPlan>::iterator plan = intercept_plans.begin(); plan != intercept_plans.end(); ++plan) {
    // std::cout << plan->tf << std::endl;
    // std::cout << plan->pose_next.matrix() << std::endl;
    if (std::abs(plan->pose_next.translation().y() - 1) > 1e-2) {
      fprintf(stderr, "intercept should be on the y=1 axis\n");
      return 1;
    }
  }

  cop = Isometry3d(Translation<double, 3>(Vector3d(-0.11, 1, 0.1)));
  icp = Isometry3d(Translation<double, 3>(Vector3d(-0.11, 1, 0.1)));
  intercept_plans = planner->getInterceptsWithCoP(swing_foot, foot_states, icp, cop);
  for (std::vector<InterceptPlan>::iterator plan = intercept_plans.begin(); plan != intercept_plans.end(); ++plan) {
    // std::cout << plan->tf << std::endl;
    // std::cout << plan->pose_next.matrix() << std::endl;
    if (std::abs(plan->pose_next.translation().y() - (foot_states[RIGHT].pose.translation().y())) > 1e-2) {
      fprintf(stderr, "should be at the same y pose\n");
      return 1;
    }
  }

  return 0;
}

int testFreeKnotTimesSpline() {
  double t0 = 11;
  double tf = 15;
  Matrix<double, 2, 4> xs;
  xs << 0, 5, 2, 3,
        4, -1, 6, 7;
  Vector2d xd0;
  xd0 << 0, 1;

  Vector2d xdf = Vector2d::Zero();

  std::unique_ptr<PiecewisePolynomial<double>> spline = QPReactiveRecoveryPlan::freeKnotTimesSpline(t0, tf, xs, xd0, xdf);
  if (!spline->value(t0).isApprox(xs.col(0))) {
    std::cout << "initial value does not match" << std::endl;
    return 1;
  }
  if (!spline->derivative().value(t0).isApprox(xd0)) {
    std::cout << "initial velocity does not match" << std::endl;
    return 1;
  }
  if (!spline->value(spline->getSegmentTimes()[1]).isApprox(xs.col(1))) {
    std::cout << "second knot does not match" << std::endl;
    return 1;
  }
  if (!spline->value(tf).isApprox(xs.col(3))) {
    std::cout << "final knot does not match" << std::endl;
    return 1;
  }
  if (!((spline->derivative().value(tf) - xdf).norm() < 1e-14)) {
    std::cout << spline->derivative().value(tf) << " " << xdf << std::endl;
    std::cout << "final velocity does not match" << std::endl;
    return 1;
  }

  return 0;
}

int main() {
  bool failed = false;
  int error;

  error = testClosestPointInConvexHull1();
  if (error) {
    std::cout << "testClosestPointInConvexHull1 FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPointInConvexHull1 passed" << std::endl;
  }
  error = testClosestPointInConvexHull2();
  if (error) {
    std::cout << "testClosestPointInConvexHull2 FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPointInConvexHull2 passed" << std::endl;
  }
  error = testClosestPointInConvexHull3();
  if (error) {
    std::cout << "testClosestPointInConvexHull3 FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPointInConvexHull3 passed" << std::endl;
  }
  error = testExpTaylor();
  if (error) {
    std::cout << "testExpTaylor FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testExpTaylor passed" << std::endl;
  }
  error = testExpIntercept();
  if (error) {
    std::cout << "testExpIntercept FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testExpIntercept passed" << std::endl;
  } 
  error = testBangBangIntercept();
  if (error) {
    std::cout << "testBangBangIntercept FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testBangBangIntercept passed" << std::endl;
  }
  error = testisICPCaptured();
  if (error) {
    std::cout << "testisICPCaptured FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testisICPCaptured passed" << std::endl;
  }
  error = testGetTWorldToLocal();
  if (error) {
    std::cout << "testGetTWorldToLocal FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testGetTWorldToLocal passed" << std::endl;
  }
  error = testMinTimeToXprime();
  if (error) {
    std::cout << "testMinTimeToXprime FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testMinTimeToXprime passed" << std::endl;
  }
  error = testClosestPoseInConvexHull();
  if (error) {
    std::cout << "testClosestPoseInConvexHull FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPoseInConvexHull passed" << std::endl;
  }
  error = testGetInterceptsWithCoP();
  if (error) {
    std::cout << "testGetInterceptsWithCoP FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testGetInterceptsWithCoP passed" << std::endl;
  }
  error = testFreeKnotTimesSpline();
  if (error) {
    std::cout << "testFreeKnotTimesSpline FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testFreeKnotTimesSpline passed" << std::endl;
  }


  if (!failed) {
    std::cout << "Reactive recovery tests passed" << std::endl;
  } else {
    std::cout << "Reactive recovery tests FAILED" << std::endl;
    exit(1);
  }

}
