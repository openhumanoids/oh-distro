#include <math.h>
#include "control/QPReactiveRecoveryPlan.hpp"

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
  Polynomial p = expform.taylorExpand(d);
  if (std::abs(p.value(0) - (a + c)) > 1e-5) {
    std::cout << p.value(0) << std::endl;
    return 1;
  }
  if (std::abs(p.value(0.5) - expform.value(0.5)) > 1e-5) {
    return 1;
  }
  if (std::abs(p.value(1.5) - expform.value(1.5)) > 1e-3) {
    return 1;
  }
  if (std::abs(p.value(5) - expform.value(5)) > 1) {
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

  Polynomial p_exp = expform.taylorExpand(d);
  // l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
  VectorXd coefs_int(3);
  coefs_int << l0 - 0.25*ld0*ld0/u, 0.5*ld0, 0.25*u;
  Polynomial p_int(coefs_int);
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
  QPReactiveRecoveryPlan plan;
  plan.capture_shrink_factor = 0.8;
  plan.capture_max_flyfoot_height = 0.025;

  std::map<FootID, FootState> foot_states;
  std::map<FootID, Matrix<double, 2, 4>> foot_vertices;
  Vector2d r_ic;
  FootState rstate;
  rstate.pose.setIdentity();
  FootState lstate;
  lstate.pose.setIdentity();
  bool captured;

  Matrix<double, 2, 4> V;
  V << -.1, .1, .1, -.1,
        -.05, -.05, .05, .05;
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

  captured = plan.isICPCaptured(r_ic, foot_states, foot_vertices);
  if (!captured) {
    std::cout << "should be captured" << std::endl;
    return 1;
  }

  foot_states[LEFT].contact = false;
  captured = plan.isICPCaptured(r_ic, foot_states, foot_vertices);
  if (!captured) {
    // should still be captured because lfoot is close to the terrain
    std::cout << "should be captured" << std::endl;
    return 1;
  }

  foot_states[LEFT].terrain_height = -0.1;
  captured = plan.isICPCaptured(r_ic, foot_states, foot_vertices);
  if (captured) {
    // should not be captured, because terrain is too low 
    std::cout << "should not be captured" << std::endl;
    return 1;
  }

  foot_states[LEFT].contact = true;
  r_ic << 1.0, 2.1;
  captured = plan.isICPCaptured(r_ic, foot_states, foot_vertices);
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



int main() {
  bool failed = false;
  int error;

  error = testClosestPointInConvexHull1();
  if (error) {
    std::cout << "testClosestPointInConvexHull1 failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPointInConvexHull1 passed" << std::endl;
  }
  error = testClosestPointInConvexHull2();
  if (error) {
    std::cout << "testClosestPointInConvexHull2 failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPointInConvexHull2 passed" << std::endl;
  }
  error = testClosestPointInConvexHull3();
  if (error) {
    std::cout << "testClosestPointInConvexHull3 failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testClosestPointInConvexHull3 passed" << std::endl;
  }
  error = testExpTaylor();
  if (error) {
    std::cout << "testExpTaylor failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testExpTaylor passed" << std::endl;
  }
  error = testExpIntercept();
  if (error) {
    std::cout << "testExpIntercept failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testExpIntercept passed" << std::endl;
  } 
  error = testBangBangIntercept();
  if (error) {
    std::cout << "testBangBangIntercept failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testBangBangIntercept passed" << std::endl;
  }
  error = testisICPCaptured();
  if (error) {
    std::cout << "testisICPCaptured failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testisICPCaptured passed" << std::endl;
  }
  error = testGetTWorldToLocal();
  if (error) {
    std::cout << "testGetTWorldToLocal failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testGetTWorldToLocal passed" << std::endl;
  }
  error = testMinTimeToXprime();
  if (error) {
    std::cout << "testMinTimeToXprime failed" << std::endl;
    failed = true;
  } else {
    std::cout << "testMinTimeToXprime passed" << std::endl;
  }


  if (!failed) {
    std::cout << "Reactive recovery tests passed" << std::endl;
  } else {
    exit(1);
  }

}
