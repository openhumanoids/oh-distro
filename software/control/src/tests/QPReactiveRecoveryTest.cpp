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
  std::set<double> roots = QPReactiveRecoveryPlan::expIntercept(expform, l0, ld0, u, d);

  Polynomial p_exp = expform.taylorExpand(d);
  // l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
  VectorXd coefs_int(3);
  coefs_int << l0 - 0.25*ld0*ld0/u, 0.5*ld0, 0.25*u;
  Polynomial p_int(coefs_int);
  for (std::set<double>::iterator it = roots.begin(); it != roots.end(); ++it) {
    double val_exp = p_exp.value(*it);
    double val_int = p_int.value(*it);
    if (std::abs(val_exp - val_int) > 1e-3) return 1;
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
  }
  error = testClosestPointInConvexHull2();
  if (error) {
    std::cout << "testClosestPointInConvexHull2 failed" << std::endl;
    failed = true;
  }
  error = testClosestPointInConvexHull3();
  if (error) {
    std::cout << "testClosestPointInConvexHull3 failed" << std::endl;
    failed = true;
  }
  error = testExpTaylor();
  if (error) {
    std::cout << "testExpTaylor failed" << std::endl;
    failed = true;
  }
  error = testExpIntercept();
  if (error) {
    std::cout << "testExpIntercept failed" << std::endl;
    failed = true;
  }

  if (!failed) {
    std::cout << "Reactive recovery tests passed" << std::endl;
  } else {
    exit(1);
  }

}
