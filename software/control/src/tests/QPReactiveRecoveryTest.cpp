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
  Polynomial p = QPReactiveRecoveryPlan::expTaylor(a, b, c, d);
  if (std::abs(p.value(0) - (a + c)) > 1e-5) {
    std::cout << p.value(0) << std::endl;
    return 1;
  }
  if (std::abs(p.value(0.5) - (a * exp(b*0.5) + c)) > 1e-5) {
    return 1;
  }
  if (std::abs(p.value(1.5) - (a * exp(b*1.5) + c)) > 1e-3) {
    return 1;
  }
  if (std::abs(p.value(5) - (a * exp(b*5) + c)) > 1) {
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

  if (!failed) {
    std::cout << "Reactive recovery tests passed" << std::endl;
  } else {
    exit(1);
  }

}
