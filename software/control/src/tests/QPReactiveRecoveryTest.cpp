#include "control/QPReactiveRecoveryPlan.hpp"

int testClosestPointInConvexHull() {
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

int main() {
  bool failed = false;
  int error = testClosestPointInConvexHull();
  if (error) {
    std::cout << "testClosestPointInConvexHull failed" << std::endl;
    failed = true;
  }

  if (!failed) {
    std::cout << "Reactive recovery tests passed" << std::endl;
  } else {
    exit(1);
  }

}
