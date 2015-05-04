#include "control/AtlasFallDetector.hpp"
#include "control/convexHull.hpp"

using namespace Eigen;

int testConvexHull() {
  Matrix<double, 2, Dynamic> pts(2, 4);
  pts << 1, 2, 3, 2,
         2, 1, 2, 3;

  if (!in_convex_hull(pts, Vector2d(2, 2))) {
    fprintf(stderr, "2,2 should be in hull\n");
    return 1;
  }

  if (!in_convex_hull(pts, Vector2d(1.0001, 2))) {
    fprintf(stderr, "1.0001, 2 should be in hull\n");
    return 1;
  }
  if (in_convex_hull(pts, Vector2d(0.9999, 2))) {
    fprintf(stderr, "0.9999, 2 should not be in hull\n");
    return 1;
  }
  if (!in_convex_hull(pts, Vector2d(2.49, 2.49))) {
    fprintf(stderr, "2.49,2.49 should be in hull\n");
    return 1;
  }
  if (in_convex_hull(pts, Vector2d(2.51, 2.51))) {
    fprintf(stderr, "2.51,2.51 should not be in hull\n");
    return 1;
  }

  return 0;

}

int testDebounce() {
  std::unique_ptr<Debounce> debounce (new Debounce());
  debounce->t_low_to_high = 1;
  debounce->t_high_to_low = 2;

  if (debounce->update(5, false)) {
    return 1;
  }
  if (debounce->update(5.5, true)) {
    return 1;
  }
  if (debounce->update(5.99, true)) {
    return 1;
  }
  if (!debounce->update(6.51, true)) {
    return 1;
  }
  if (!debounce->update(6.8, false)) {
    return 1;
  }
  if (!debounce->update(7.0, true)) {
    return 1;
  }
  if (!debounce->update(7.1, false)) {
    return 1;
  }
  if (!debounce->update(9.09, false)) {
    return 1;
  }
  if (debounce->update(9.11, false)) {
    return 1;
  }
  return 0;
}


int main() {
  bool failed = false;
  int error;

  error = testConvexHull();
  if (error) {
    std::cout << "testConvexHull FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testConvexHull passed" << std::endl;
  }
  error = testDebounce();
  if (error) {
    std::cout << "testDebounce FAILED" << std::endl;
    failed = true;
  } else {
    std::cout << "testDebounce passed" << std::endl;
  }

  if (!failed) {
    std::cout << "fall detector tests passed" << std::endl;
  } else {
    std::cout << "fall detector tests FAILED" << std::endl;
    exit(1);
  }
}
