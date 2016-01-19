#include "../AtlasFallDetector.hpp"

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
