//
// Created by manuelli on 9/2/16.
//

#ifndef CONTROL_RATE_LIMITER_H
#define CONTROL_RATE_LIMITER_H
#include <chrono>

class RateLimiter{
public:
  RateLimiter();
  ~RateLimiter(){};

  void setSpeedLimit(double speedLimitInHz);
  bool tick();

private:
  std::chrono::time_point<std::chrono::system_clock> prevTime;
  std::chrono::time_point<std::chrono::system_clock> currTime;
  std::chrono::milliseconds requiredElapsedTimeInMilliseconds;
  std::chrono::milliseconds elapsedMilleseconds;
};

#endif //CONTROL_RATE_LIMITER_H
