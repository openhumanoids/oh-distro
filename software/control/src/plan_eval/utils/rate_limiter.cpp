//
// Created by manuelli on 9/2/16.
//

#include "rate_limiter.h"


RateLimiter::RateLimiter():requiredElapsedTimeInMilliseconds(0){
  prevTime = std::chrono::system_clock::now();
};

void RateLimiter::setSpeedLimit(double speedLimitInHz) {
  if (speedLimitInHz > 0){
    int temp = (int) 1e3*1.0/speedLimitInHz;
    requiredElapsedTimeInMilliseconds= std::chrono::milliseconds(temp);
  }
}

bool RateLimiter::tick(){
  elapsedMilleseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - prevTime);

  if(elapsedMilleseconds > requiredElapsedTimeInMilliseconds){
    prevTime = std::chrono::system_clock::now();
    return true;
  }

  return false;
}