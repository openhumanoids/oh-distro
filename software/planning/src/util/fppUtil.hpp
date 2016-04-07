#ifndef SRC_FPPUTIL_HPP_
#define SRC_FPPUTIL_HPP_

#include <sys/time.h>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <chrono>

#include "bot_lcmgl_client/lcmgl.h"
#include "drake/systems/plants/RigidBodyTree.h"

void draw3dLine(bot_lcmgl_t *lcmgl, const Eigen::Vector3d start, const Eigen::Vector3d end);
void draw3dLine(bot_lcmgl_t *lcmgl, const double start_x, const double start_y,
    const double start_z, const double end_x, const double end_y, const double end_z);
void HSVtoRGB(float &r, float &g, float &b, float h, float s, float v);
void drawPointCloud(bot_lcmgl_t *lcmgl, const std::vector<Eigen::Vector3d> point_cloud);

class CandidateRobotPosePublisher
{
public:
  CandidateRobotPosePublisher();
  void publish(boost::shared_ptr<lcm::LCM> lcm, const RigidBodyTree &robot,
      const Eigen::VectorXd &pose);
};

class FPPTimer
{
public:
  FPPTimer();
  void start();
  void stop();
  void reset();
  double getDuration()
  {
    return duration.count() / 1.e6;
  }
private:
  std::chrono::high_resolution_clock::time_point before_time;
  std::chrono::high_resolution_clock::time_point after_time;
  std::chrono::microseconds duration;
};

#endif
