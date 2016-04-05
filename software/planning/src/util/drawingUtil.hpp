#ifndef SRC_DRAWINGUTIL_HPP_
#define SRC_DRAWINGUTIL_HPP_

#include <Eigen/Dense>
#include <sys/time.h>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <chrono>

#include "bot_lcmgl_client/lcmgl.h"
#include "drake/systems/plants/RigidBodyTree.h"

void draw3dLine(bot_lcmgl_t *lcmgl, Eigen::Vector3d start, Eigen::Vector3d end);
void draw3dLine(bot_lcmgl_t *lcmgl, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z);
void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v );
void drawPointCloud(bot_lcmgl_t *lcmgl, std::vector<Eigen::Vector3d> point_cloud);

class CandidateRobotPosePublisher
{
public:
	CandidateRobotPosePublisher();
	int64_t timestamp_now();
	void publish(boost::shared_ptr<lcm::LCM> lcm, RigidBodyTree &robot, Eigen::VectorXd &pose);
};

class FPPTimer
{
public:
	FPPTimer();
	void start();
	void stop();
	void reset();
	double getDuration(){return duration.count()/1.e6;}
private:
	std::chrono::high_resolution_clock::time_point before_time;
	std::chrono::high_resolution_clock::time_point after_time;
	std::chrono::microseconds duration;
};

#endif
