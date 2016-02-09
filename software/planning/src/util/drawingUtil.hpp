#ifndef SRC_DRAWINGUTIL_HPP_
#define SRC_DRAWINGUTIL_HPP_

#include <Eigen/Dense>
#include "bot_lcmgl_client/lcmgl.h"

void draw3dLine(bot_lcmgl_t *lcmgl, Eigen::Vector3d start, Eigen::Vector3d end);
void draw3dLine(bot_lcmgl_t *lcmgl, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z);
void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v );
void drawPointCloud(bot_lcmgl_t *lcmgl, std::vector<Eigen::Vector3d> point_cloud);

#endif
