#include "drawingUtil.hpp"

using namespace Eigen;

void draw3dLine(bot_lcmgl_t *lcmgl, Vector3d start, Vector3d end)
{
	bot_lcmgl_begin(lcmgl, LCMGL_LINES);
	bot_lcmgl_vertex3f(lcmgl, start[0], start[1], start[2]);
	bot_lcmgl_vertex3f(lcmgl, end[0], end[1], end[2]);
	bot_lcmgl_end(lcmgl);
}

void draw3dLine(bot_lcmgl_t *lcmgl, double start_x, double start_y, double start_z, double end_x, double end_y, double end_z)
{
	bot_lcmgl_begin(lcmgl, LCMGL_LINES);
	bot_lcmgl_vertex3f(lcmgl, start_x, start_y, start_z);
	bot_lcmgl_vertex3f(lcmgl, end_x, end_y, end_z);
	bot_lcmgl_end(lcmgl);
}
