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

void HSVtoRGB( float &r, float &g, float &b, float h, float s, float v )
{
	int i;
	float f, p, q, t;

	if( s == 0 ) {
		// achromatic (grey)
		r = g = b = v;
		return;
	}

	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );

	switch( i ) {
		case 0:
			r = v;
			g = t;
			b = p;
			break;
		case 1:
			r = q;
			g = v;
			b = p;
			break;
		case 2:
			r = p;
			g = v;
			b = t;
			break;
		case 3:
			r = p;
			g = q;
			b = v;
			break;
		case 4:
			r = t;
			g = p;
			b = v;
			break;
		default:		// case 5:
			r = v;
			g = p;
			b = q;
			break;
	}

}
