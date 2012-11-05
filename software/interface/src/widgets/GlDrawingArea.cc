/*
 * GlDrawingArea.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: ppetrova
 */

#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "GlDrawingArea.h"

using namespace std;
///////////////////////////////////////////////////////////////////////////////
//
// OpenGL frame buffer configuration utilities.
//
///////////////////////////////////////////////////////////////////////////////

struct GLConfigUtil
{
	static void print_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig,
			const char* attrib_str,
			int attrib,
			bool is_boolean);

	static void examine_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig);
};

//
// Print a configuration attribute.
//
void GLConfigUtil::print_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig,
		const char* attrib_str,
		int attrib,
		bool is_boolean)
{
	int value;

	if (glconfig->get_attrib(attrib, value))
	{
		std::cout << attrib_str << " = ";
		if (is_boolean)
			std::cout << (value == true ? "true" : "false") << std::endl;
		else
			std::cout << value << std::endl;
	}
	else
	{
		std::cout << "*** Cannot get "
				<< attrib_str
				<< " attribute value\n";
	}
}

//
// Print configuration attributes.
//
void GLConfigUtil::examine_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig)
{
	/*std::cout << "\nOpenGL visual configurations :\n\n";

	std::cout << "glconfig->is_rgba() = "
			<< (glconfig->is_rgba() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->is_double_buffered() = "
			<< (glconfig->is_double_buffered() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->is_stereo() = "
			<< (glconfig->is_stereo() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_alpha() = "
			<< (glconfig->has_alpha() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_depth_buffer() = "
			<< (glconfig->has_depth_buffer() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_stencil_buffer() = "
			<< (glconfig->has_stencil_buffer() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_accum_buffer() = "
			<< (glconfig->has_accum_buffer() ? "true" : "false")
			<< std::endl;

	std::cout << std::endl;

	print_gl_attrib(glconfig, "Gdk::GL::USE_GL",           Gdk::GL::USE_GL,           true);
	print_gl_attrib(glconfig, "Gdk::GL::BUFFER_SIZE",      Gdk::GL::BUFFER_SIZE,      false);
	print_gl_attrib(glconfig, "Gdk::GL::LEVEL",            Gdk::GL::LEVEL,            false);
	print_gl_attrib(glconfig, "Gdk::GL::RGBA",             Gdk::GL::RGBA,             true);
	print_gl_attrib(glconfig, "Gdk::GL::DOUBLEBUFFER",     Gdk::GL::DOUBLEBUFFER,     true);
	print_gl_attrib(glconfig, "Gdk::GL::STEREO",           Gdk::GL::STEREO,           true);
	print_gl_attrib(glconfig, "Gdk::GL::AUX_BUFFERS",      Gdk::GL::AUX_BUFFERS,      false);
	print_gl_attrib(glconfig, "Gdk::GL::RED_SIZE",         Gdk::GL::RED_SIZE,         false);
	print_gl_attrib(glconfig, "Gdk::GL::GREEN_SIZE",       Gdk::GL::GREEN_SIZE,       false);
	print_gl_attrib(glconfig, "Gdk::GL::BLUE_SIZE",        Gdk::GL::BLUE_SIZE,        false);
	print_gl_attrib(glconfig, "Gdk::GL::ALPHA_SIZE",       Gdk::GL::ALPHA_SIZE,       false);
	print_gl_attrib(glconfig, "Gdk::GL::DEPTH_SIZE",       Gdk::GL::DEPTH_SIZE,       false);
	print_gl_attrib(glconfig, "Gdk::GL::STENCIL_SIZE",     Gdk::GL::STENCIL_SIZE,     false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_RED_SIZE",   Gdk::GL::ACCUM_RED_SIZE,   false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_GREEN_SIZE", Gdk::GL::ACCUM_GREEN_SIZE, false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_BLUE_SIZE",  Gdk::GL::ACCUM_BLUE_SIZE,  false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_ALPHA_SIZE", Gdk::GL::ACCUM_ALPHA_SIZE, false);

	std::cout << std::endl;
	*/
}

namespace gui {

static float zoomFactor; /* Modofied by user input, initially 1.0 */

GlDrawingArea::GlDrawingArea(boost::shared_ptr<lcm::LCM> &lcm)
{

	this->_lcm = lcm;

	//
	// Configure OpenGL-capable visual.
	//

	Glib::RefPtr<Gdk::GL::Config> glconfig;

	// Try double-buffered visual
	glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB    |
			Gdk::GL::MODE_DEPTH  |
			Gdk::GL::MODE_DOUBLE);
	if (!glconfig)
	{
		std::cerr << "*** Cannot find the double-buffered visual.\n"
				<< "*** Trying single-buffered visual.\n";

		// Try single-buffered visual
		glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB   |
				Gdk::GL::MODE_DEPTH);
		if (!glconfig)
		{
			std::cerr << "*** Cannot find any OpenGL-capable visual.\n";
			std::exit(1);
		}
	}

	// print frame buffer attributes.
	GLConfigUtil::examine_gl_attrib(glconfig);

	//
	// Set OpenGL-capability to the widget.
	//

	set_gl_capability(glconfig);
	//glutMouseFunc(mouseWheel);
	zoomFactor = 1.0;
	//	Gtk::Widget::add_events(Gdk::BUTTON_MOTION_MASK);
	// Register the fact that we want to receive these events
	add_events(Gdk::BUTTON1_MOTION_MASK    |
			Gdk::BUTTON2_MOTION_MASK    |
			Gdk::BUTTON3_MOTION_MASK    |
			Gdk::SCROLL_MASK		 |
			Gdk::BUTTON_PRESS_MASK      |
			Gdk::BUTTON_RELEASE_MASK    |
			Gdk::KEY_PRESS_MASK	 |
			Gdk::KEY_RELEASE_MASK	 |
			Gdk::VISIBILITY_NOTIFY_MASK);

	//signal_motion_notify_event().connect(sigc::mem_fun(*this, &GlDrawingArea::on_motion_notify_event));
}

bool GlDrawingArea::on_key_press_event(GdkEventKey* event)
{

	cout << "in key press" << endl;
	if (event->keyval == GDK_a || event->keyval == GDK_A)
		if (event->state & GDK_CONTROL_MASK)
		{
			if (event->type == GDK_KEY_PRESS)
				cout << "Ctrl-A pressed";
			else
				cout << "Ctrl-A released";

			// stop further processing of this event -
			// we've handled it
			return true;
		}

	// this signal handler didn't do anything with the event
	// pass it on to the default (or other signal) handlers
	return false;
}

/*
 * A routine for setting the projeection matrix. May be called from a a resize
 * event handler in a typical application. Takes integer width and height
 * dimensions of the drawing area. Creates a projection matrix with correct
 * aspect ratio and zoom factor.
 */
void GlDrawingArea::setProjectionMatrix(int width, int height) {

	double zNear = 1.0;
	double zFar = 70.0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0 * zoomFactor, (double) width / (double) height, zNear, zFar);
	/* fill in zNear and zFar */
}


GlDrawingArea::GlDrawingArea(const GlDrawingArea& other)
{

}

GlDrawingArea::~GlDrawingArea()
{
}

void GlDrawingArea::on_realize()
{

	Gtk::DrawingArea::on_realize();
	Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();

	if (!glwindow->gl_begin(get_gl_context()))
		return;

	glwindow->gl_end();

}

void GlDrawingArea::doRefresh()
{
	Gtk::Allocation allocation = Gtk::Widget::get_allocation();

	Glib::RefPtr<Gdk::Window> window = get_gl_window()->get_window();
	if (window) {
		window->invalidate_rect(Gdk::Rectangle(allocation), false);
	}
}

void GlDrawingArea::set_size_request(int width, int height) {
	setProjectionMatrix(width, height);
}

void GlDrawingArea::invalidate()
{
	// Force a rerender
	Gtk::Allocation allocation = get_allocation();
	get_window()->invalidate_rect( allocation, false);
}

bool GlDrawingArea::on_configure_event(GdkEventConfigure* event)
{
	Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();

	if (!glwindow->gl_begin(get_gl_context()))
		return false;

	glViewport(0, 0, get_width(), get_height());

	glwindow->gl_end();

	return true;
}

bool GlDrawingArea::on_scroll_event(GdkEventScroll* event) {

	if (event->direction & GDK_SCROLL_DOWN) {
		zoomFactor = zoomFactor - 0.03;
	}
	else {
		zoomFactor = zoomFactor + 0.03;
	}
	doRefresh();
}

bool GlDrawingArea::on_motion_notify_event(GdkEventMotion* event)
{
	if (event->state & GDK_BUTTON1_MASK) { // LMB
		// continue translation changes (x & y)
		//cout << "BUTtON 1 " << endl;
	}
	if (event->state & GDK_BUTTON2_MASK) { // MMB
		// continue translation changes (z)
		//cout << "BUTtON 2 " << endl;
	}
	if (event->state & GDK_BUTTON3_MASK) { // RMB
		// continue rotation changes
		//cout << "BUTtON 3 " << endl;
	}
	return true;
}

bool GlDrawingArea::on_expose_event(GdkEventExpose* event)
{
	// Get GL::Window.
	Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();
	if (!glwindow) return false;

	// *** OpenGL BEGIN ***
	if (!glwindow->gl_begin(get_gl_context()))
		return false;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_COLOR_MATERIAL);
	//glEnable(GL_LINE_SMOOTH_HINT);
	//glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	static GLfloat light_diffuse[] = {0.5, 0.5, 0.5, 1.0};
	static GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);

	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClearDepth(1.0);

	double c[3] = {0.95,0.95,0.95};
	double alpha = 1;

	// Draw the grid
	int max = 800;
	int min = -800;
	for (int xi = min; xi < max; xi++) {
		glPushMatrix();
		glBegin(GL_LINES);
		glColor4f(c[0],c[1],c[2],alpha);
		glVertex2i(xi, min);
		glVertex2i(xi, max);
		glEnd();
		glPopMatrix();
		glPushMatrix();
		glBegin(GL_LINES);
		glColor4f(c[0],c[1],c[2],alpha);
		glVertex2i(min, xi);
		glVertex2i(max, xi);
		glEnd();
		glPopMatrix();

	}

	double xAxisColor[3] = {0.0, 1.0, 0.0};
	double yAxisColor[3] = {0.0, 1.0, 0.0};
	double zAxisColor[3] = {0.0, 0.0, 1.0};
	double transparancy = 0.6;

	glPushMatrix();
	glBegin(GL_LINES);
	glColor4f(xAxisColor[0],xAxisColor[1],xAxisColor[2], transparancy);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(3.0, 0.0, 0.0);

	glColor4f(yAxisColor[0],yAxisColor[1],yAxisColor[2], transparancy);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 3.0, 0.0);

	glColor4f(zAxisColor[0],zAxisColor[1],zAxisColor[2], transparancy);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 3.0, 0.0);
	glEnd();
	glPopMatrix();

	glViewport(0, 0, get_width(), get_height());

	setProjectionMatrix(500, 500);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//gluLookAt( eyeX, eyeY, eyeZ, CenterX, CenterY, CenterZ, UpVectorX, UpVectorY, UpVectorZ)
	gluLookAt(-10.0, 3.0, 10.0,
			0.0, 0.0, 0.0,
			0.0, 0.0, 1.0);

	/*glCallList(1);

	// Swap buffers.
	if (glwindow->is_double_buffered())
		glwindow->swap_buffers();
	else
		glFlush();

	glwindow->gl_end();
	// *** OpenGL END ***
*/
	return true;
}

/*int GlDrawingArea::geom_ray_z_plane_intersect_3d(const point3d_t *eye,
		const point3d_t *ray_dir, double plane_z, point2d_t *result_xy)
{
	point3d_t plane_pt = { 0, 0, plane_z};
	point3d_t plane_normal = { 0, 0, 1};
	point3d_t plane_isect_point;
	double plane_point_dist;
	if (!geom_ray_plane_intersect_3d (ray_point, ray_dir, &plane_pt,
			&plane_normal, &plane_isect_point, &plane_point_dist) ||
			plane_point_dist <= 0) {
		return -1;
	}
	result_xy->x = plane_isect_point.x;
	result_xy->y = plane_isect_point.y;
	return 0;
}

bot_gtk_gl_drawing_area_set_context (self->gl_area);
double ray_start[3];
double ray_dir[3];
_window_coord_to_ray (event->x, widget->allocation.height - event->y, ray_start, ray_dir);

static int _window_coord_to_ray (double x, double y, double ray_start[3], double ray_dir[3])
{
    GLdouble model_matrix[16];
    GLdouble proj_matrix[16];
    GLint viewport[4];

    glGetDoublev (GL_MODELVIEW_MATRIX, model_matrix);
    glGetDoublev (GL_PROJECTION_MATRIX, proj_matrix);
    glGetIntegerv (GL_VIEWPORT, viewport);

    if (mygluUnProject (x, y, 0,
                      model_matrix, proj_matrix, viewport,
                      &ray_start[0], &ray_start[1], &ray_start[2]) == GL_FALSE)
        return -1;

    double ray_end[3];

    if (mygluUnProject (x, y, 1,
                      model_matrix, proj_matrix, viewport,
                      &ray_end[0], &ray_end[1], &ray_end[2]) == GL_FALSE)
        return -1;

    ray_dir[0] = ray_end[0] - ray_start[0];
    ray_dir[1] = ray_end[1] - ray_start[1];
    ray_dir[2] = ray_end[2] - ray_start[2];

    bot_vector_normalize_3d(ray_dir);

    return 0;
}*/

}
