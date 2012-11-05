/*
 * MainDrawingArea.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: ppetrova
 */

#include <iostream>
#include <gdkmm/rectangle.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <boost/function.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <bot_vis/bot_vis.h>
#include <bot_core/rotations.h>
#include <gdk/gdkkeysyms.h>
#include <boost/shared_ptr.hpp>

#include "MainDrawingArea.h"


using namespace boost;
using namespace std;

namespace gui {

MainDrawingArea::MainDrawingArea(boost::shared_ptr<lcm::LCM> &lcm) : GlDrawingArea(lcm), m_drag_data_requested_for_drop(false), m_drop_item()
{

	std::cout << "Main Pane OK!" << std::endl;
	robotStateListener = boost::shared_ptr<RobotStateListener>(new RobotStateListener(_lcm, this));
	eye = boost::shared_ptr<otdf::Vector3>(new otdf::Vector3(10, 10, 2));
	center = boost::shared_ptr<otdf::Vector3>(new otdf::Vector3(0, 0, 0));

	//Gtk::Widget::add_events(Gdk::BUTTON_MOTION_MASK);
	//signal_motion_notify_event().connect(sigc::mem_fun(*this, &MainDrawingArea::on_motion_notify_event));
}


/*void mouseWheel(int button, int state, int x, int y)
{
	std::cout << "In Mousewheel" << endl;
	if ((button == 3) || (button == 4))
	{
		if (state == GLUT_UP)
			return;

		if (button == 3) {
			// Up
			std::cout << "Zoom in" << endl;
		} else {
			// Down
			std::cout << "Zoom out" << endl;
		}
	} else {
		std::cout << "Regular button click." << endl;
	}
}
 */
//MainDrawingArea::MainDrawingArea(const MainDrawingArea& other) { }

MainDrawingArea::~MainDrawingArea() { }

static void draw(shared_ptr<urdf::Geometry> link, const drc::link_transform_t &nextTf)
{

	//--get rotation in angle/axis form
	double theta;
	double axis[3];
	double quat[4] = {nextTf.tf.rotation.w,
			nextTf.tf.rotation.x,
			nextTf.tf.rotation.y,
			nextTf.tf.rotation.z};
	bot_quat_to_angle_axis(quat, &theta, axis);

	//---debugging
	/* cout << "\n(w,x,y,z) = ("
       << nextTf.tf.rotation.w
       << "," << nextTf.tf.rotation.x
       << "," << nextTf.tf.rotation.y
       << "," << nextTf.tf.rotation.z
       << ")" << endl;
  cout << "\naxis = ("
       << axis[0] << ","
       << axis[1] << ","
       << axis[2] << ")" << endl;
  cout << "theta = " << theta << endl;*/

	//----

	GLUquadricObj* quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_FILL);
	gluQuadricNormals(quadric, GLU_SMOOTH);
	gluQuadricOrientation(quadric, GLU_OUTSIDE);


	int type = link->type ;
	enum {SPHERE, BOX, CYLINDER, MESH};

	if (type == SPHERE)
	{

		//		cout << "Drawing SPHERE" << endl;
		glPushMatrix();
		shared_ptr<urdf::Sphere> sphere(shared_dynamic_cast<urdf::Sphere>(link));
		double radius = sphere->radius;
		glPointSize(radius);
		//glColor3ub(0,1,0);
		glBegin(GL_POINTS);
		glVertex3f(radius, radius, radius);
		glEnd();
		glPopMatrix();
	}
	else if  (type == BOX)
	{
		shared_ptr<urdf::Box> box(shared_dynamic_cast<urdf::Box>(link));
		double xDim = box->dim.x;
		double yDim = box->dim.y;
		double zDim = box->dim.z;

		//todo
		glPushMatrix();
		//size cuboid

		// move base up so that bottom face is at origin
		// glTranslatef(0,0.5,0.0);
		glTranslatef(nextTf.tf.translation.x,
				nextTf.tf.translation.y,
				nextTf.tf.translation.z);

		glRotatef(theta * 180/3.141592654,
				axis[0], axis[1], axis[2]);
		glScalef(xDim,yDim,zDim);
		bot_gl_draw_cube();
		//cube();
		glPopMatrix();


	}
	else if  (type == CYLINDER)
	{

		shared_ptr<urdf::Cylinder> cyl(shared_dynamic_cast<urdf::Cylinder>(link));
		/*glPointSize(10.0f);
      glColor3ub(0,1,0);
      glBegin(GL_POINTS);
      glVertex3f(nextTf.tf.translation.x,
	       nextTf.tf.translation.y,
	       nextTf.tf.translation.z);
	       glEnd();*/

		glPushMatrix();
		double v[] = {0,0, -cyl->length/2.0};
		double result[3];
		bot_quat_rotate_to(quat,v,result);

		// Translate tf origin to cylinder centre
		glTranslatef(result[0],result[1],result[2]);

		glTranslatef(nextTf.tf.translation.x,
				nextTf.tf.translation.y,
				nextTf.tf.translation.z);

		glRotatef(theta * 180/3.141592654,
				axis[0], axis[1], axis[2]);

		gluCylinder(quadric,
				cyl->radius,
				cyl->radius,
				(double) cyl->length,
				36,
				1);

		//gluDeleteQuadric(quadric);
		glPopMatrix();

		// drawing two disks to make a SOLID cylinder
		glPushMatrix();

		v[2] = -(cyl->length/2.0);
		bot_quat_rotate_to(quat,v,result);

		// Translate tf origin to cylinder centre
		glTranslatef(result[0],result[1],result[2]);
		glTranslatef(nextTf.tf.translation.x,
				nextTf.tf.translation.y,
				nextTf.tf.translation.z);
		glRotatef(theta * 180/3.141592654,
				axis[0], axis[1], axis[2]);
		gluDisk(quadric,
				0,
				cyl->radius,
				36,
				1);
		glPopMatrix();
		glPushMatrix();

		v[2] = (cyl->length/2.0);
		bot_quat_rotate_to(quat,v,result);

		// Translate tf origin to cylinder centre
		glTranslatef(result[0],result[1],result[2]);
		glTranslatef(nextTf.tf.translation.x,
				nextTf.tf.translation.y,
				nextTf.tf.translation.z);
		glRotatef(theta * 180/3.141592654,
				axis[0], axis[1], axis[2]);
		gluDisk(quadric,
				0,
				cyl->radius,
				36,
				1);
		glPopMatrix();

		//cout << "radius : "<<  cyl->radius << endl;
		//cout << "length : "<<  cyl->length << endl;
		// drawBox(radius,length, it->second -> visual->origin);
	}
	else if  (type == MESH)
	{
		cout << "Drawing MESH" << endl;
		//cout << "MESH"<< endl;
		//shared_ptr<urdf::Mesh> mesh(shared_dynamic_cast<urdf::Mesh>(it->second->visual->geometry));
		//renderMesh(mesh->filename)
	}
	else {
		//cout << "UNKNOWN"<< endl;
	}

	gluDeleteQuadric(quadric);
}

bool MainDrawingArea::on_expose_event(GdkEventExpose* event)
{

	GlDrawingArea::on_expose_event(event);

	// Get GL::Window.
	Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();
	if (!glwindow)
		return false;

	// GL calls.

	// *** OpenGL BEGIN ***
	if (!glwindow->gl_begin(get_gl_context()))
		return false;

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// viewer stuff
	glEnable(GL_DEPTH_TEST);

	vector<shared_ptr<urdf::Geometry> > link_shapes;
	vector<drc::link_transform_t> link_tfs;

	robotStateListener->getState(link_shapes, link_tfs);

	//RobotStateListener::printTransforms(link_shapes, link_tfs);

	//-draw
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	double c[3] = {0.9,0.2,0.3};
	double alpha = 1;
	glColor4f(c[0],c[1],c[2],alpha);

	for(uint i = 0; i < link_tfs.size(); i++)
	{
		drc::link_transform_t nextTf = link_tfs[i];
		shared_ptr<urdf::Geometry> nextLink = link_shapes[i];
		draw(nextLink, nextTf);
	}

	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClearDepth(1.0);

	// move camera if the robot has moved
	///glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	//gluLookAt( eyeX, eyeY, eyeZ, CenterX, CenterY, CenterZ, UpVectorX, UpVectorY, UpVectorZ)
	/*gluLookAt(10.0, 0.0, 10.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1.0);

	glViewport(0, 0, get_width(), get_height());
*/
	glCallList(1);

	// Swap buffers.
	if (glwindow->is_double_buffered())
		glwindow->swap_buffers();
	else
		glFlush();

	glwindow->gl_end();
	// *** OpenGL END ***

	// This is where we draw on the window
	Glib::RefPtr<Gdk::Window> window = get_window();
	if(!window)
		return false;

	/*
	 Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
	const Gdk::Region region(event->region, true );
	Gdk::Cairo::add_region_to_path(cr, region);
	cr->clip();

	cr->set_source_rgb(1.0, 1.0, 1.0);
	const Gtk::Allocation allocation = get_allocation();
	cr->rectangle(0, 0, allocation.get_width(), allocation.get_height());
	cr->fill();
*/
	for(type_list_items::iterator iter = m_canvas_items.begin();
			iter != m_canvas_items.end(); ++iter )
	{
		//item_draw(*iter, cr, false);
	}

	if(m_drop_item)
		std::cout << "DRAWING item" << std::endl;
		//item_draw (m_drop_item, cr, true);

	return true;

}

void MainDrawingArea::polygon(int a, int b, int c , int d)
{


	float vertices[][3] =
	{
			{-0.5,-0.5,-0.5},{0.5,-0.5,-0.5},
			{0.5,0.5,-0.5}, {-0.5,0.5,-0.5}, {-0.5,-0.5,0.5},
			{0.5,-0.5,0.5}, {0.5,0.5,0.5}, {-0.5,0.5,0.5}
	};

	float colors[][3] = {{0.0,0.5,0.5},{1.0,0.0,0.0},
			{1.0,1.0,0.0}, {0.0,1.0,0.0}, {0.0,0.0,1.0},
			{1.0,0.0,1.0}, {1.0,1.0,1.0}, {0.0,1.0,1.0}};

	// draw a polygon using colour of first vertex

	glBegin(GL_POLYGON);
	// glColor3fv(colors[a]);
	//glColor3f(0.15,0.15,0.15);
	glVertex3fv(vertices[a]);
	glVertex3fv(vertices[b]);
	glVertex3fv(vertices[c]);
	glVertex3fv(vertices[d]);
	glEnd();
}

void MainDrawingArea::cube(void)
{
	//Draw unit cube centred on the origin

	/* map vertices to faces */

	polygon(0,3,2,1);
	polygon(2,3,7,6);
	polygon(4,7,3,0);
	polygon(1,2,6,5);
	polygon(7,4,5,6);
	polygon(5,4,0,1);
}

void MainDrawingArea::item_draw(const CanvasItem *item,
		const Cairo::RefPtr<Cairo::Context>& cr,
		bool preview)
{

	//std::cout <<"IN item draw"<< std::endl;

	if(!item || !item->pixbuf)
		return;

	const double cx = 50;//item->pixbuf->get_width();
	const double cy = 50;//item->pixbuf->get_height();

	Gdk::Cairo::set_source_pixbuf(cr,
			item->pixbuf,
			item->x - cx * 0.5, item->y - cy * 0.5);

	if(preview) {
		std::cout << " preview " << std::endl;
		cr->paint_with_alpha(0.6);
	} else {
		std::cout << "paint" << std::endl;
		cr->paint();
	}
}

bool MainDrawingArea::on_drag_motion(const Glib::RefPtr<Gdk::DragContext>& context,
		int x, int y, guint time)
{
	std::cout <<"IN drag motion"<< std::endl;

	m_drag_data_requested_for_drop = false; //It's for drag-motion instead.

	if(m_drop_item)
	{
		// We already have a drop indicator so just update its position.

		m_drop_item->x = x;
		m_drop_item->y = y;

		queue_draw();
		context->drag_status(Gdk::ACTION_COPY, time);
	}
	else
	{
		// Request DnD data for creating a drop indicator.
		// This will cause on_drag_data_received() to be called.
		const Glib::ustring target = drag_dest_find_target(context);

		if (target.empty())
			return false;

		drag_get_data(context, target, time);
	}

	Gtk::DrawingArea::on_drag_motion(context, x, y, time);
	return true;
}


void MainDrawingArea::on_drag_data_received(const Glib::RefPtr<Gdk::DragContext>& context, int x, int y, const Gtk::SelectionData& selection_data, guint info, guint time)
{

	std::cout <<"IN drag data received"<< std::endl;

	// Find the tool button which is the source of this DnD operation.
	Gtk::Widget* widget = drag_get_source_widget(context);

	Gtk::ToolPalette* drag_palette = dynamic_cast<Gtk::ToolPalette*>(widget);
	while(widget && !drag_palette)
	{
		widget = widget->get_parent();
		drag_palette = dynamic_cast<Gtk::ToolPalette*>(widget);
	}

	Gtk::ToolItem* drag_item = 0;
	if(drag_palette)
		drag_item = drag_palette->get_drag_item(selection_data);

	// Create a drop indicator when a tool button was found:
	Gtk::ToolButton* button = dynamic_cast<Gtk::ToolButton*>(drag_item);
	if(!button)
		return;

	if(m_drop_item)
	{
		delete m_drop_item;
		m_drop_item = 0;
	}

	CanvasItem* item = new CanvasItem(this, button, x, y);

	if(m_drag_data_requested_for_drop)
	{
		m_canvas_items.push_back(item);

		// Signal that the item was accepted and then redraw.
		context->drag_finish(true /* success */, false /* del */, time);
	}
	else
	{
		m_drop_item = item;

		// We are getting this data due to a request in drag_motion,
		// rather than due to a request in drag_drop, so we are just
		// supposed to call gdk_drag_status (), not actually paste in
		// the data.
		context->drag_status(Gdk::ACTION_COPY, time);
	}

	queue_draw();

	Gtk::DrawingArea::on_drag_data_received(context, x, y, selection_data, info, time);
}


bool MainDrawingArea::on_drag_drop(const Glib::RefPtr<Gdk::DragContext>& context, int /* x */, int /* y */, guint time)
{

	std::cout <<"IN drag draw"<< std::endl;

	// Request DnD data for creating a dopped item.
	// This will cause on_drag_data_received() to be called.
	const Glib::ustring target = drag_dest_find_target(context);

	if (target.empty())
		return false;

	m_drag_data_requested_for_drop = true;
	drag_get_data(context, target, time);

	return true;
}

void MainDrawingArea::on_drag_leave(const Glib::RefPtr<Gdk::DragContext>& context, guint time)
{

	std::cout <<"IN drag leave"<< std::endl;

	//This signal is emitted to clean up the item used for drag-motion,
	//either when the cursor moves out of the widget or when we drop.

	if(!m_drop_item)
		return;

	delete m_drop_item;
	m_drop_item = 0;

	queue_draw();

	Gtk::DrawingArea::on_drag_leave(context, time);
}

}
