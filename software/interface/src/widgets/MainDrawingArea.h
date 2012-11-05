/*
 * MainDrawingArea.hpp
 *
 *Created on: Sep 25, 2012
 *Author: ppetrova
 */

#ifndef MAINDRAWINGAREA_HPP_
#define MAINDRAWINGAREA_HPP_

#include <gtkmm.h>
#include <gtkglmm.h>
#include <gdkmm/general.h>
#include <lcmtypes/drc_position_3d_t.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/shared_ptr.hpp>
#include "GlDrawingArea.h"
#include "RobotStateListener.h"

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>

namespace gui {

class MainDrawingArea : public GlDrawingArea {
public:
	MainDrawingArea(boost::shared_ptr<lcm::LCM> &lcm);
	//	MainDrawingArea(const MainDrawingArea& other);
	virtual ~MainDrawingArea();
	void polygon(int a, int b, int c , int d);
	void cube(void);
	boost::shared_ptr<otdf::Vector3> eye;
	boost::shared_ptr<otdf::Vector3> center;

protected:

	// Events we implement
	// Note that we could use gtkmm's "signals and slots" mechanism
	// instead, but for many classes there's a convenient member
	// function one just needs to define that'll be called with the
	// event.

	// Called when GL is first initialized
	//	void on_realize();
	// Called when our window needs to be redrawn
	bool on_expose_event(GdkEventExpose* event);
	// Called when a mouse button is pressed
	//virtual bool on_button_press_event(GdkEventButton* event);
	// Called when a mouse button is released
	//virtual bool on_button_release_event(GdkEventButton* event);
	// Called when the mouse moves
	//bool on_motion_notify_event(GdkEventMotion* event);


private:
	boost::shared_ptr<RobotStateListener> robotStateListener;

	class CanvasItem
	{
	public:
		CanvasItem(Gtk::Widget* canvas, Gtk::ToolButton* button, double x, double y)
		{
			this->pixbuf = (dynamic_cast <Gtk::Image*> (button->get_icon_widget()))->get_pixbuf();
			this->x = x;
			this->y = y;
		}

		Glib::RefPtr<Gdk::Pixbuf> pixbuf;
		double x, y;
	};

	void item_draw(const CanvasItem *item, const Cairo::RefPtr<Cairo::Context>& cr,	bool preview);

	virtual void on_drag_data_received(const Glib::RefPtr<Gdk::DragContext>& context,
			int x, int y, const Gtk::SelectionData& selection_data, guint info, guint time);
	virtual bool on_drag_motion(const Glib::RefPtr<Gdk::DragContext>& context, int x, int y, guint time);
	virtual bool on_drag_drop(const Glib::RefPtr<Gdk::DragContext>& context, int x, int y, guint time);
	virtual void on_drag_leave(const Glib::RefPtr<Gdk::DragContext>& context, guint time);

	bool m_drag_data_requested_for_drop; //So we know what to do in on_drag_data_received().
	CanvasItem* m_drop_item;

	typedef std::list<CanvasItem*> type_list_items;
	type_list_items m_canvas_items;
	//std::set<boost::shared_ptr<otdf::ModelInterface> otdfObject>> affordances;
};
}
#endif /* MAINDRAWINGAREA_HPP_ */
