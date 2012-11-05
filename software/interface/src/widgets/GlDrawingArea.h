/*
 * GlDrawingArea.h
 *
 *  Created on: Sep 11, 2012
 *      Author: ppetrova
 */

#include <gtkmm.h>
#include <gtkglmm.h>
#include <gdkmm/general.h>

#include <lcm/lcm_coretypes.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/shared_ptr.hpp>

#ifndef GLDRAWINGAREA_H_
#define GLDRAWINGAREA_H_

namespace gui {

class GlDrawingArea : public Gtk::DrawingArea, public Gtk::GL::Widget<GlDrawingArea> {
public:
	GlDrawingArea(boost::shared_ptr<lcm::LCM> &lcm);
	GlDrawingArea(const GlDrawingArea& other);
	virtual ~GlDrawingArea();
	void doRefresh();
	void invalidate();
/*	int GlDrawingArea::geom_ray_z_plane_intersect_3d(const point3d_t *eye,
			const point3d_t *ray_dir, double plane_z, point2d_t *result_xy);
*/
	void set_size_request(int width, int height);

	protected:

	//Overrides:
	virtual void on_realize();
	virtual bool on_configure_event(GdkEventConfigure* event);
	virtual bool on_expose_event(GdkEventExpose* event);
	virtual bool on_motion_notify_event(GdkEventMotion* event);
	virtual bool on_key_press_event(GdkEventKey *evt);
	virtual bool on_scroll_event(GdkEventScroll* event);
	virtual void setProjectionMatrix(int width, int height);

	boost::shared_ptr<lcm::LCM> _lcm;
	double zoomFactor;
};
}
#endif /* GLDRAWINGAREA_H_ */

