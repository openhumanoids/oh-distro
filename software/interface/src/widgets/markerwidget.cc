#include "markerwidget.h"
#include <gdkmm/drawable.h>
#include <gdkmm/general.h>  // for cairo helper functions
#include <iostream>
//#include <gtk/gtkwidget.h> //For GTK_IS_WIDGET()
#include <cstring>


MarkerWidget::MarkerWidget(std::string _name) :
  Glib::ObjectBase("markerwidget"),
  Gtk::Widget(),
  m_scale(1000)
{
  set_has_window(true);
  name = _name;
  //This shows the GType name, which must be used in the RC file.
 // std::cout << "GType name: " << G_OBJECT_TYPE_NAME(gobj()) << std::endl;

  //This show that the GType still derives from GtkWidget:
  //std::cout << "Gtype is a GtkWidget?:" << GTK_IS_WIDGET(gobj()) << std::endl;

  //Install a style so that an aspect of this widget may be themed via an RC
  //file: 
  gtk_widget_class_install_style_property(GTK_WIDGET_CLASS(
							   G_OBJECT_GET_CLASS(gobj())), 
					  g_param_spec_int(name.c_str(), //"example_scale",
							   "Scale of Example Drawing",
							   "The scale to use when drawing. This is just a silly example.",
							   G_MININT,
							   G_MAXINT,
							   0,
							   G_PARAM_READABLE) );

  gtk_rc_parse("custom_gtkrc");
}

MarkerWidget::~MarkerWidget()
{
}

void MarkerWidget::setActual(double percentage) { 
  // LCM updates come in here
  actualAtPercentage = percentage;
  on_expose_event(NULL);
}

double MarkerWidget::getCommanded() {
  return markAtPercentage;
}

void MarkerWidget::setCommanded(double percentage) {
  markAtPercentage = percentage; 
  on_expose_event(NULL);
}

void MarkerWidget::on_size_request(Gtk::Requisition* requisition)
{
  //Initialize the output parameter:
  *requisition = Gtk::Requisition();

  //Discover the total amount of minimum space needed by this widget.
  requisition->height = getMinHeight();
  requisition->width = 10;
}

int MarkerWidget::getMinHeight() {
  return 10;
}

void MarkerWidget::on_size_allocate(Gtk::Allocation& allocation)
{
  //Do something with the space that we have actually been given:
  //(We will not be given heights or widths less than we have requested, though
  //we might get more)

  //Use the offered allocation for this container:
  set_allocation(allocation);

  if(m_refGdkWindow)
    {
      m_refGdkWindow->move_resize( allocation.get_x(), allocation.get_y(),
				   allocation.get_width(), allocation.get_height() );
    }
}

void MarkerWidget::on_map()
{
  //Call base class:
  Gtk::Widget::on_map();
}

void MarkerWidget::on_unmap()
{
  //Call base class:
  Gtk::Widget::on_unmap();
}

void MarkerWidget::on_realize()
{
  //Do not call base class Gtk::Widget::on_realize().
  //It's intended only for widgets that set_has_window(false).

  set_realized();
  ensure_style();

  //Get the themed style from the RC file:
  get_style_property(name.c_str(), m_scale);
//  std::cout << "m_scale (example_scale from the theme/rc-file) is: " << m_scale << std::endl; 

  if(!m_refGdkWindow)
    {
      //Create the GdkWindow:

      GdkWindowAttr attributes;
      memset(&attributes, 0, sizeof(attributes));

      Gtk::Allocation allocation = get_allocation();

      //Set initial position and size of the Gdk::Window:
      attributes.x = allocation.get_x();
      attributes.y = allocation.get_y();
      attributes.width = allocation.get_width();
      attributes.height = allocation.get_height();

      attributes.event_mask = get_events () | Gdk::EXPOSURE_MASK; 
      attributes.window_type = GDK_WINDOW_CHILD;
      attributes.wclass = GDK_INPUT_OUTPUT;

      m_refGdkWindow = Gdk::Window::create(get_parent_window(), &attributes,
					   GDK_WA_X | GDK_WA_Y);
      set_window(m_refGdkWindow);

      //Attach this widget's style to its Gdk::Window.
      style_attach();

      //set colors
      //modify_bg(Gtk::STATE_NORMAL , Gdk::Color("white"));
      modify_fg(Gtk::STATE_NORMAL , Gdk::Color("black"));

      //make the widget receive expose events
      m_refGdkWindow->set_user_data(gobj());
    }
}

void MarkerWidget::on_unrealize()
{
  m_refGdkWindow.reset();

  //Call base class:
  Gtk::Widget::on_unrealize();
}

bool MarkerWidget::on_expose_event(GdkEventExpose* event)
{
  if(m_refGdkWindow)
    {
      double scale_x = (double)get_allocation().get_width() / m_scale;
      double scale_y = (double)get_allocation().get_height() / m_scale;

      Cairo::RefPtr<Cairo::Context> cr = m_refGdkWindow->create_cairo_context();

      if(event)
	{
	  // clip to the area that needs to be re-exposed so we don't draw any
	  // more than we need to.
	  cr->rectangle(event->area.x, event->area.y,
			event->area.width, event->area.height);
	  cr->clip();
	}

      
      int widget_width = get_allocation().get_width();
      int widget_height = get_allocation().get_width();

      // paint the background
      Gdk::Cairo::set_source_color(cr, get_style()->get_bg(Gtk::STATE_NORMAL));
      cr->paint();

      //Gdk::Cairo::set_source_color(cr, Gdk::Color("green"));

      int marker_y = 10;
      int marker_x = 10;
      int marker_size;
      bool vertical_orientation = false;
      if (vertical_orientation) {
	marker_y = (int)(widget_height * actualAtPercentage);
	marker_size = widget_width;
      } else {
	marker_x = (int)(widget_width * actualAtPercentage);
	marker_size = widget_height;
      }

      bool draw_delta = (markAtPercentage != actualAtPercentage);
      if (draw_delta) {
	if (vertical_orientation) {
	  // TODO
	} else {
	  int actual_x = (int)(widget_height * markAtPercentage);
	  Gdk::Cairo::set_source_color(cr, Gdk::Color("red"));
	  cr->move_to(marker_x, marker_y);
	  cr->line_to(marker_x, marker_y - 3); //(marker_size / 5));
	  cr->line_to(actual_x, marker_y - 3); //(marker_size / 5));
	  cr->line_to(actual_x, marker_y);
	  cr->close_path();
	  cr->fill();
	}
      }

      // the foreground
      Gdk::Cairo::set_source_color(cr, get_style()->get_fg(Gtk::STATE_NORMAL));

      if (vertical_orientation) {
	cr->move_to(marker_x - marker_size, marker_y - (marker_size / 2));
	cr->line_to(marker_x, marker_y);
	cr->line_to(marker_x - marker_size, marker_y + (marker_size / 2));
	cr->move_to(marker_x - marker_size, marker_y - (marker_size / 2));
      } else {
	cr->move_to(marker_x - (marker_size / 2), marker_y - marker_size);
	cr->line_to(marker_x, marker_y);
	cr->line_to(marker_x + (marker_size / 2), marker_y - marker_size);
	cr->move_to(marker_x - (marker_size / 2), marker_y - marker_size);
      }

      cr->close_path();
      cr->fill();

    }

  return true;
}
