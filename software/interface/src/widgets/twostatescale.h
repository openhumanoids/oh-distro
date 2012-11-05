#ifndef GTKMM_CUSTOM_CONTAINER_MYCONTAINER_H
#define GTKMM_CUSTOM_CONTAINER_MYCONTAINER_H

#include <string>
#include <gtkmm.h>
#include <gtkmm/container.h>

#include <lcm/lcm_coretypes.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals.hpp>
#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include "AffordanceDrawingArea.h"
#include "markerwidget.h"

class TwoStateScale : public Gtk::Container
{
 public:
  TwoStateScale(std::string name, boost::shared_ptr<lcm::LCM> &lcm, gui::AffordanceDrawingArea* p_DrawingArea);
  virtual ~TwoStateScale();

  MarkerWidget actualSlider;
  Gtk::HScale commandedSlider;
  
  virtual void on_actual_slider_updated(double actual);

 protected:
  void set_child_widgets(Gtk::Widget& child_one, Gtk::Widget& child_two);

  //Overrides:
  virtual void on_size_request(Gtk::Requisition* requisition);
  virtual void on_size_allocate(Gtk::Allocation& allocation);
  virtual void forall_vfunc(gboolean include_internals, GtkCallback callback, gpointer callback_data);
  virtual void on_add(Gtk::Widget* child);
  virtual void on_remove(Gtk::Widget* child);
  virtual GType child_type_vfunc() const;

  void on_commanded_value_changed();

  double sliderMin;
  double sliderMax;

  Gtk::Widget* m_child_one;
  Gtk::Widget* m_child_two;

  std::string name;
  boost::shared_ptr<lcm::LCM> _lcm;

 private:
	boost::signal<void (std::string parameter_name, double delta)> sigParamChange;

};

#endif //GTKMM_CUSTOM_CONTAINER_MYCONTAINER_H
