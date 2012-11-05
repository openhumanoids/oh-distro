#include <iostream>
#include "twostatescale.h"
using namespace std;

TwoStateScale::TwoStateScale(std::string name, boost::shared_ptr<lcm::LCM> &lcm, gui::AffordanceDrawingArea* m_DrawingArea)
: commandedSlider(-50.0, 50.0, 1.0), actualSlider(name), _lcm(lcm)
{

	//lcm ok?
	if(!_lcm->good())
	{
		cerr << "\nLCM Not Good: TwoStateScale(" << name << ")" << endl;
		return;
	} else {
		//cout << "LCM is good TwoStateScale(" << name << ")" << endl;
	}

	this->name = name;

	set_has_window(false);
	set_redraw_on_allocate(false);

	sliderMax = 50.0;
	sliderMin = -50.0;

	actualSlider.setCommanded(0.0);
	commandedSlider.set_draw_value(false);
	commandedSlider.set_value(0.0);
	commandedSlider.signal_value_changed().connect(
			sigc::mem_fun(*this, &TwoStateScale::on_commanded_value_changed));

	sigParamChange.connect(bind(&gui::AffordanceDrawingArea::on_param_change, m_DrawingArea, _1, _2));
	set_child_widgets(actualSlider, commandedSlider);
}

TwoStateScale::~TwoStateScale()
{
}

void TwoStateScale::on_commanded_value_changed() {
  double oldValue = actualSlider.getCommanded();
  double newValue = commandedSlider.get_value()/(sliderMax*2.0);

  actualSlider.setCommanded(newValue);
  //actualSlider.setActual(commandedSlider.get_value() / sliderMax - 0.1); // TODO REMOVE -0.1, demo only
  
 /*
  drc::affordance_parameter_t msg;
  msg.parameter_name = name;
  msg.value = newValue - oldValue;
  _lcm->publish("AFF_PARAM", &msg);
 */
  sigParamChange(name, newValue - oldValue);
}

void TwoStateScale::on_actual_slider_updated(double actual)
{
	actualSlider.setActual(actual);
}


void TwoStateScale::set_child_widgets(Gtk::Widget& child_one,
		Gtk::Widget& child_two)
{
	m_child_one = &child_one;
	m_child_two = &child_two;

	m_child_one->set_parent(*this);
	m_child_two->set_parent(*this);
}


void TwoStateScale::on_size_request(Gtk::Requisition* requisition)
{
	//Initialize the output parameter:
	*requisition = Gtk::Requisition();

	//Discover the total amount of minimum space needed by this container widget,
	//by examining its child widgets.  The layouts in this custom container will
	//be arranged vertically, one above the other.

	Gtk::Requisition child_requisition_one = {0, 0};
	Gtk::Requisition child_requisition_two = {0, 0};
	if(m_child_one && m_child_one->get_visible())
		child_requisition_one = m_child_one->size_request();

	if(m_child_two && m_child_two->get_visible())
		child_requisition_two = m_child_two->size_request();

	//See which one has the most width:
	int max_width = MAX(child_requisition_one.width,
			child_requisition_two.width);

	//Add the heights together:
	int total_height = child_requisition_one.height +
			child_requisition_two.height;

	//Request the width for this container based on the sizes requested by its
	//child widgets:
	requisition->height = total_height;
	requisition->width = max_width;
}

void TwoStateScale::on_size_allocate(Gtk::Allocation& allocation)
{
	//Do something with the space that we have actually been given:
	//(We will not be given heights or widths less than we have requested, though
	//we might get more)

	//Use the offered allocation for this container:
	set_allocation(allocation);

	//Assign sign space to the child:
	Gtk::Allocation child_allocation_one, child_allocation_two;

	//Place the first child at the top-left,
	child_allocation_one.set_x( allocation.get_x() );
	child_allocation_one.set_y( allocation.get_y() );

	//Make it take up the full width available:
	child_allocation_one.set_width( allocation.get_width() );

	//Make it take up half the height available:
	child_allocation_one.set_height(actualSlider.getMinHeight()); //allocation.get_height() / 2);

	if(m_child_one && m_child_one->get_visible())
		m_child_one->size_allocate(child_allocation_one);

	//Place the second child below the first child:
	child_allocation_two.set_x( allocation.get_x() );
	child_allocation_two.set_y( allocation.get_y() +
			child_allocation_one.get_height());

	//Make it take up the full width available:
	child_allocation_two.set_width( allocation.get_width() );

	//Make it take up half the height available:
	child_allocation_two.set_height( allocation.get_height() -
			child_allocation_one.get_height());

	if(m_child_two && m_child_two->get_visible())
		m_child_two->size_allocate(child_allocation_two);
}

void TwoStateScale::forall_vfunc(gboolean, GtkCallback callback, gpointer callback_data)
{
	if(m_child_one)
		callback(m_child_one->gobj(), callback_data);

	if(m_child_two)
		callback(m_child_two->gobj(), callback_data);
}

void TwoStateScale::on_add(Gtk::Widget* child)
{
	if(!m_child_one)
	{
		m_child_one = child;
		m_child_one->set_parent(*this);
	}
	else if(!m_child_two)
	{
		m_child_two = child;

		m_child_two->set_parent(*this);
	}
}

void TwoStateScale::on_remove(Gtk::Widget* child)
{
	if(child)
	{
		const bool visible = child->get_visible();
		bool found = false;

		if(child == m_child_one)
		{
			m_child_one = 0;
			found = true;
		}
		else if(child == m_child_two)
		{
			m_child_two = 0;
			found = true;
		}

		if(found)
		{
			child->unparent();

			if(visible)
				queue_resize();
		}
	}
}

GType TwoStateScale::child_type_vfunc() const
{
	//If there is still space for one widget, then report the type of widget that
	//may be added.
	if(!m_child_one || !m_child_two)
		return Gtk::Widget::get_type();
	else
	{
		//No more widgets may be added.
		return G_TYPE_NONE;
	}
}
