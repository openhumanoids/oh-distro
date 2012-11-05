#include <stdio.h>
#include <iostream>
#include <inttypes.h>
#include <boost/bind.hpp>
#include <boost/signals.hpp>
#include <boost/ref.hpp>

#include "AffordancePane.h"
#include "twostatescale.h"
#include <lcmtypes/drc_lcmtypes.hpp>

using namespace std;
using namespace boost;

AffordancePane::AffordancePane(boost::shared_ptr<lcm::LCM> _lcm, boost::shared_ptr<otdf::ModelInterface> _otdfObject) : m_Button_Quit("Quit")
{
this->otdfObject = _otdfObject;

      std::cout << "Loading otdfObject : " << otdfObject->getName() << std::endl;
      std::map<std::string, double> params = otdfObject->params_map_; // all parameters

      stringstream title;
      title << "Affordance: " << otdfObject->getName();
      set_title(title.str().c_str());
      set_border_width(6);
      set_default_size(600, 400);
      add(m_HBox);

	p_DrawingArea = Gtk::manage(new gui::AffordanceDrawingArea(_otdfObject, _lcm));
	p_DrawingArea->set_size_request(400, 400);
	m_HBox.pack_start(*p_DrawingArea, Gtk::PACK_EXPAND_WIDGET);
	m_HBox.pack_start(m_VBox);

	for(std::map<string,double>::iterator ite = params.begin(); ite != params.end(); ++ite) {
	  string dofName =  ite->first;
	  double v = ite->second;

  	  Gtk::HBox* m_HBox_Pairing = Gtk::manage(new Gtk::HBox(true, 0));
	  Gtk::Label* dofNameLabel = Gtk::manage(new Gtk::Label(dofName, 0));
	  dofNameLabel->set_justify(Gtk::JUSTIFY_LEFT);

  	  m_HBox_Pairing->pack_start(*dofNameLabel, Gtk::PACK_EXPAND_WIDGET);
	  m_HBox_Pairing->pack_start(*Gtk::manage(new TwoStateScale(dofName, _lcm, p_DrawingArea)), Gtk::PACK_EXPAND_WIDGET);
	  m_VBox.pack_start(*m_HBox_Pairing, Gtk::PACK_EXPAND_PADDING);

	  //cout << "found param " << dofName << " (value: " << v << ")" << endl;
	  //int foo = otdfObject->params_map_[k].default_value;
	  // eg: <param name="Y" default_value="0.0" inc="0.01" min="-100" max="100"/>
	  //ignore value
	}

	m_VBox.show();
	m_HBox.show();

	show_all_children();
}

AffordancePane::~AffordancePane()
{
	//lcm.unsubscribe(robot_state_subscription);
}

void AffordancePane::on_button_quit()
{
	hide();
}
