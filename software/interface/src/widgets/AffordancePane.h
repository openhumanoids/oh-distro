#ifndef GTKMM_AFFORDANCEPANE_H
#define GTKMM_AFFORDANCEPANE_H

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/signals.hpp>
#include <boost/signal.hpp>

#include <gtkmm.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include "otdf_parser/otdf_parser.h"

#include "AffordanceDrawingArea.h"

class AffordancePane : public Gtk::Window
{
public:
	// constructor/ destructor
	AffordancePane(boost::shared_ptr<lcm::LCM> _lcm, boost::shared_ptr<otdf::ModelInterface> _otdfObject);
	virtual ~AffordancePane();

	//void setOTDF(boost::shared_ptr<otdf::ModelInterface> otdfObject);
protected:
	//Signal handlers:
	void on_button_quit();

	//Child widgets:
	Gtk::VBox m_VBox;
	Gtk::HBox m_HBox;
	Gtk::VBox pairing;

	//gui::MainPane m_DrawingArea;
	gui::AffordanceDrawingArea* p_DrawingArea;		
	Gtk::HButtonBox m_ButtonBox;
	Gtk::Button m_Button_Quit;

private:

	boost::shared_ptr<otdf::ModelInterface> otdfObject;

};

#endif //GTKMM_AFFORDANCEPANE_H
