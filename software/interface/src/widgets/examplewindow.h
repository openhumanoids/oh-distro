#ifndef GTKMM_EXAMPLEWINDOW_H
#define GTKMM_EXAMPLEWINDOW_H

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

#include "AffordancePane.h"
#include "MainPane.h"

class ExampleWindow : public Gtk::Window
{
public:
	// constructor/ destructor
	ExampleWindow(boost::shared_ptr<lcm::LCM> _lcm, boost::shared_ptr<otdf::ModelInterface> _otdfObject);
	virtual ~ExampleWindow();

	//void setOTDF(boost::shared_ptr<otdf::ModelInterface> otdfObject);
protected:
	//Signal handlers:
	void on_button_quit();

	//Child widgets:
	Gtk::VBox m_VBox;
	Gtk::HBox m_HBox;
	Gtk::VBox pairing;

	//gui::MainPane m_DrawingArea;
	gui::AffordancePane m_DrawingArea;		
	Gtk::HButtonBox m_ButtonBox;
	Gtk::Button m_Button_Quit;

private:

	// connection
	boost::signal<void (double actual)> sigToXScale, sigToYScale;
	boost::signal<void (double& x, double& y, double& z)> sigToGl;

	boost::signals::connection connectionToXScale, connectionToYScale;
	boost::signals::connection connectionToGL;

	boost::shared_ptr<otdf::ModelInterface> otdfObject;

};

#endif //GTKMM_EXAMPLEWINDOW_H
