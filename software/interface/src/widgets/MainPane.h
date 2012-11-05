#ifndef GTKMM_MAINPANE_H
#define GTKMM_MAINPANE_H

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

#include "MainDrawingArea.h"

namespace gui {

class MainPane : public Gtk::Window
{
public:
	// constructor/ destructor
	MainPane(boost::shared_ptr<lcm::LCM> _lcm);
	virtual ~MainPane();

protected:
	//Signal handlers:
	void on_button_quit();
	void on_treeview_row_activated(const Gtk::TreeModel::Path& path, Gtk::TreeViewColumn* column);

	//Child widgets:
	Gtk::VBox m_VBox;
	Gtk::HBox m_HBox;
	Gtk::VBox pairing;

	//gui::MainPane m_DrawingArea;
	gui::MainDrawingArea m_DrawingArea;
	Gtk::HButtonBox m_ButtonBox;
	Gtk::Button m_Button_Quit;

	// Tree model columns:
	class AffordanceModelColumns : public Gtk::TreeModel::ColumnRecord {
	public:
		AffordanceModelColumns() {
			add(m_col_id);
			add(m_col_type);
			add(m_col_name);
		}

		Gtk::TreeModelColumn<unsigned int>	m_col_id;
		Gtk::TreeModelColumn<Glib::ustring> m_col_type;
		Gtk::TreeModelColumn<Glib::ustring> m_col_name;
	};

	AffordanceModelColumns m_Columns;

	Gtk::ScrolledWindow m_ScrolledWindow;
	Gtk::TreeView* m_TreeView;
	Glib::RefPtr<Gtk::ListStore> m_refTreeModel;

private:

	// connection
	boost::signal<void (double actual)> sigToXScale, sigToYScale;
	boost::signal<void (double& x, double& y, double& z)> sigToGl;

	boost::signals::connection connectionToXScale, connectionToYScale;
	boost::signals::connection connectionToGL;

};
}

#endif //GTKMM_MAINPANE_H
