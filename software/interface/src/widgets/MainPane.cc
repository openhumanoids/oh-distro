#include <stdio.h>
#include <iostream>
#include <inttypes.h>
#include <boost/bind.hpp>
#include <boost/signals.hpp>
#include <boost/ref.hpp>

#include "MainPane.h"
#include "twostatescale.h"
#include <lcmtypes/drc_lcmtypes.hpp>
#include "AffordanceToolPalette.h"

using namespace std;
using namespace boost;
using namespace Gtk;

namespace gui {

MainPane::MainPane(boost::shared_ptr<lcm::LCM> _lcm) : m_Button_Quit("Quit"), m_DrawingArea(_lcm)
{
	string title = "MainPane";
	set_title(title.c_str());
	set_border_width(6);
	set_size_request(800, 400);
	add(m_HBox);

	//m_DrawingArea.set_size_request(400, 400);
	m_HBox.pack_start(m_DrawingArea, PACK_EXPAND_WIDGET);
	Frame* m_AffordanceFrame = manage(new Frame);
	m_VBox.set_size_request(100,400);
	m_AffordanceFrame->set_label("Affordances");
	m_AffordanceFrame->set_label_align(ALIGN_START, ALIGN_START);
	m_AffordanceFrame->set_shadow_type(SHADOW_ETCHED_OUT);
	Notebook* m_AffordanceNotebook = manage(new Notebook());

	//Gtk::Image* cylinderIcon = Gtk::manage(new Gtk::Image("/home/drc/drc/software/drc-gui/images/cylinder.jpg"));

	// Create exising affordances tab and table

	// Add the TreeView, inside a ScrolledWindow
	m_TreeView = new TreeView();
	m_ScrolledWindow.add(*m_TreeView);

	// only show the scrollbars when necessary
	m_ScrolledWindow.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);

	//TODO: http://python-gtk-3-tutorial.readthedocs.org/en/latest/treeview.html

	// Create the tree Model
	m_refTreeModel = ListStore::create(m_Columns);
	m_TreeView->set_model(m_refTreeModel);

	// Fill the TreeView's model
	TreeModel::Row row = *(m_refTreeModel->append());
	row[m_Columns.m_col_id] = 1;
	row[m_Columns.m_col_type] = "cylinder";
	row[m_Columns.m_col_name] = "Affordance X";

	row = *(m_refTreeModel->append());
	row[m_Columns.m_col_id] = 3;
	row[m_Columns.m_col_type] = "ladder";
	row[m_Columns.m_col_name] = "Affordance B";

	/*row = *(m_refTreeModel->append());
	row[m_Columns.m_col_id] = 4;
	row[m_Columns.m_col_type] = "steering";
	row[m_Columns.m_col_name] = "Affordance B";
*/
	//Add the TreeView's view columns:
	// This number will be shown with the default numeric formatting
	m_TreeView->append_column("ID", m_Columns.m_col_id);
	m_TreeView->append_column("Type", m_Columns.m_col_type);
	m_TreeView->append_column("Name", m_Columns.m_col_name);

	// Make all columns reordable
	for(guint i = 0; i < 2; i++) {
		TreeView::Column* pColumn = m_TreeView->get_column(i);
		pColumn->set_reorderable();
	}

	m_TreeView->set_headers_clickable(true);
	//Connect signal:
	m_TreeView->signal_row_activated().connect(sigc::mem_fun(*this, &MainPane::on_treeview_row_activated) );

	m_AffordanceNotebook->append_page(m_ScrolledWindow, "Existing Affordances", false);

	VBox* newAffordancesBox = manage(new AffordanceToolPalette(&m_DrawingArea));
	m_AffordanceNotebook->append_page(*newAffordancesBox, "New Affordance", false);

	ToolPalette* affordancesPallete = manage(new ToolPalette());

	m_AffordanceFrame->add(*m_AffordanceNotebook);

	m_VBox.pack_start(*m_AffordanceFrame, PACK_EXPAND_WIDGET);
	m_HBox.pack_start(m_VBox, PACK_EXPAND_WIDGET);

	m_HBox.show();
	show_all_children();
}

void MainPane::on_treeview_row_activated(const Gtk::TreeModel::Path& path, Gtk::TreeViewColumn* /* column */)
{
	Gtk::TreeModel::iterator iter = m_refTreeModel->get_iter(path);
	if(iter)
	{
		Gtk::TreeModel::Row row = *iter;
		std::cout << "Row activated: ID=" << row[m_Columns.m_col_id] << ", Name=" << row[m_Columns.m_col_name] << std::endl;

		std::string file_name = row[m_Columns.m_col_type] + ".otdf\0";
		cout << "spawning affordance pane for otdf " << file_name << endl;

		GPid pid;
		GError *error[] = {NULL};
		gchar* child_argv[] = {"./affmain","affordance", const_cast<gchar *> (file_name.c_str()), NULL};
		gboolean ret = g_spawn_async_with_pipes("/home/drc/drc/software/drc-gui/pod-build/bin/", //NULL, 			// child's current working directory or NULL to inherit the parent's
				child_argv,	// child's argument vector [array zero-terminated=1]
				NULL,			// child's environment or NULL ot inherit the parent's
				(GSpawnFlags)G_SPAWN_DO_NOT_REAP_CHILD, // flags from GSpawnFlags
				NULL,			// function to run in the child just before exec(). [scope async][allow-none]
				NULL,			// user data for child_setup. [closure]
				&pid,			// return location for child process id
				NULL,			// return location for file descriptor to write to child's stdin or NULL
				NULL, 			// return location for file descriptor to read child's stdout or NULL
				NULL, 			// return location for file descriptor to read child's stderr or NULL
				error);
		// wait for any child process
		if( ! ret ) {
			cout << "ERROR " << error[0]->code << " : " << error[0]->message << endl;

			g_error( "SPAWN FAILED" );
		}

	}
}

MainPane::~MainPane()
{
	//lcm.unsubscribe(robot_state_subscription);
}

void MainPane::on_button_quit()
{
	hide();
}
}
