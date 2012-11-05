#include "AffordanceToolPalette.h"
#include <iostream>
#include <exception>

using namespace Gtk;

static bool sort_predicate(const StockID& a, const StockID& b)
{
	return a.get_string() < b.get_string();
}


void AffordanceToolPalette::load_stock_items()
{
	ToolItemGroup* group = manage(new ToolItemGroup("Affordances"));
	m_ToolPalette.add(*group);

	try {
		//Glib::RefPtr<Gdk::Pixbuf> image = Gdk::Pixbuf::create_from_file("/home/drc/drc/software/drc-gui/images/cylinder.jpg");
		Image* cylinderIcon = manage(new Image("/home/drc/drc/software/drc-gui/images/cylinder.png"));
		ToolButton* button = manage(new ToolButton(*cylinderIcon));
		button->set_tooltip_text("Cylinder");
		button->set_is_important();
		button->set_label("Cylinder");
		group->insert(*button);

		Image* leverIcon = manage(new Image("/home/drc/drc/software/drc-gui/images/lever.png"));
		button = manage(new ToolButton(*leverIcon));
		button->set_tooltip_text("Lever");
		button->set_is_important();
		button->set_label("Lever");
		group->insert(*button);

		Image* wheelIcon = manage(new Image("/home/drc/drc/software/drc-gui/images/wheel.png"));
		button = manage(new ToolButton(*wheelIcon));
		button->set_tooltip_text("Wheel");
		button->set_is_important();
		button->set_label("Wheel");
		group->insert(*button);


		Image* ladderIcon = manage(new Image("/home/drc/drc/software/drc-gui/images/ladder.png"));
		button = manage(new ToolButton(*ladderIcon));
		button->set_tooltip_text("Ladder");
		button->set_is_important();
		button->set_label("Ladder");
		group->insert(*button);

	} catch (Glib::FileError & e){
		std::cout << e.what() << std::endl;

	}

}

AffordanceToolPalette::AffordanceToolPalette(Gtk::DrawingArea* canvas) : VBox(false, 6)
{

	m_Canvas = canvas;
	//The Orientation ComboBox:
	m_refTreeModelOrientation = ListStore::create(m_ColumnsOrientation);
	TreeModel::Row row = *(m_refTreeModelOrientation->append());
	row[m_ColumnsOrientation.m_col_value] = ORIENTATION_HORIZONTAL;
	row[m_ColumnsOrientation.m_col_name] = "Horizontal";\
	row = *(m_refTreeModelOrientation->append());
	row[m_ColumnsOrientation.m_col_value] = ORIENTATION_VERTICAL;
	row[m_ColumnsOrientation.m_col_name] = "Vertical";
	m_ComboOrientation.set_model(m_refTreeModelOrientation);
	this->pack_start(m_ComboOrientation, PACK_SHRINK);
	m_ComboOrientation.pack_start(m_ColumnsOrientation.m_col_name);
	m_ComboOrientation.signal_changed().connect(sigc::mem_fun(*this, &AffordanceToolPalette::on_combo_orientation_changed) );
	m_ComboOrientation.set_active(row);

	//The Style ComboBox:
	m_refTreeModelStyle = ListStore::create(m_ColumnsStyle);
	row = *(m_refTreeModelStyle->append());
	row[m_ColumnsStyle.m_col_value] = TOOLBAR_TEXT;
	row[m_ColumnsStyle.m_col_name] = "Text";\
	row = *(m_refTreeModelStyle->append());
	row[m_ColumnsStyle.m_col_value] = TOOLBAR_BOTH;
	row[m_ColumnsStyle.m_col_name] = "Both";
	row = *(m_refTreeModelStyle->append());
	row[m_ColumnsStyle.m_col_value] = TOOLBAR_BOTH_HORIZ;
	row[m_ColumnsStyle.m_col_name] = "Both: Horizontal";
	row = *(m_refTreeModelStyle->append());
	row[m_ColumnsStyle.m_col_value] = TOOLBAR_ICONS;
	row[m_ColumnsStyle.m_col_name] = "Icons";
	row = *(m_refTreeModelStyle->append());
	row[m_ColumnsStyle.m_col_value] = -1; // A custom meaning for this demo.
	row[m_ColumnsStyle.m_col_name] = "Default";
	m_ComboStyle.set_model(m_refTreeModelStyle);
	this->pack_start(m_ComboStyle, PACK_SHRINK);
	m_ComboStyle.pack_start(m_ColumnsStyle.m_col_name);
	m_ComboStyle.signal_changed().connect(sigc::mem_fun(*this, &AffordanceToolPalette::on_combo_style_changed) );
	m_ComboStyle.set_active(row);

	//Add and fill the ToolPalette:
	load_stock_items();


	m_ScrolledWindowPalette.set_policy(POLICY_NEVER, POLICY_AUTOMATIC);
	m_ScrolledWindowPalette.set_border_width(6);
	m_ScrolledWindowPalette.add(m_ToolPalette);
	this->pack_start(m_ScrolledWindowPalette, PACK_EXPAND_WIDGET);

	on_combo_orientation_changed();
/*
	m_ScrolledWindowCanvas.set_policy(POLICY_AUTOMATIC, POLICY_ALWAYS);
	m_ScrolledWindowCanvas.set_border_width(6);
	m_ScrolledWindowCanvas.add(m_Canvas);
	m_ScrolledWindowCanvas.set_size_request(200, -1);
	m_HBox.pack_start(m_ScrolledWindowCanvas);
*/
	m_ToolPalette.add_drag_dest(*m_Canvas, DEST_DEFAULT_HIGHLIGHT, TOOL_PALETTE_DRAG_ITEMS, Gdk::ACTION_COPY);

	show_all_children();
}

AffordanceToolPalette::~AffordanceToolPalette()
{
}

void AffordanceToolPalette::on_combo_orientation_changed()
{
	TreeModel::iterator iter = m_ComboOrientation.get_active();
	if(!iter)
		return;

	TreeModel::Row row = *iter;
	const Orientation value = row[m_ColumnsOrientation.m_col_value];

	m_ToolPalette.set_orientation(value);

	if(value == ORIENTATION_HORIZONTAL)
		m_ScrolledWindowPalette.set_policy(POLICY_AUTOMATIC, POLICY_NEVER);
	else
		m_ScrolledWindowPalette.set_policy(POLICY_NEVER, POLICY_AUTOMATIC);
}

void AffordanceToolPalette::on_combo_style_changed()
{
	TreeModel::iterator iter = m_ComboStyle.get_active();
	if(!iter)
		return;

	TreeModel::Row row = *iter;
	const int value = row[m_ColumnsStyle.m_col_value];

	if(value == -1)
		m_ToolPalette.unset_style();
	else
		m_ToolPalette.set_style((ToolbarStyle)value);
}
