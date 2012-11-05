#ifndef GTKMM_AFFORDANCETOOLPALETTE_H
#define GTKMM_AFFORDANCETOOLPALETTE_H

#include <gtkmm.h>
//#include "canvas.h"

class AffordanceToolPalette : public Gtk::VBox
{
public:
  AffordanceToolPalette(Gtk::DrawingArea* canvas);
  virtual ~AffordanceToolPalette();

private:

  void load_stock_items();
  void load_toggle_items();
  void load_special_items();

  //Signal handlers:
  void on_combo_orientation_changed();
  void on_combo_style_changed();

  //Tree model columns:
  class ModelColumnsOrientation : public Gtk::TreeModel::ColumnRecord
  {
  public:

    ModelColumnsOrientation()
    { add(m_col_value); add(m_col_name); }

    Gtk::TreeModelColumn<Gtk::Orientation> m_col_value;
    Gtk::TreeModelColumn<Glib::ustring> m_col_name;
  };

  ModelColumnsOrientation m_ColumnsOrientation;

  //Tree model columns:
  class ModelColumnsStyle : public Gtk::TreeModel::ColumnRecord
  {
  public:

    ModelColumnsStyle()
    { add(m_col_value); add(m_col_name); }

    Gtk::TreeModelColumn<int> m_col_value; //We use int to also allow -1
    Gtk::TreeModelColumn<Glib::ustring> m_col_name;
  };

  ModelColumnsStyle m_ColumnsStyle;

  //Child widgets:
  Gtk::ComboBox m_ComboOrientation;
  Glib::RefPtr<Gtk::ListStore> m_refTreeModelOrientation;
  Gtk::ComboBox m_ComboStyle;
  Glib::RefPtr<Gtk::ListStore> m_refTreeModelStyle;
  Gtk::ToolPalette m_ToolPalette;
  Gtk::ScrolledWindow m_ScrolledWindowPalette;
  //Gtk::ScrolledWindow m_ScrolledWindowCanvas;
  Gtk::DrawingArea* m_Canvas;
};

#endif //GTKMM_AFFORDANCETOOLPALETTE_H
