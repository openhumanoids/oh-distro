#ifndef AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H
#define AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H

#include <iostream>
#include <QtGui/QLabel>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QPushButton>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QLineEdit>

#include <authoring/constraint.h>
#include <authoring/qt4_widget_constraint_task_space_region_editor.h>
#include <authoring/qt4_widget_double_slider.h>

namespace authoring {
  class Qt4_Widget_Constraint_Editor : public QWidget {
    Q_OBJECT
  public:
    Qt4_Widget_Constraint_Editor( Constraint *& constraint, urdf::Model& robotModel, std::vector< affordance::AffordanceState >& affordanceCollection, const std::string& urdf_xml_string = "N/A", const std::string& id = "N/A", QWidget * parent = 0 );
    ~Qt4_Widget_Constraint_Editor();
    Qt4_Widget_Constraint_Editor( const Qt4_Widget_Constraint_Editor& other );
    Qt4_Widget_Constraint_Editor& operator=( const Qt4_Widget_Constraint_Editor& other );

  static QString description_from_constraint( const Constraint* constraint );

  signals:
    void info_update( const QString& info );
    void highlight_link_by_name( const QString& linkName ); 
    void constraint_selected( const QString& constraintID );

  public slots:
    void update_constraint( void );
    void update_description( const QString& description );

  protected slots:
    void _double_spin_box_time_start_value_changed( double start );
    void _double_spin_box_time_end_value_changed( double end );
    void _check_box_active_changed( int state );
    void _combo_box_type_changed( int index );
    void _push_button_edit_pressed( void );
    void check_valid_times( void );
    void show_visualizer( void );

  protected:
    Constraint *& _constraint;
    urdf::Model& _robot_model;
    std::vector< std::pair< std::string, std::string > > _robot_affordances;
    std::vector< affordance::AffordanceState >& _object_affordances;
    std::string _id;
    std::string _urdf_xml_string;
    QLabel * _label_id;
    QCheckBox * _check_box_active;
    QComboBox * _combo_box_type;
    QPushButton * _push_button_edit;
    QDoubleSpinBox * _double_spin_box_time_start;
    QDoubleSpinBox * _double_spin_box_time_end;
    QLineEdit * _line_edit_description;
    QWidget * _constraint_editor_popup;
    QWidget * _constraint_visualizer_popup;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Constraint_Editor& other );
}

#endif /* AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H */
