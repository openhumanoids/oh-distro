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
    Qt4_Widget_Constraint_Editor( const Constraint_Task_Space_Region& constraint, urdf::Model& robotModel, std::vector< affordance::AffordanceState >& affordanceCollection, const std::string& urdf_xml_string = "N/A", unsigned int constraintIndex = 0, QWidget * parent = 0 );
    ~Qt4_Widget_Constraint_Editor();
    Qt4_Widget_Constraint_Editor( const Qt4_Widget_Constraint_Editor& other );
    Qt4_Widget_Constraint_Editor& operator=( const Qt4_Widget_Constraint_Editor& other );

  static QString description_from_constraint( const Constraint_Task_Space_Region& constraint );

  signals:
    void info_update( const QString& info );
    void constraint_update( const Constraint_Task_Space_Region& constraint );
    void constraint_update( const Constraint_Task_Space_Region& constraint, unsigned int constraintIndex );
    void highlight_link_by_name( const QString& linkName ); 
    void constraint_visible( bool visible, unsigned int constraintIndex );
    void constraint_highlight( const QString& id, bool highlight );
    void child_highlight( const QString& id, const QString& child, bool highlight );

  public slots:
    void update_constraint( const Constraint_Task_Space_Region& constraint );
    void update_description( const QString& description );
    void highlight_constraint( const QString& id, bool highlight );
    void highlight_child( const QString& id, const QString& child, bool highlight );

  protected slots:
    void _double_spin_box_time_start_value_changed( double start );
    void _double_spin_box_time_end_value_changed( double end );
    void _line_edit_metadata_text_changed( const QString& text );
    void _check_box_active_changed( int state );
    void _check_box_visible_changed( int state );
    void _push_button_edit_pressed( void );
    void check_valid_times( void );

  protected:
    Constraint_Task_Space_Region _constraint;;
    urdf::Model& _robot_model;
    std::vector< std::pair< std::string, std::string > > _robot_affordances;
    std::vector< affordance::AffordanceState >& _object_affordances;
    std::string _urdf_xml_string;
    unsigned int _constraint_index;
    QLabel * _label_id;
    QCheckBox * _check_box_active;
    QCheckBox * _check_box_visible;
    QPushButton * _push_button_edit;
    QDoubleSpinBox * _double_spin_box_time_start;
    QDoubleSpinBox * _double_spin_box_time_end;
    QLineEdit * _line_edit_metadata;
    QLineEdit * _line_edit_description;
    QWidget * _constraint_editor_popup;
    QWidget * _constraint_visualizer_popup;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Constraint_Editor& other );
}

#endif /* AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H */
