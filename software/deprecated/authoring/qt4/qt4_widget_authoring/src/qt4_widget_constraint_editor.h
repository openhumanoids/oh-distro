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
#include <authoring/opengl_object_constraint_sequence.h>

namespace authoring {
  enum select_class_t {
    SELECT_NONE,
    SELECT_OPENGL,
    SELECT_WIDGET,
    SELECT_EDIT
  };
  
  class Qt4_Widget_Constraint_Editor : public QWidget {
    Q_OBJECT
  public:
    Qt4_Widget_Constraint_Editor( const Constraint_Task_Space_Region& constraint, urdf::Model& robotModel, std::vector< affordance::AffordanceState >& affordanceCollection, const std::string& urdf_xml_string = "N/A", unsigned int constraintIndex = 0, QWidget * parent = 0 );
    ~Qt4_Widget_Constraint_Editor();
    Qt4_Widget_Constraint_Editor( const Qt4_Widget_Constraint_Editor& other );
    Qt4_Widget_Constraint_Editor& operator=( const Qt4_Widget_Constraint_Editor& other );

  static QString description_from_constraint( const Constraint_Task_Space_Region& constraint );
    select_class_t select_class( void ) { return _select_class; }
    
  signals:
    void info_update( const QString& info );
    void constraint_update( const Constraint_Task_Space_Region& constraint );
    void constraint_update( const Constraint_Task_Space_Region& constraint, unsigned int constraintIndex );
    void highlight_link_by_name( const QString& linkName ); 
    void constraint_visible( bool visible, unsigned int constraintIndex );
    void constraint_highlight( const QString& id, highlight_class_t highlight_class, bool highlight );
    void child_highlight( const QString& id, const QString& child, bool highlight );
    void update_collision_info( void );
    void bind_axes_to_constraint( Constraint_Task_Space_Region * cnst, bool unbind_if_duplicate );
    void unbind_axes_from_constraint( Constraint_Task_Space_Region * cnst );
    void publish_constraints ( float ik_time_of_interest = -1.0 );

  public slots:
    void update_constraint( const Constraint_Task_Space_Region& constraint );
    void highlight_constraint( const QString& id, highlight_class_t highlight_class, bool highlight );
    void highlight_child( const QString& id, const QString& child, bool highlight );
    void select_constraint( const QString& id, select_class_t select_class);

  protected slots:
    void _double_spin_box_time_start_value_changed( double start );
    void _double_spin_box_time_end_value_changed( double end );
    void _line_edit_metadata_text_changed( const QString& text );
    void _check_box_active_changed( int state );
    void _check_box_visible_changed( int state );
    void _push_button_edit_3D_pressed( void );
    void _push_button_edit_pressed( void );
    void check_valid_times( void );

    void enterEvent( QEvent * event );
    void leaveEvent( QEvent * event );

  protected:
    Constraint_Task_Space_Region _constraint;;
    urdf::Model& _robot_model;
    std::vector< std::pair< std::string, std::string > > _robot_affordances;
    std::vector< affordance::AffordanceState >& _object_affordances;
    std::string _urdf_xml_string;
    unsigned int _constraint_index;
    QLabel * _label_id;
    select_class_t _select_class;
    QCheckBox * _check_box_active;
    QCheckBox * _check_box_visible;
    QPushButton * _push_button_edit_3D;
    QPushButton * _push_button_edit;
    QDoubleSpinBox * _double_spin_box_time_start;
    QDoubleSpinBox * _double_spin_box_time_end;
    QLineEdit * _line_edit_metadata;
    QWidget * _constraint_editor_popup;
    QWidget * _constraint_visualizer_popup;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Constraint_Editor& other );
}

#endif /* AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H */
