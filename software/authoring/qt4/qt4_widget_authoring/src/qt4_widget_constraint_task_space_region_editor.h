#ifndef AUTHORING_QT4_WIDGET_CONSTRAINT_TASK_SPACE_REGION_EDITOR_H
#define AUTHORING_QT4_WIDGET_CONSTRAINT_TASK_SPACE_REGION_EDITOR_H

#include <iostream>
#include <QtGui/QWidget>
#include <QtGui/QLabel>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QPushButton>

#include <urdf/model.h>
#include <affordance/AffordanceState.h>

#include <authoring/constraint_task_space_region.h>

namespace authoring {
  class Qt4_Widget_Constraint_Task_Space_Region_Editor : public QWidget {
    Q_OBJECT
  public:
    Qt4_Widget_Constraint_Task_Space_Region_Editor( Constraint_Task_Space_Region * constraint, urdf::Model& robotModel, std::vector< affordance::AffordanceState >& affordanceCollection, QWidget * parent = 0 );
    ~Qt4_Widget_Constraint_Task_Space_Region_Editor();
    Qt4_Widget_Constraint_Task_Space_Region_Editor( const Qt4_Widget_Constraint_Task_Space_Region_Editor& other );
    Qt4_Widget_Constraint_Task_Space_Region_Editor& operator=( const Qt4_Widget_Constraint_Task_Space_Region_Editor& other );

  signals:
    void description_update( const QString& description ); 
 
  public slots: 
    void _constraint_changed( void );
    void _constraint_changed( double value );
    void _constraint_changed( int index );

  protected slots:
    inline void _range_x_minimize( void ){ _double_spin_box_x_min->setValue( _double_spin_box_x_min->minimum() ); return; };
    inline void _range_x_maximize( void ){ _double_spin_box_x_max->setValue( _double_spin_box_x_max->maximum() ); return; };
    inline void _range_y_minimize( void ){ _double_spin_box_y_min->setValue( _double_spin_box_y_min->minimum() ); return; };
    inline void _range_y_maximize( void ){ _double_spin_box_y_max->setValue( _double_spin_box_y_max->maximum() ); return; };
    inline void _range_z_minimize( void ){ _double_spin_box_z_min->setValue( _double_spin_box_z_min->minimum() ); return; };
    inline void _range_z_maximize( void ){ _double_spin_box_z_max->setValue( _double_spin_box_z_max->maximum() ); return; };
    inline void _range_roll_minimize( void ){ _double_spin_box_roll_min->setValue( _double_spin_box_roll_min->minimum() ); return; };
    inline void _range_roll_maximize( void ){ _double_spin_box_roll_max->setValue( _double_spin_box_roll_max->maximum() ); return; };
    inline void _range_pitch_minimize( void ){ _double_spin_box_pitch_min->setValue( _double_spin_box_pitch_min->minimum() ); return; };
    inline void _range_pitch_maximize( void ){ _double_spin_box_pitch_max->setValue( _double_spin_box_pitch_max->maximum() ); return; };
    inline void _range_yaw_minimize( void ){ _double_spin_box_yaw_min->setValue( _double_spin_box_yaw_min->minimum() ); return; };
    inline void _range_yaw_maximize( void ){ _double_spin_box_yaw_max->setValue( _double_spin_box_yaw_max->maximum() ); return; };

  protected:
    Constraint_Task_Space_Region * _constraint;
    urdf::Model& _robot_model;
    std::vector< std::pair< std::string, std::string > > _robot_affordances;
    std::vector< affordance::AffordanceState >& _object_affordances;

    QLabel * _label_id;
    QComboBox * _combo_box_parent;
    QComboBox * _combo_box_child;
    QComboBox * _combo_box_type;
    QDoubleSpinBox * _double_spin_box_x_min;
    QDoubleSpinBox * _double_spin_box_x_max;
    QDoubleSpinBox * _double_spin_box_y_min;
    QDoubleSpinBox * _double_spin_box_y_max;
    QDoubleSpinBox * _double_spin_box_z_min;
    QDoubleSpinBox * _double_spin_box_z_max;
    QDoubleSpinBox * _double_spin_box_roll_min;
    QDoubleSpinBox * _double_spin_box_roll_max;
    QDoubleSpinBox * _double_spin_box_pitch_min;
    QDoubleSpinBox * _double_spin_box_pitch_max;
    QDoubleSpinBox * _double_spin_box_yaw_min;
    QDoubleSpinBox * _double_spin_box_yaw_max;
    QPushButton * _push_button_x_min;
    QPushButton * _push_button_x_max;
    QPushButton * _push_button_y_min;
    QPushButton * _push_button_y_max;
    QPushButton * _push_button_z_min;
    QPushButton * _push_button_z_max;
    QPushButton * _push_button_roll_min;
    QPushButton * _push_button_roll_max;
    QPushButton * _push_button_pitch_min;
    QPushButton * _push_button_pitch_max;
    QPushButton * _push_button_yaw_min;
    QPushButton * _push_button_yaw_max;
    QDoubleSpinBox * _double_spin_box_parent_to_constraint_x;
    QDoubleSpinBox * _double_spin_box_parent_to_constraint_y;
    QDoubleSpinBox * _double_spin_box_parent_to_constraint_z;
    QDoubleSpinBox * _double_spin_box_child_to_constraint_x;
    QDoubleSpinBox * _double_spin_box_child_to_constraint_y;
    QDoubleSpinBox * _double_spin_box_child_to_constraint_z;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Constraint_Task_Space_Region_Editor& other );
}

#endif /* AUTHORING_QT4_WIDGET_CONSTRAINT_TASK_SPACE_REGION_EDITOR_H */
