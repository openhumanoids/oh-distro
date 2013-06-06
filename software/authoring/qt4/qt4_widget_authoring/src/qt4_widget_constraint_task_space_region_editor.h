#ifndef AUTHORING_QT4_WIDGET_CONSTRAINT_TASK_SPACE_REGION_EDITOR_H
#define AUTHORING_QT4_WIDGET_CONSTRAINT_TASK_SPACE_REGION_EDITOR_H

#include <iostream>
#include <QtGui/QWidget>
#include <QtGui/QLabel>
#include <QtGui/QComboBox>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QPushButton>
#include <QtGui/QListWidget>
#include <QEvent>

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
    void widget_selected( void );
    void constraint_selected( const QString& constraintID );
    void highlight_parent_link_by_name( const QString& linkName );

  public slots: 
    void _constraint_changed( void );
    void _constraint_changed( double value );
    void _constraint_changed( int index );
    void _constraint_changed( QListWidgetItem * item );

  protected slots:
    void _mark_invalid_spin_boxes( void );
    bool eventFilter(QObject *object, QEvent *event);
    void _map_highlighted_parent( int );

  protected:
    Constraint_Task_Space_Region * _constraint;
    urdf::Model& _robot_model;
    std::vector< affordance::AffordanceState >& _object_affordances;

    QListWidget * _list_widget_parent;

    QComboBox * _combo_box_child;
    QComboBox * _combo_box_type;

    std::vector< QDoubleSpinBox * > _double_spin_box_ranges;
    std::vector< QCheckBox * > _check_box_ranges;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Constraint_Task_Space_Region_Editor& other );
}

#endif /* AUTHORING_QT4_WIDGET_CONSTRAINT_TASK_SPACE_REGION_EDITOR_H */
