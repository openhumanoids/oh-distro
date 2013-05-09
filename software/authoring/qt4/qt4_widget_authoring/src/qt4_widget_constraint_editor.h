#ifndef AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H
#define AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H

#include <iostream>
#include <QtGui/QLabel>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QPushButton>

#include <authoring/constraint.h>
#include <authoring/qt4_widget_constraint_task_space_region_editor.h>
#include <authoring/qt4_widget_double_slider.h>

namespace authoring {
  class Qt4_Widget_Constraint_Editor : public QWidget {
    Q_OBJECT
  public:
    Qt4_Widget_Constraint_Editor( Constraint *& constraint, urdf::Model& robotModel, std::vector< affordance::AffordanceState >& affordanceCollection, const std::string& id = "N/A", QWidget * parent = 0 );
    ~Qt4_Widget_Constraint_Editor();
    Qt4_Widget_Constraint_Editor( const Qt4_Widget_Constraint_Editor& other );
    Qt4_Widget_Constraint_Editor& operator=( const Qt4_Widget_Constraint_Editor& other );

    inline double& time_min( void ){ return _time_min; };
    inline const double& time_min( void )const{ return _time_min; };
    inline double& time_max( void ){ return _time_max; };
    inline const double& time_max( void )const{ return _time_max; };

  signals:
    void info_update( const QString& info );

  public slots:
//    void update_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_time_min( double timeMin );
    void update_time_max( double timeMax );

  protected slots:
    void _update_start( double index );
    void _update_end( double index );
    void _check_box_active_changed( int state );
    void _combo_box_type_changed( int index );
    void _push_button_edit_pressed( void );

  protected:
    Constraint *& _constraint;
    urdf::Model& _robot_model;
    std::vector< affordance::AffordanceState >& _affordance_collection;
    std::string _id;
    double _time_min;
    double _time_max;
    QLabel * _label_id;
    QCheckBox * _check_box_active;
    QComboBox * _combo_box_type;
    QPushButton * _push_button_edit;
    Qt4_Widget_Double_Slider* _widget_double_slider;
    QLabel * _label_time_start;
    QLabel * _label_time_end;
    QWidget * _constraint_editor_popup;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Constraint_Editor& other );
}

#endif /* AUTHORING_QT4_WIDGET_CONSTRAINT_EDITOR_H */
