#ifndef AUTHORING_QT4_WIDGET_AUTHORING_H
#define AUTHORING_QT4_WIDGET_AUTHORING_H

#include <iostream>
#include <QtGui/QWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QCheckBox>

#include <qt4/qt4_widget_opengl.h>

#include <urdf/model.h>
#include <affordance/AffordanceState.h>

#include <state/state_gfe.h>

#include <authoring/constraint.h>
#include <authoring/constraint_sequence.h>
#include <authoring/qt4_widget_constraint_editor.h>
#include <authoring/qt4_widget_opengl_authoring.h>

namespace authoring {
  class Qt4_Widget_Authoring : public QWidget {
    Q_OBJECT
  public:
    Qt4_Widget_Authoring( const std::string& xmlString, unsigned int numConstraints = 128, QWidget * parent = 0 );
    ~Qt4_Widget_Authoring();
    Qt4_Widget_Authoring( const Qt4_Widget_Authoring& other );
    Qt4_Widget_Authoring& operator=( const Qt4_Widget_Authoring& other );
    inline Qt4_Widget_OpenGL_Authoring* qt4_widget_opengl_authoring( void ){ return _widget_opengl_authoring; };

  signals:
    void info_update( const QString& info );
    void affordance_collection_update( std::vector< affordance::AffordanceState >& affordanceCollection );
    void robot_plan_update( std::vector< state::State_GFE >& robotPlan );
    void state_gfe_update( state::State_GFE& stateGFE );
    void drc_action_sequence_t_publish( const drc::action_sequence_t& msg );

  public slots:
    void update_info( const QString& info );
    void update_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_robot_plan( std::vector< state::State_GFE >& robotPlan );
    void update_state_gfe( state::State_GFE& stateGFE );

  protected slots:
    void _push_button_grab_pressed( void );
    void _push_button_import_pressed( void );
    void _push_button_export_pressed( void );
    void _push_button_publish_pressed( void );
    void _slider_updated( int currentIndex );

  protected:
    Qt4_Widget_OpenGL_Authoring * _widget_opengl_authoring;
    QTextEdit * _text_edit_info;
    QPushButton * _push_button_grab;
    QPushButton * _push_button_import;
    QPushButton * _push_button_export;
    QPushButton * _push_button_publish;
    QTextEdit * _text_edit_affordance_collection;
    QSlider * _slider_plan_current_index;
    QCheckBox * _check_box_visible_current_index;
    QCheckBox * _check_box_visible_trajectory;
    QCheckBox * _check_box_visible_trajectory_wrist;
    QLabel * _slider_current_time;

    urdf::Model _robot_model;
    std::vector< affordance::AffordanceState > _affordance_collection;
    std::vector< affordance::AffordanceState > _affordance_collection_ghost;
    std::vector< state::State_GFE > _robot_plan;
    state::State_GFE _state_gfe_ghost;
    Constraint_Sequence _constraint_sequence;
    std::vector< Qt4_Widget_Constraint_Editor* > _constraint_editors;

  private:
    
  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Authoring& other );
}

#endif /* AUTHORING_QT4_WIDGET_AUTHORING_H */
