#ifndef AUTHORING_QT4_WIDGET_AUTHORING_H
#define AUTHORING_QT4_WIDGET_AUTHORING_H

#include <iostream>
#include <QtGui/QWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QCheckBox>
#include <QtGui/QProgressBar>
#include <QtGui/QGridLayout>

#include <qt4/qt4_widget_opengl.h>

#include <urdf/model.h>
#include <affordance/AffordanceState.h>

#include <state/state_gfe.h>
#include <kinematics/kinematics_model_gfe.h>

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
    void drc_action_sequence_t_oneshot_publish( const drc::action_sequence_t& msg );
    void robot_plan_w_keyframes_t_publish( const drc::robot_plan_w_keyframes_t& msg );

  public slots:
    void update_info( const QString& info );
    void update_constraint( const Constraint_Task_Space_Region& constraint );
    void update_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_robot_plan( const std::vector< state::State_GFE >& robotPlan );
    void insert_robot_plan( const state::State_GFE& robotPlanSlice );
    void update_state_gfe( state::State_GFE& stateGFE );
    void aas_got_status_msg( long int plan_index, bool server_ready_status, 
        float last_time_solved, float total_time_to_solve,
        bool solving_highres, bool plan_is_good, bool plan_is_warn );
    void publish_constraints( float ik_time_of_interest = 0.0 );
    
  protected slots:
    void _push_button_grab_pressed( void );
    void _push_button_import_pressed( void );
    void _push_button_export_pressed( void );
    void _push_button_publish_pressed( void );
    void _push_button_publish_plan_pressed( void );
    void _push_button_publish_reverse_plan_pressed( void );
    void _push_button_insert_before_pressed( void );
    void _push_button_insert_after_pressed( void );
    void _push_button_remove_at_pressed( void );
    void _slider_updated( int currentIndex );

  protected:
    void _add_new_constraint( int constraint_num );
    void _refresh_constraint_editors( void );

    std::string _xmlString;
    Qt4_Widget_OpenGL_Authoring * _widget_opengl_authoring;
    QTextEdit * _text_edit_info;
    QPushButton * _push_button_grab;
    QPushButton * _push_button_import;
    QPushButton * _push_button_export;
    QPushButton * _push_button_publish;
    QTextEdit * _text_edit_affordance_collection;
    QSlider * _slider_plan_current_index;
    QLabel * _label_plan_step_count;
    QCheckBox * _check_box_visible_current_index;
    QCheckBox * _check_box_visible_trajectory;
    QCheckBox * _check_box_visible_trajectory_wrist;
    QCheckBox * _check_box_visible_initial_state;
    QPushButton * _push_button_publish_plan;
    QPushButton * _push_button_publish_reverse_plan;
    QDoubleSpinBox * _double_spin_box_publish_start;
    QLabel * _double_spin_box_publish_range_label;
    QDoubleSpinBox * _double_spin_box_publish_end;
    QLabel * _slider_current_time;
    QProgressBar * _progress_bar_planner;
    QLabel * _label_planner_feedback;
    QDoubleSpinBox * _double_spin_box_insert_remove;
    QPushButton * _push_button_insert_before;
    QPushButton * _push_button_insert_after;
    QPushButton * _push_button_remove_at;
    QGridLayout * _constraints_layout;
    QWidget * _constraints_widget;
    QWidget * _constraints_layout_labels;
    QScrollArea * _constraints_scroll_area;
    urdf::Model _robot_model;
    kinematics::Kinematics_Model_GFE _kinematics_model_gfe;
    std::vector< affordance::AffordanceState > _affordance_collection;
    std::vector< affordance::AffordanceState > _affordance_collection_ghost;
    std::vector< state::State_GFE > _robot_plan;
    state::State_GFE _state_gfe_ghost;
    Constraint_Sequence _constraint_sequence;
    std::vector< Qt4_Widget_Constraint_Editor* > _constraint_editors;

    int _constraint_ctr;
    long int _current_plan_index;
    
  private:
    
  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Authoring& other );
}

#endif /* AUTHORING_QT4_WIDGET_AUTHORING_H */
