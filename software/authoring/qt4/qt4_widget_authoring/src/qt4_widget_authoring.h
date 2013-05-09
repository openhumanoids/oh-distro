#ifndef AUTHORING_QT4_WIDGET_AUTHORING_H
#define AUTHORING_QT4_WIDGET_AUTHORING_H

#include <iostream>
#include <QtGui/QWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QDoubleSpinBox>

#include <qt4/qt4_widget_opengl.h>

#include <urdf/model.h>
#include <affordance/AffordanceState.h>

#include <authoring/constraint.h>
#include <authoring/qt4_widget_constraint_editor.h>

namespace authoring {
  class Qt4_Widget_Authoring : public QWidget {
    Q_OBJECT
  public:
    Qt4_Widget_Authoring( const std::string& urdfFilename = "/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf", unsigned int numConstraints = 128, QWidget * parent = 0 );
    ~Qt4_Widget_Authoring();
    Qt4_Widget_Authoring( const Qt4_Widget_Authoring& other );
    Qt4_Widget_Authoring& operator=( const Qt4_Widget_Authoring& other );

  signals:
    void info_update( const QString& info );
    void time_min_update( double timeMin );
    void time_max_update( double timeMax );
    void affordance_collection_update( std::vector< affordance::AffordanceState >& affordanceCollection );

  public slots:
    void update_info( const QString& info );
    void update_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );

  protected slots:
    void _push_button_grab_pressed( void );
    void _push_button_import_pressed( void );
    void _push_button_export_pressed( void );
    void _push_button_publish_pressed( void );
    void _double_spin_box_end_time_changed( double endTime );

  protected:
    qt4::Qt4_Widget_OpenGL * _widget_opengl;
    QTextEdit * _text_edit_info;
    QPushButton * _push_button_grab;
    QPushButton * _push_button_import;
    QPushButton * _push_button_export;
    QPushButton * _push_button_publish;
    QDoubleSpinBox * _double_spin_box_end_time;
    QTextEdit * _text_edit_affordance_collection;

    urdf::Model _robot_model;
    std::vector< affordance::AffordanceState > _affordance_collection;
    std::vector< affordance::AffordanceState > _affordance_collection_ghost;
    std::vector< Constraint* > _constraints;
    std::vector< Qt4_Widget_Constraint_Editor* > _constraint_editors;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Authoring& other );
}

#endif /* AUTHORING_QT4_WIDGET_AUTHORING_H */
