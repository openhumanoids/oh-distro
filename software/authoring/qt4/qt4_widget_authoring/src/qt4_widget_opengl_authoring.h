#ifndef AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H
#define AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H

#include <iostream>
#include <vector>

#include <qt4/qt4_widget_opengl.h>

#include <affordance/AffordanceState.h>
#include <opengl/opengl_object_gfe.h>
#include <opengl/opengl_object_trajectory_gfe.h>
#include <state/state_gfe.h>

#include <authoring/opengl_object_affordance_collection.h>

namespace authoring {
  class Qt4_Widget_OpenGL_Authoring : public qt4::Qt4_Widget_OpenGL {
    Q_OBJECT
  public:
    Qt4_Widget_OpenGL_Authoring( const std::string& xmlString, QWidget * parent = 0 );
    ~Qt4_Widget_OpenGL_Authoring();
    Qt4_Widget_OpenGL_Authoring( const Qt4_Widget_OpenGL_Authoring& other );
    Qt4_Widget_OpenGL_Authoring& operator=( const Qt4_Widget_OpenGL_Authoring& other );

  public slots:
    void update_opengl_object_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_affordance_collection_ghost( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_robot_plan( std::vector< state::State_GFE >& robotPlan );
    void update_opengl_object_gfe( state::State_GFE& stateGFE );
    void update_opengl_object_gfe_ghost( state::State_GFE& stateGFE );
    void update_opengl_object_robot_plan_current_index( int currentIndex );
    void update_opengl_object_robot_plan_visible_current_index( int visibleCurrentIndex );
    void update_opengl_object_robot_plan_visible_trajectory( int visibleTrajectory );
    void update_opengl_object_robot_plan_visible_trajectory_wrist( int visibleTrajectoryWrist );

  protected slots:
    void _timer_update_callback( void );

  protected:
    OpenGL_Object_Affordance_Collection _opengl_object_affordance_collection;
    OpenGL_Object_Affordance_Collection _opengl_object_affordance_collection_ghost;
    opengl::OpenGL_Object_Trajectory_GFE _opengl_object_robot_plan;
    opengl::OpenGL_Object_GFE _opengl_object_gfe;
    opengl::OpenGL_Object_GFE _opengl_object_gfe_ghost;

    QTimer* _timer_update;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_OpenGL_Authoring& other );
}

#endif /* AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H */
