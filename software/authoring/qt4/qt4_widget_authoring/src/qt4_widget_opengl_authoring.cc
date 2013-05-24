#include "authoring/qt4_widget_opengl_authoring.h"

using namespace std;
using namespace qt4;
using namespace state;
using namespace opengl;
using namespace authoring;

Qt4_Widget_OpenGL_Authoring::
Qt4_Widget_OpenGL_Authoring( const string& xmlString, 
                              QWidget * parent ) : Qt4_Widget_OpenGL( parent ),
                                                  _opengl_object_affordance_collection(),
                                                  _opengl_object_affordance_collection_ghost(),
                                                  _opengl_object_robot_plan(),
                                                  _opengl_object_gfe( xmlString ),
                                                  _opengl_object_gfe_ghost( xmlString ),
                                                  _opengl_object_constraint_visualizer(),
                                                  _timer_update( new QTimer( this ) ) {
  setMinimumSize( 800, 400 );

  _opengl_object_affordance_collection.set_visible( false );
  _opengl_object_affordance_collection_ghost.set_visible( false );
  _opengl_object_affordance_collection_ghost.set_transparency( 0.1 );
  _opengl_object_robot_plan.set_visible( false );
  _opengl_object_robot_plan.set_transparency( 0.1 );
  _opengl_object_gfe.set_visible( false );
  _opengl_object_gfe_ghost.set_visible( false );
  _opengl_object_gfe_ghost.set_transparency( 0.1 );

  _opengl_object_constraint_visualizer.set_color( Eigen::Vector3f(1.0, 0.0, 0.0) );
  _opengl_object_constraint_visualizer.set_transparency( 0.5 );
  _opengl_object_constraint_visualizer.set_visible( false );

//_opengl_object_constraint_visualizer.set_visible( false ) 

  opengl_scene().add_object( _opengl_object_affordance_collection );
  opengl_scene().add_object( _opengl_object_affordance_collection_ghost );
  opengl_scene().add_object( _opengl_object_robot_plan );
  opengl_scene().add_object( _opengl_object_gfe );
  opengl_scene().add_object( _opengl_object_gfe_ghost );
  opengl_scene().add_object( _opengl_object_constraint_visualizer);
  _timer_update->start( 100 );

  connect( _timer_update, SIGNAL( timeout() ), this, SLOT( _timer_update_callback() ) );
}

Qt4_Widget_OpenGL_Authoring::
~Qt4_Widget_OpenGL_Authoring() {

}

Qt4_Widget_OpenGL_Authoring::
Qt4_Widget_OpenGL_Authoring( const Qt4_Widget_OpenGL_Authoring& other ) {

}

Qt4_Widget_OpenGL_Authoring&
Qt4_Widget_OpenGL_Authoring::
operator=( const Qt4_Widget_OpenGL_Authoring& other ) {

  return (*this);
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_affordance_collection( vector< affordance::AffordanceState >& affordanceCollection ){
  _opengl_object_affordance_collection.set_visible( true );
  _opengl_object_affordance_collection.set( affordanceCollection );
  return;
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_affordance_collection_ghost( vector< affordance::AffordanceState >& affordanceCollection ){
  _opengl_object_affordance_collection_ghost.set_visible( true );
  _opengl_object_affordance_collection_ghost.set( affordanceCollection );
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_robot_plan( vector< State_GFE >& robotPlan ){
  _opengl_object_robot_plan.set_visible( true );
  _opengl_object_robot_plan.set( robotPlan );
  update();
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_gfe( State_GFE& stateGFE ){
  _opengl_object_gfe.set_visible( true );
  _opengl_object_gfe.set( stateGFE );
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_gfe_ghost( State_GFE& stateGFE ){
  _opengl_object_gfe_ghost.set_visible( true );
  _opengl_object_gfe_ghost.set( stateGFE );
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_robot_plan_current_index( int currentIndex ){
  _opengl_object_robot_plan.set_current_index( currentIndex );
  update();
  return;
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_robot_plan_visible_current_index( int visibleCurrentIndex ){
  if( visibleCurrentIndex == Qt::Checked ){
    _opengl_object_robot_plan.set_visible_current_index( true );
  } else {
    _opengl_object_robot_plan.set_visible_current_index( false );
  }
  update();
  return;
} 

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_robot_plan_visible_trajectory( int visibleTrajectory ){
  if( visibleTrajectory == Qt::Checked ){
    _opengl_object_robot_plan.set_visible_trajectory( true );
  } else {
    _opengl_object_robot_plan.set_visible_trajectory( false );
  }
  update();
  return;
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_robot_plan_visible_trajectory_wrist( int visibleTrajectoryWrist ){
  if( visibleTrajectoryWrist == Qt::Checked ){
    _opengl_object_robot_plan.set_visible_trajectory_wrist( true );
  } else {
    _opengl_object_robot_plan.set_visible_trajectory_wrist( false );
  }
  update();
  return;
}

void 
Qt4_Widget_OpenGL_Authoring::
update_constraint_visualizer( Constraint* constraint) {
  // update the box from the constraint
  if ( constraint->type() == CONSTRAINT_TASK_SPACE_REGION_TYPE ) {
      Constraint_Task_Space_Region* _tsr_constraint = dynamic_cast <Constraint_Task_Space_Region*>(constraint);
      if ( _tsr_constraint->child() != NULL) {
        double h = _tsr_constraint->ranges()[0].second - _tsr_constraint->ranges()[0].first;
        double w = _tsr_constraint->ranges()[1].second - _tsr_constraint->ranges()[1].first;
        double l = _tsr_constraint->ranges()[2].second - _tsr_constraint->ranges()[2].first;

        _opengl_object_constraint_visualizer.set_visible( true );
        // Use the frame of the child link; also, add a small constant offset
        // makes it easier to visualize the TSRs when they have very few
        // constraints
        _opengl_object_constraint_visualizer.set(_tsr_constraint->child()->getOriginFrame(), 
                                                 Eigen::Vector3f(h + 0.01, w + 0.01, l + 0.01));
      }
    }
  update();
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
_timer_update_callback( void ){
  update();
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_OpenGL_Authoring& other ) {
    return out;
  }

}
