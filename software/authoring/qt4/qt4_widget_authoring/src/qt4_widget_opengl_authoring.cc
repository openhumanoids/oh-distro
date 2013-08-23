#include "authoring/qt4_widget_opengl_authoring.h"
#include <QtGui/QMouseEvent>

using namespace std;
using namespace qt4;
using namespace state;
using namespace opengl;
using namespace affordance;
using namespace collision;
using namespace authoring;
using namespace interface;
using namespace KDL;

Qt4_Widget_OpenGL_Authoring::
Qt4_Widget_OpenGL_Authoring( const string& xmlString, 
                              QWidget * parent ) : Qt4_Widget_OpenGL( parent ),
                                                  _opengl_object_affordance_collection(),
                                                  _opengl_object_affordance_collection_ghost(),
                                                  _opengl_object_constraint_sequence(),
                                                  _opengl_object_robot_plan(xmlString),
                                                  _opengl_object_gfe( xmlString ),
                                                  _opengl_object_gfe_ghost( xmlString ),
                                                  _opengl_object_xyz_axis(true, false),
                                                  _opengl_object_rpy_axis(false, true),
                                                  _collision_object_container_xyz_axis(true, false, "CA"),
                                                  _collision_object_container_rpy_axis(false, true, "CA"),
                                                  _bound_constraint_task_space_region( NULL ),
                                                  _timer_update( new QTimer( this ) ),
                                                  _bound_constraint_midpoint(),
                                                  _mouse_capture_last_point() {
  setMinimumSize( 800, 400 );

  _opengl_object_affordance_collection.set_visible( false );
  _opengl_object_affordance_collection_ghost.set_visible( false );
  _opengl_object_affordance_collection_ghost.set_transparency( 0.1 );
  _opengl_object_constraint_sequence.set_visible( false );
  _opengl_object_constraint_sequence.set_transparency( 0.2 );
  _opengl_object_robot_plan.set_visible( false );
  _opengl_object_robot_plan.set_transparency( 0.1 );
  _opengl_object_gfe.set_visible( false );
  _opengl_object_gfe_ghost.set_visible( false );
  _opengl_object_gfe_ghost.set_transparency( 0.1 );
  _opengl_object_xyz_axis.set_visible( false );
  _opengl_object_xyz_axis.set_transparency( 0.4 );
  _opengl_object_rpy_axis.set_visible( false );
  _opengl_object_rpy_axis.set_transparency( 0.4 );
  // collision detector invisible 
  _opengl_object_collision_detector.set_visible( false );
  //_opengl_object_collision_detector.set_visible( true );
  _mouse_captured = false;
  _adjust_axis = ADJUST_NONE;
  _adjust_dir = ADJUST_NONE;

  opengl_scene().add_object( _opengl_object_xyz_axis );
  opengl_scene().add_object( _opengl_object_rpy_axis );
  opengl_scene().add_object( _opengl_object_affordance_collection );
  opengl_scene().add_object( _opengl_object_robot_plan );
  opengl_scene().add_object( _opengl_object_gfe );
  opengl_scene().add_object( _opengl_object_constraint_sequence );
  opengl_scene().add_object( _opengl_object_affordance_collection_ghost );
  opengl_scene().add_object( _opengl_object_gfe_ghost );

  _timer_update->start( 100 );

  connect( _timer_update, SIGNAL( timeout() ), this, SLOT( _timer_update_callback() ) );
}

Qt4_Widget_OpenGL_Authoring::
~Qt4_Widget_OpenGL_Authoring() {

}

Qt4_Widget_OpenGL_Authoring::
Qt4_Widget_OpenGL_Authoring( const Qt4_Widget_OpenGL_Authoring& other ) :
                                                  _collision_object_container_xyz_axis(true, false, "CA"),
                                                  _collision_object_container_rpy_axis(false, true, "CA") {
  //_interface_handler_authoring = new Interface_Handler_Authoring(&_opengl_object_constraint_sequence); 
  //*_interface_handler_authoring = *other._interface_handler_authoring;
  _bound_constraint_task_space_region = other._bound_constraint_task_space_region;
  _mouse_captured = other._mouse_captured;
  _adjust_axis = other._adjust_axis;
  _adjust_dir = other._adjust_dir;
}

Qt4_Widget_OpenGL_Authoring&
Qt4_Widget_OpenGL_Authoring::
operator=( const Qt4_Widget_OpenGL_Authoring& other ) {
  _opengl_scene = other._opengl_scene;
  //*_interface_handler_authoring = *other._interface_handler_authoring;
  return (*this);
}

void 
Qt4_Widget_OpenGL_Authoring:: 
set_affordance_collection( const vector< AffordanceState >& affordanceCollection ){
  _affordance_collection = affordanceCollection;
  return;
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_affordance_collection( vector< affordance::AffordanceState >& affordanceCollection ){
  _opengl_object_affordance_collection.set_visible( true );
  set_affordance_collection( affordanceCollection );
  _opengl_object_affordance_collection.set( affordanceCollection );
  _opengl_object_constraint_sequence.set_affordance_collection( affordanceCollection );

  /* Reset axes */
  // set things invisible
  _opengl_object_xyz_axis.set_visible( false );
  _opengl_object_rpy_axis.set_visible( false );

  // clear out collision detector
  _collision_object_container_xyz_axis.remove_from_collision_detector(&_interface_handler->collision_detector());
  _collision_object_container_rpy_axis.remove_from_collision_detector(&_interface_handler->collision_detector());

  // no more bound constraint!
  _bound_constraint_task_space_region = NULL;

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
update_opengl_object_constraint_sequence( const Constraint_Sequence& constraintSequence ){
  _opengl_object_constraint_sequence.set_visible( true );
  _opengl_object_constraint_sequence.set( constraintSequence );

  return;
}

void
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_robot_plan( const vector< State_GFE >& robotPlan ){
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
update_opengl_object_gfe_selected_links( const vector<string>& linkNames ){
  vector<string> linkNamesShortened;
  for (int i=0; i<linkNames.size(); i++){
    // take everything up to the dash, not inclusive
    int dash_pos = linkNames[i].find("-");
    linkNamesShortened.push_back(linkNames[i].substr(0, dash_pos));
  }
  _opengl_object_gfe.set_selected_links( linkNamesShortened, Eigen::Vector3f( 1.0, 0.0, 0.0 ), Eigen::Vector3f( 1.0, 1.0, 1.0 ) );
  _opengl_object_robot_plan.set_selected_links( linkNamesShortened, Eigen::Vector3f( 1.0, 0.0, 0.0 ), Eigen::Vector3f( 1.0, 1.0, 1.0 ) );
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
update_opengl_object_robot_plan_visible_initial_state( int visibleInitialState ){
  if( visibleInitialState == Qt::Checked ){
    _opengl_object_gfe.set_visible( true );
  } else {
    _opengl_object_gfe.set_visible( false );
  }
  update();
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
highlight_constraint( const QString& id, highlight_class_t highlight_class,
                      bool highlight ){
  if( highlight ){
    _opengl_object_constraint_sequence.set_highlight( vector< string >( 1, id.toStdString() ), highlight_class );
  } else {
    _opengl_object_constraint_sequence.set_highlight( vector< string >(), highlight_class );
  }
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
highlight_constraints( const vector< string >& ids, highlight_class_t highlight_class ){
  _opengl_object_constraint_sequence.set_highlight( ids, highlight_class );
  return;
}

void
Qt4_Widget_OpenGL_Authoring::
highlight_child( const QString& id,
                  const QString& child,
                  bool highlight ){
  if( highlight ){
    _opengl_object_affordance_collection.set_highlight( vector< string >( 1, child.toStdString() ) );
  } else {
    _opengl_object_affordance_collection.set_highlight( vector< string >() );
  }
}

/***
 * set_collision_to_constraints
 * Updates collision object info held by the interface handler by cycling through
 *   all of our constraints and adding them to interface handler's coll detector.
 */
void
Qt4_Widget_OpenGL_Authoring::
set_collision_to_constraints( void ){
  _interface_handler->collision_detector().clear_collision_objects();
  _opengl_object_constraint_sequence.add_to_collision( _interface_handler->collision_detector() );
  return;
}

/***
 * set_collision_to_axes
 *  Updates collision objects to just the adjustment axes, if they exist.
 */
void
Qt4_Widget_OpenGL_Authoring::
set_collision_to_axes( void ){
  _interface_handler->collision_detector().clear_collision_objects();
  if (_bound_constraint_task_space_region){
    _collision_object_container_xyz_axis.add_to_collision_detector(&_interface_handler->collision_detector());
    _collision_object_container_rpy_axis.add_to_collision_detector(&_interface_handler->collision_detector());
  }
}

/**
 * bind_axes_to_constraint
 * Attaches the manipulation axes (xyz and rpy) to the given constraint region.
 * Adds axes to the collision detector (and removes everything else).
 * If passed the same axes as are already shown, removes them if corresponding
 * option is set (this is useful to interface manager, saves a signal...)
 */
 void
 Qt4_Widget_OpenGL_Authoring::
 bind_axes_to_constraint( Constraint_Task_Space_Region * constraint, bool unbind_if_duplicate ){
  // Find relevant constraint
  //Constraint_Task_Space_Region * cnst = _opengl_object_constraint_sequence.get_constraint_task_space_region( id.toStdString() );
  if (!constraint || (unbind_if_duplicate && constraint == _bound_constraint_task_space_region)){
     unbind_axes_from_constraint( _bound_constraint_task_space_region );
     update_opengl_object_gfe_selected_links( vector<string>() );

  } else {
    _bound_constraint_task_space_region = constraint;
    // select urdf part this constraints
    update_opengl_object_gfe_selected_links( _bound_constraint_task_space_region->parents() );
    // convert it to a frame (should this functionality be wrapped into the constraints themselves?)
    double xmin = constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].first ? constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second : 0.0;
    double xmax = constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].first ? constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second : 0.0;
    double ymin = constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].first ? constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second : 0.0;
    double ymax = constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].first ? constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second : 0.0;
    double zmin = constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].first ? constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second : 0.0;
    double zmax = constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].first ? constraint->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second : 0.0;
    // get child affordance to which this constraint is relative
    AffordanceState * child = NULL;
    for( vector< AffordanceState >::iterator it = _affordance_collection.begin(); it != _affordance_collection.end(); it++ ){
      if( it->getName() == constraint->child() ){
        child = &(*it);
      }
    }
    if (child){
      Frame affordance_frame = Frame( Rotation::RPY( child->getRPY().x(), child->getRPY().y(), child->getRPY().z() ),
                              Vector( child->getXYZ().x(), child->getXYZ().y(), child->getXYZ().z() ) );


      // frame is centered around the midpoint of the constraint bounding box
      _bound_constraint_midpoint = 
            affordance_frame * constraint->offset() * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), 
            Vector( 0.5 * ( xmax + xmin ), 0.5 * ( ymax + ymin ), 0.5 * ( zmax + zmin ) ) ); 
      _opengl_object_xyz_axis.set_transform( _bound_constraint_midpoint );
      _opengl_object_rpy_axis.set_transform( _bound_constraint_midpoint );
      // And scaling is average length of axes
      double scaling = 1.0 / 3.0 * ( (xmax-xmin) + (ymax-ymin) + (zmax-zmin) );
      _opengl_object_xyz_axis.set_scale( scaling );
      _opengl_object_rpy_axis.set_scale( scaling );
      // Good to be shown
      _opengl_object_xyz_axis.set_visible( true );
      _opengl_object_rpy_axis.set_visible( true );

      // And now handle collision detector side
      _interface_handler->collision_detector().clear_collision_objects();
      _collision_object_container_xyz_axis.set_transform( _bound_constraint_midpoint, scaling );
      _collision_object_container_rpy_axis.set_transform( _bound_constraint_midpoint, scaling );
      _collision_object_container_xyz_axis.add_to_collision_detector(&_interface_handler->collision_detector());
      _collision_object_container_rpy_axis.add_to_collision_detector(&_interface_handler->collision_detector());
   
      // highlight it in viewer and list
      highlight_constraint(QString::fromStdString(constraint->id()), HIGHLIGHT_PURPLE, true);
      emit select_constraint(QString::fromStdString(constraint->id()), SELECT_EDIT);
    }
  }
}
/**
 * unbind_axes_from_constraint
 * Removes manipulation axes from given constraint if it is the one bound.
 */
 void
 Qt4_Widget_OpenGL_Authoring::
 unbind_axes_from_constraint( Constraint_Task_Space_Region * constraint ){
  if (constraint && (constraint == _bound_constraint_task_space_region)){
    /* Remove */
    update_opengl_object_gfe_selected_links( vector<string>() );
    // set things invisible
    _opengl_object_xyz_axis.set_visible( false );
    _opengl_object_rpy_axis.set_visible( false );

    // clear out collision detector
    _collision_object_container_xyz_axis.remove_from_collision_detector(&_interface_handler->collision_detector());
    _collision_object_container_rpy_axis.remove_from_collision_detector(&_interface_handler->collision_detector());

    // no more bound constraint!
    _bound_constraint_task_space_region = NULL;

    highlight_constraint(QString(), HIGHLIGHT_PURPLE, true);
    emit select_constraint(QString(), SELECT_EDIT);
  }
}

/**
 * mouseIdleEvent
 * callback when there is no mouse movement
 */
void
Qt4_Widget_OpenGL_Authoring::
mouseIdleEvent( void ){
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );

  _interface_handler->set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler->set_modelview_matrix( modelview_matrix );
  _interface_handler->set_projection_matrix( projection_matrix );
  _interface_handler->set_viewport( viewport );

  _interface_handler->mouse_event_idle();
  
  /*
  if( _interface_handler->intersected_object() != NULL ){
    cout << "selected object: " << _interface_handler->intersected_object()->id() << endl;
  }
*/

  return;
}

/**
 * mouseMoveEvent 
 * callback when the mouse is moved
 */
void
Qt4_Widget_OpenGL_Authoring::
mouseMoveEvent( QMouseEvent * event ){
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );
  
  _interface_handler->set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler->set_modelview_matrix( modelview_matrix );
  _interface_handler->set_projection_matrix( projection_matrix );
  _interface_handler->set_viewport( viewport );

  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    if (!_mouse_captured)
      _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT );
    _interface_handler->mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_LEFT, event->x(), event->y() );
    break;
  case ( Qt::MidButton ):
    if (!_mouse_captured)
      _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE );
    _interface_handler->mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_MIDDLE, event->x(), event->y() );
    break;
  case ( Qt::RightButton ):
    if (!_mouse_captured)
      _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT );
    _interface_handler->mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_RIGHT, event->x(), event->y() );
    break;
  default:
    _interface_handler->mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_NONE, event->x(), event->y() );
    break;
  }
  // IF mouse is captured, we're doing a move!
  if ( _mouse_captured ){
    Vector new_pos;
    Vector axes(0.0, 0.0, 0.0);
    // Do adjustment based on whatever we're modifying at the moment
    int axis;
    double working;
    switch ( _adjust_axis ){
      case (ADJUST_NONE):
        break;
      case (ADJUST_TX):
      case (ADJUST_TY):
      case (ADJUST_TZ):
        axis = (int) _adjust_axis;
        axes = _collision_object_container_xyz_axis.get_axis_direction((axis_num_t)axis);
        new_pos = get_approx_closest_approach_on_line_from_mouse_coords(
            modelview_matrix, projection_matrix, viewport,
            event->x(), event->y(), _bound_constraint_midpoint.p, axes);
        switch( event->buttons() ){
          case ( Qt::LeftButton ):
            // Figure out how far we moved along the axis since last time, and add that change
            //  to the constraint cood in this axis
            working = dot(new_pos - _mouse_capture_last_point, _collision_object_container_xyz_axis.get_axis_direction((axis_num_t)axis));
            _bound_constraint_task_space_region->offset().p[axis] +=
              working;
            break;
          case ( Qt::MidButton ):
            break;
          case ( Qt::RightButton ):
            // Figure out how far we moved along the axis since last time, and add that change
            //  to the constraint cood in this axis, unless such a change would be illegal...
            working = dot(new_pos - _mouse_capture_last_point, _collision_object_container_xyz_axis.get_axis_direction((axis_num_t)axis));
            if (_adjust_dir == ADJUST_MINUS) {
              if (_bound_constraint_task_space_region->ranges()[2*axis].second + working <
                  _bound_constraint_task_space_region->ranges()[2*axis+1].second){
                _bound_constraint_task_space_region->ranges()[2*axis].second +=
                  working;
              }
            } else if (_adjust_dir == ADJUST_PLUS){
              if (_bound_constraint_task_space_region->ranges()[2*axis].second <
                  _bound_constraint_task_space_region->ranges()[2*axis+1].second + working ){
                _bound_constraint_task_space_region->ranges()[2*axis+1].second +=
                  working;
              }
            } else {
              printf("ADJUST_DIR IN IMPOSSIBLE STATE!\n");
              exit(1);
            }
            break;
          default:
            break;
        }
        emit update_constraint(*_bound_constraint_task_space_region);
        bind_axes_to_constraint( _bound_constraint_task_space_region );
        _mouse_capture_last_point = new_pos;
        break;
      case (ADJUST_RR):
      case (ADJUST_RP):
      case (ADJUST_RY):
        // rotate
        axis = (int) _adjust_axis - ADJUST_RR;
        axes = _collision_object_container_xyz_axis.get_axis_direction((axis_num_t)(_adjust_axis-(int)ADJUST_RR));
        new_pos = 
          get_intersection_on_plane_from_mouse_coords(
            modelview_matrix, projection_matrix, viewport,
            event->x(), event->y(), _bound_constraint_midpoint.p, axes);
        switch( event->buttons() ){
          case ( Qt::LeftButton ):
            // Figure out how far we moved around, and add in to current orient
            working = -dot(((new_pos - _mouse_capture_last_point)*(new_pos - _bound_constraint_midpoint.p)), axes);
            switch ( _adjust_axis ){
              case (ADJUST_RR):
                _bound_constraint_task_space_region->offset().M.DoRotX(working);
                break;
              case (ADJUST_RP):
                _bound_constraint_task_space_region->offset().M.DoRotY(working);
                break;
              case (ADJUST_RY):
                _bound_constraint_task_space_region->offset().M.DoRotZ(working);
                break;
            }
            break;
          case ( Qt::MidButton ):
            break;
          case ( Qt::RightButton ):
            break;
          default:
            break;
        }
        emit update_constraint(*_bound_constraint_task_space_region);
        bind_axes_to_constraint( _bound_constraint_task_space_region );
        _mouse_capture_last_point = new_pos;
        break;
      default:
        printf("Unknown adjust_axes state!");
        exit(1);
        break;
    }
  }
  update();
  return;
}

/**
 * mousePressEvent
 * callback when a mouse button is pressed
 */
void
Qt4_Widget_OpenGL_Authoring::
mousePressEvent( QMouseEvent * event ){
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );


  _interface_handler->set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler->set_modelview_matrix( modelview_matrix );
  _interface_handler->set_projection_matrix( projection_matrix );
  _interface_handler->set_viewport( viewport );

  bool selecting_axis = true;

  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    if (!_mouse_captured){
      set_collision_to_axes();
      _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT );
    }
    _interface_handler->mouse_event( MOUSE_EVENT_CLICK, MOUSE_BUTTON_LEFT, event->x(), event->y() );
    break;
  case ( Qt::MidButton ):
    if (!_mouse_captured){
      // if the mouse isn't already captured, use middle click to bring a constraint into focus
      set_collision_to_constraints();
      selecting_axis = false;
      _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE );
      // deselect; if this is valid we'll reselect something else before a draw
      highlight_constraint(QString(), HIGHLIGHT_BLUE, true);
      select_constraint(QString(), SELECT_OPENGL);
    }
    _interface_handler->mouse_event( MOUSE_EVENT_CLICK, MOUSE_BUTTON_MIDDLE, event->x(), event->y() );
    break;
  case ( Qt::RightButton ):
    if (!_mouse_captured){
      set_collision_to_axes();
      _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT );
    }
    _interface_handler->mouse_event( MOUSE_EVENT_CLICK, MOUSE_BUTTON_RIGHT, event->x(), event->y() );
    break;
  default:  
    break;
  }

  // If the interface handler tells us we selected something...
  string tmp, tmp2;
  if (_interface_handler->intersected_object()){
    // and it's an axis, go into manipulation mode
    if (selecting_axis && _interface_handler->intersected_object()->id().length() > 2){
      if ( _bound_constraint_task_space_region){
        tmp = _bound_constraint_task_space_region->id();
        tmp2;
        //tmp.erase(0, 1);
        //highlight_constraint(QString::fromStdString(tmp), true);

        tmp = _interface_handler->intersected_object()->id();
        tmp2 = tmp.substr(tmp.length()-2, 2);
        // Adjusting which axis?
        if (tmp2 == "tx")
          _adjust_axis = ADJUST_TX;
        else if (tmp2 == "ty")
          _adjust_axis = ADJUST_TY;
        else if (tmp2 == "tz")
          _adjust_axis = ADJUST_TZ;
        else if (tmp2 == "rr")
          _adjust_axis = ADJUST_RR;
        else if (tmp2 == "rp")
          _adjust_axis = ADJUST_RP;
        else if (tmp2 == "ry")
          _adjust_axis = ADJUST_RY;
        else {
          printf("MOUSE PRESS HANDLER: NO MATCH!\n");
          _adjust_axis = ADJUST_NONE;
          _mouse_captured = false;
        }
        // + or - side of the axis?
        tmp2 = tmp.substr(tmp.length()-3, 1);
        if (tmp2 == "+"){
          _adjust_dir = ADJUST_PLUS;
        } else if (tmp2 == "-"){
          _adjust_dir = ADJUST_MINUS;
        } else {
          printf("MOUSE PRESS HANDLER: NO MATCH ON DIR!\n");
          _adjust_axis = ADJUST_NONE;
          _adjust_dir = ADJUST_NONE;
          _mouse_captured = false;
        }
        if (_adjust_axis != ADJUST_NONE){
          _mouse_captured = true;
          _mouse_capture_pos = Vector2(event->x(), event->y());
          Vector axes(0.0, 0.0, 0.0);
          if ( _adjust_axis <= ADJUST_TZ){
            // Handle grabbing translation / scaling axes
            axes = _collision_object_container_xyz_axis.get_axis_direction((axis_num_t)_adjust_axis);
            _mouse_capture_last_point = 
              get_approx_closest_approach_on_line_from_mouse_coords(
                modelview_matrix, projection_matrix, viewport,
                event->x(), event->y(), _bound_constraint_midpoint.p, axes);
          } else {
            // Handle dragging rotation bar
            axes = _collision_object_container_xyz_axis.get_axis_direction((axis_num_t)(_adjust_axis-(int)ADJUST_RR));
            _mouse_capture_last_point = 
              get_intersection_on_plane_from_mouse_coords(
                modelview_matrix, projection_matrix, viewport,
                event->x(), event->y(), _bound_constraint_midpoint.p, axes);
          }
        }
      }
    } // otherwise, we're selecting a constraint. if we got one, highlight it.
    else {
      tmp = _interface_handler->intersected_object()->id();
      highlight_constraint(QString::fromStdString(tmp), HIGHLIGHT_BLUE, true);
      emit select_constraint(QString::fromStdString(tmp), SELECT_OPENGL);
    }
  }

/*
  if( _interface_handler->intersected_object() != NULL ){
    cout << "highlighting selected object: " << _interface_handler->intersected_object()->id() << endl;
    highlight_constraint(QString::fromStdString(_interface_handler->intersected_object()->id()), true);
  } else {
    cout << "highlighting selected object: none\n";
  }
*/

  update();
} 

/**
 * mouseReleaseEvent
 * callback when a mouse button is released
 */
void
Qt4_Widget_OpenGL_Authoring::
mouseReleaseEvent( QMouseEvent * event ){
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );


  _interface_handler->set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler->set_modelview_matrix( modelview_matrix );
  _interface_handler->set_projection_matrix( projection_matrix );
  _interface_handler->set_viewport( viewport );

  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    if (!_mouse_captured)
      _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT );
    _interface_handler->mouse_event( MOUSE_EVENT_RELEASE, MOUSE_BUTTON_LEFT, event->x(), event->y() );
    break;
  case ( Qt::MidButton ):
    if (!_mouse_captured)
      _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE );
    _interface_handler->mouse_event( MOUSE_EVENT_RELEASE, MOUSE_BUTTON_MIDDLE, event->x(), event->y() );
    break;
  case ( Qt::RightButton ):
    if (!_mouse_captured)
      _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT );
    _interface_handler->mouse_event( MOUSE_EVENT_RELEASE, MOUSE_BUTTON_RIGHT, event->x(), event->y() );
    break;
  default:
    break;
  }
  if (_mouse_captured){
    // submit plan to server
    emit publish_constraints( _bound_constraint_task_space_region->start() );
    // clear error highlighting
    highlight_constraints( vector<string>(), HIGHLIGHT_RED );
  }
  _mouse_captured = false;
  set_collision_to_axes();
  update();
  return;
}

Vector
Qt4_Widget_OpenGL_Authoring::
get_approx_closest_approach_on_line_from_mouse_coords( GLdouble * modelview_matrix, 
  GLdouble * projection_matrix, GLint * viewport, double x, double y,
  Vector line_origin, Vector line_dir){
  // Get mouse click point in 3space
  double mouse_x = 0.0;
  double mouse_y = 0.0;
  double mouse_z = 0.0;
  gluUnProject( x, 
                ( ( double )viewport[ 3 ] ) - y, 
                0.0,
                modelview_matrix,
                projection_matrix,
                viewport,
                &mouse_x,
                &mouse_y,
                &mouse_z );
  Vector mouse_position( mouse_x, mouse_y, mouse_z );
  Vector ray_vec = mouse_position - _opengl_scene.camera().eye_position();
  ray_vec.Normalize();

  // project out from eye through mouse pos the distance from
  // eye to the point desired
  double dist_to_point = (line_origin - 
    _opengl_scene.camera().eye_position()).Norm();
  Vector new_pos = _opengl_scene.camera().eye_position() + 
      dist_to_point * (ray_vec);
  // project that onto the line we care about
  Vector working = (((line_origin - new_pos)*line_dir)*line_dir);
  working.Normalize();
  return new_pos + dot(line_origin - new_pos, working)*working;
}

Vector
Qt4_Widget_OpenGL_Authoring::
get_intersection_on_plane_from_mouse_coords( GLdouble * modelview_matrix, 
  GLdouble * projection_matrix, GLint * viewport, double x, double y,
  Vector plane_origin, Vector plane_normal){
  // Get mouse click point in 3space
  double mouse_x = 0.0;
  double mouse_y = 0.0;
  double mouse_z = 0.0;
  gluUnProject( x, 
                ( ( double )viewport[ 3 ] ) - y, 
                0.0,
                modelview_matrix,
                projection_matrix,
                viewport,
                &mouse_x,
                &mouse_y,
                &mouse_z );
  Vector mouse_position( mouse_x, mouse_y, mouse_z );
  Vector ray_vec = mouse_position - _opengl_scene.camera().eye_position();
  ray_vec.Normalize();

  // Calculate dist from eye pos to plane through rayvec
  //  derivation: on the plane, 
  double dist_to_plane_from_eyepos = 0;
  if (dot(ray_vec, plane_normal) != 0)
    dist_to_plane_from_eyepos = 
      (dot( plane_origin, plane_normal) -
      dot(_opengl_scene.camera().eye_position(), plane_normal)) /
      dot( ray_vec, plane_normal);

  // and generate the final point
  return _opengl_scene.camera().eye_position() + dist_to_plane_from_eyepos*ray_vec;
}

/* duplicates of functions from our base class, modified to use our new
   specialized interface handler */
//void
//Qt4_Widget_OpenGL_Authoring::
//add_collision_object( Collision_Object& collisionObject ){
//  _interface_handler->add_collision_object( collisionObject );
//  return;
//}
//Interface_Handler_Authoring&
//Qt4_Widget_OpenGL_Authoring::
//interface_handler( void ){
//  return *_interface_handler;
//}

/*
void 
Qt4_Widget_OpenGL_Authoring::
update_constraint_visualizer( Constraint* constraint) {
  // update the box from the constraint
  if ( constraint->type() == CONSTRAINT_TASK_SPACE_REGION_TYPE ) {
      Constraint_Task_Space_Region* _tsr_constraint = dynamic_cast <Constraint_Task_Space_Region*>(constraint);

      AffordanceState * child = NULL;
  
      if ( child != NULL) {
        double h = _tsr_constraint->ranges()[1].second - _tsr_constraint->ranges()[0].second;
        double w = _tsr_constraint->ranges()[3].second - _tsr_constraint->ranges()[2].second;
        double l = _tsr_constraint->ranges()[5].second - _tsr_constraint->ranges()[4].second;

        double x = (_tsr_constraint->ranges()[1].second + _tsr_constraint->ranges()[0].second) / 2.0;
        double y = (_tsr_constraint->ranges()[3].second + _tsr_constraint->ranges()[2].second) / 2.0;
        double z = (_tsr_constraint->ranges()[5].second + _tsr_constraint->ranges()[4].second) / 2.0;

        KDL::Frame tsr_frame = child->getOriginFrame() * KDL::Frame( KDL::Vector( x, y, z ) );

        _opengl_object_constraint_visualizer.set_visible( true );
        // Use the frame of the child link; also, add a small constant offset
        // makes it easier to visualize the TSRs when they have very few
        // constraints
        _opengl_object_constraint_visualizer.set(tsr_frame, // TODO: ASK MIKE HERE
                                                 Eigen::Vector3f(h + 0.01, w + 0.01, l + 0.01));
      }
    }
  update();
  return;
}
*/
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
