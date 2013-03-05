#include <GL/glu.h>
#include "interface/interface_handler.h"

using namespace std;
using namespace KDL;
using namespace collision;
using namespace interface;

/**
 * Interface_Handler
 * class constructor
 */
Interface_Handler::
Interface_Handler() : _modelview_matrix(),
                      _projection_matrix(),
                      _viewport(),
                      _collision_detector(),
                      _collision_objects(),
                      _intersected_object( NULL ){
  for( unsigned int i = 0; i < 4; i++ ){
    for( unsigned int j = 0; j < 4; j++ ){
      if( i == j ){
        _modelview_matrix[ 4 * i + j ] = 1.0;
        _projection_matrix[ 4 * i + j ] = 1.0;
      } else {
        _modelview_matrix[ 4 * i + j ] = 0.0;
        _projection_matrix[ 4 * i + j ] = 0.0;
      }
    }
  }
  _viewport[ 0 ] = 0;
  _viewport[ 1 ] = 0;
  _viewport[ 2 ] = 800;
  _viewport[ 3 ] = 600;
}

/**
 * ~Interface_Handler
 * class destructor
 */
Interface_Handler::
~Interface_Handler() {

}

/**
 * Interface_Handler
 * copy constructor
 */
Interface_Handler::
Interface_Handler( const Interface_Handler& other ) : _collision_detector(),
                                                      _collision_objects(),
                                                      _intersected_object( NULL ){
  for( unsigned int i = 0; i < 16; i++ ){
    _modelview_matrix[ i ] = other._modelview_matrix[ i ];
    _projection_matrix[ i ] = other._projection_matrix[ i ];
  }
  for( unsigned int i = 0; i < 4; i++ ){
    _viewport[ i ] = other._viewport[ i ];
  } 
}

/**
 * operator=
 * assignment operator
 */
Interface_Handler&
Interface_Handler::
operator=( const Interface_Handler& other ) {
  for( unsigned int i = 0; i < 16; i++ ){
    _modelview_matrix[ i ] = other._modelview_matrix[ i ];
    _projection_matrix[ i ] = other._projection_matrix[ i ];
  }
  for( unsigned int i = 0; i < 4; i++ ){
    _viewport[ i ] = other._viewport[ i ];
  }
  return (*this);
}

/**
 * set_eye_position
 * sets the eye position for ray intersection  
 */
void
Interface_Handler::
set_eye_position( Vector eyePosition ){
  _eye_position = eyePosition;
  return;
}

/**
 * set_modelview_matrix
 * sets the modelview matrix for ray intersection 
 */
void
Interface_Handler::
set_modelview_matrix( GLdouble modelviewMatrix[] ){
  for( unsigned int i = 0; i < 16; i++ ){
    _modelview_matrix[ i ] = modelviewMatrix[ i ];
  }
  return;
}

/**
 * set_projection_matrix
 * sets the projection matrix for ray intersection
 */
void
Interface_Handler::
set_projection_matrix( GLdouble projectionMatrix[] ){
  for( unsigned int i = 0; i < 16; i++ ){
    _projection_matrix[ i ] = projectionMatrix[ i ];
  }
  return;
}

/**
 * set_viewport
 * sets the viewport for ray intersection
 */
void
Interface_Handler::
set_viewport( GLint viewport[] ){
  for( unsigned int i = 0; i < 4; i++ ){
    _viewport[ i ] = viewport[ i ];
  }
  return;
}

/** 
 * add_collision_object
 * adds a collision object to the collision detector for ray intersection
 */
void
Interface_Handler::
add_collision_object( Collision_Object& collisionObject ){
  _collision_detector.add_collision_object( &collisionObject );
  return;
}

/**
 * mouse_event
 * handles mouse move, click, double click, and release events
 */
void
Interface_Handler::
mouse_event( mouse_event_t mouseEvent,
              mouse_button_t mouseButton,
              unsigned int mouseX,
              unsigned int mouseY ){
  double mouse_x = 0.0;
  double mouse_y = 0.0;
  double mouse_z = 0.0;
  gluUnProject( ( double )( mouseX ), 
                ( double )( _viewport[ 3 ] - mouseY ), 
                0.0,
                _modelview_matrix,
                _projection_matrix,
                _viewport,
                &mouse_x,
                &mouse_y,
                &mouse_z );
  _mouse_position = Vector( mouse_x, mouse_y, mouse_z );

  switch( mouseEvent ){
  case ( MOUSE_EVENT_MOVE ):
    _mouse_event_move( mouseButton );
    break;
  case ( MOUSE_EVENT_CLICK ):
    _mouse_event_click( mouseButton );
    break;
  case ( MOUSE_EVENT_DOUBLE_CLICK ):
    _mouse_event_double_click( mouseButton );
    break;
  case ( MOUSE_EVENT_RELEASE ):
    _mouse_event_release( mouseButton );
    break;
  default:
    break;
  } 
  return;
}

/**
 * mouse_event_idle
 * handles an idle mouse event
 */
void
Interface_Handler::
mouse_event_idle( void ){
  if( _intersected_object == NULL ){
  _collision_detector.ray_test( _eye_position, _eye_position + ( _mouse_position - _eye_position ) * 100000.0, _intersected_object );
  if( _intersected_object != NULL ){
    _num_event_idle_intersections++;
    if( _num_event_idle_intersections < 10 ){
      _intersected_object = NULL;
    }
  } else {
    _num_event_idle_intersections = 0;
  }
  }
  return;
}

/**
 * collision_detector
 * returns a reference to the collision detector
 */
Collision_Detector&
Interface_Handler::
collision_detector( void ){
  return _collision_detector;
}

/**
 * intersected_object
 * returns the pointer to the object that has been identified as intersected
 */
Collision_Object*
Interface_Handler::
intersected_object( void ){
  return _intersected_object;
}

/**
 * _mouse_event_move
 * handles a mouse move event
 */
void
Interface_Handler::
_mouse_event_move( mouse_button_t mouseButton ){
  if( _intersected_object != NULL ){
    _collision_detector.ray_test( _eye_position, _eye_position + ( _mouse_position - _eye_position ) * 100000.0, _intersected_object );
  }
  return;
}

/**
 * _mouse_event_click
 * handles a mouse click event
 */
void
Interface_Handler::
_mouse_event_click( mouse_button_t mouseButton ){
  switch( mouseButton ){
  case ( MOUSE_BUTTON_LEFT ):
    break;
  case ( MOUSE_BUTTON_MIDDLE ):
    break;
  case ( MOUSE_BUTTON_RIGHT ):
    _collision_detector.ray_test( _eye_position, _eye_position + ( _mouse_position - _eye_position ) * 100000.0, _intersected_object );
    break;
  default:
    break;
  }  
  return;
}

/**
 * _mouse_event_double_click
 * handles a mouse double click event
 */
void
Interface_Handler::
_mouse_event_double_click( mouse_button_t mouseButton ){

  return;
}

/**
 * _mouse_event_release
 * handles a mouse release event
 */
void
Interface_Handler::
_mouse_event_release( mouse_button_t mouseButton ){

  return;
}

/**
 * operator<<
 * ostream operator
 */
namespace interface {
  ostream&
  operator<<( ostream& out,
              const Interface_Handler& other ) {
    return out;
  }
}
