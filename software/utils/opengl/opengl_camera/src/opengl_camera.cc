#include <GL/glu.h>

#include <opengl/opengl_camera.h>

using namespace std;
using namespace KDL;
using namespace opengl;

/**
 * OpenGL_Camera
 * class constructor
 */
OpenGL_Camera::
OpenGL_Camera() : _eye_position( 2.0, 2.0, 2.0 ),
                  _target_position( 0.0, 0.0, 0.0 ),
                  _field_of_view( 45.0 ),
                  _mouse_press_pos( 0.0, 0.0 ),
                  _prev_eye_position( _eye_position ),
                  _prev_target_position( _target_position ),
                  _prev_eye_rotation(){

}

/**
 * ~OpenGL_Camera
 * class destructor
 */
OpenGL_Camera::
~OpenGL_Camera(){

}

/** 
 * OpenGL_Camera( const OpenGL_Camera& other )
 * copy constructor
 */
OpenGL_Camera::
OpenGL_Camera( const OpenGL_Camera& other ) : _eye_position( other._eye_position ),
                                              _target_position( other._target_position ),
                                              _field_of_view( other._field_of_view ),
                                              _mouse_press_pos( other._mouse_press_pos ),
                                              _prev_eye_position( other._prev_eye_position ),
                                              _prev_target_position( other._prev_target_position ),
                                              _prev_eye_rotation( other._prev_eye_rotation ){

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Camera&
OpenGL_Camera::
operator=( const OpenGL_Camera& other ){
  _eye_position = other._eye_position;
  _target_position = other._target_position;
  _field_of_view = other._field_of_view;
  _mouse_press_pos = other._mouse_press_pos;
  _prev_eye_position = other._prev_eye_position;
  _prev_eye_rotation = other._prev_eye_rotation;
  return (*this);
}

/**
 * apply_transform
 * sets the GL_PROJECTION and GL_MODELVIEW matrices
 */
void
OpenGL_Camera::
apply_transform( int width,
                  int height ){
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluPerspective( _field_of_view, ( double )( width ) / ( double )( height ), 0.01, 1000.0);
  glMatrixMode( GL_MODELVIEW );

  gluLookAt( _eye_position( 0 ), _eye_position( 1 ), _eye_position( 2 ),
              _target_position( 0 ), _target_position( 1 ), _target_position( 2 ),
              0.0, 0.0, 1.0 );
  return;
}

/**
 * mouse_move
 * callback when the mouse is in motion over the opengl display
 */
void
OpenGL_Camera::
mouse_move( Vector2 pos,
            mouse_button_t mouseButton,
            int width,
            int height ){
  if( mouseButton == OPENGL_MOUSE_BUTTON_LEFT ){
    Vector delta_mouse_pos( _mouse_press_pos(0) - pos(0), pos(1) - _mouse_press_pos(1), 0.0 );
    _eye_position = _prev_eye_position + _prev_eye_rotation.Inverse() * delta_mouse_pos * 0.01;
    _target_position = _prev_target_position + _prev_eye_rotation.Inverse() * delta_mouse_pos * 0.01; 
  } else if ( mouseButton == OPENGL_MOUSE_BUTTON_MIDDLE ){
    Vector delta_mouse_pos( _mouse_press_pos(0) - pos(0), pos(1) - _mouse_press_pos(1), 0.0 );
    if( delta_mouse_pos.Norm() > 0.01 ){
      Vector eye_position_to_target_position = _prev_eye_position - _target_position;
      eye_position_to_target_position = _prev_eye_rotation * eye_position_to_target_position;
      Vector axis = eye_position_to_target_position * delta_mouse_pos;
      axis.Normalize();
      axis = _prev_eye_rotation.Inverse() * axis;
      Rotation rotation = Rotation::Rot( axis,  0.005 * delta_mouse_pos.Norm() );
      eye_position_to_target_position = _prev_eye_position - _target_position;
      _eye_position = _target_position + ( rotation * eye_position_to_target_position );
    }
  } else if ( mouseButton == OPENGL_MOUSE_BUTTON_RIGHT ) {
    Vector zoom = _prev_eye_position - _target_position;
    _eye_position = _target_position + zoom * ( 1.0 - ( pos(1) - _mouse_press_pos( 1 ) ) / 1000.0 );
  }
  return;
} 

/**
 * mouse_press
 * callback when the mouse is pressed over the opengl display
 */
void
OpenGL_Camera::
mouse_press( Vector2 pos,
              mouse_button_t mouseButton,
              int width,
              int height ){
  _mouse_press_pos = pos;
  _prev_eye_position = _eye_position;
  _prev_target_position = _target_position;
  _prev_eye_rotation = eye_rotation();
  return;
} 

/**
 * mouse_release
 * callback when a mouse button is released over the opengl display
 */
void
OpenGL_Camera::
mouse_release( Vector2 pos,
                mouse_button_t mouseButton,
                int width,
                int height ){
  return;
}

/**
 * set_eye_position
 * sets the camera eye position (where you are looking from)
 */
void
OpenGL_Camera::
set_eye_position( Vector eyePosition ){
  _eye_position = eyePosition;
  return;
}

/**
 * set_target_position
 * sets the camera target position (what you are looking at)
 */
void
OpenGL_Camera::
set_target_position( Vector targetPosition ){
  _target_position = targetPosition;
  return;
}

/**
 * set_field_of_view
 * sets the camera field of view
 */
void
OpenGL_Camera::
set_field_of_view( double fieldOfView ){
  _field_of_view = fieldOfView;
  return;
}

/**
 * eye_position
 * returns the position of the camera eye
 */
Vector
OpenGL_Camera::
eye_position( void )const{
  return _eye_position;
} 

/**
 * target_position
 * returns the position of the camera target
 */
Vector
OpenGL_Camera::
target_position( void )const{
  return _target_position;
}

/**
 * eye_rot
 * returns the rotation of the eye
 */
Rotation
OpenGL_Camera::
eye_rotation( void )const{
  Vector zaxis = _eye_position - _target_position;
  zaxis.Normalize();
  Vector xaxis = Vector( 0.0, 0.0, 1.0 ) * zaxis;
  xaxis.Normalize();
  Vector yaxis = zaxis * xaxis;
  yaxis.Normalize();
  return Rotation( xaxis(0), xaxis(1), xaxis(2), yaxis(0), yaxis(1), yaxis(2), zaxis(0), zaxis(1), zaxis(2) );
}

/**
 * field_of_view
 * returns the camera field of view
 */
double
OpenGL_Camera::
field_of_view( void )const{
  return _field_of_view;
}

/**
 * operator<<
 * ostream operator
 */
namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Camera& other ){
    return out;
  }
}
