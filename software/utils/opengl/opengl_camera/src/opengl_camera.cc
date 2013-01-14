#include <GL/glu.h>

#include <opengl/opengl_camera.h>

using namespace std;
using namespace Eigen;
using namespace opengl;

/**
 * OpenGL_Camera
 * class constructor
 */
OpenGL_Camera::
OpenGL_Camera() : _eye_position( -2.0, -2.0, 2.0 ),
                  _target_position( 0.0, 0.0, 0.0 ),
                  _mouse_press_pos( 0.0, 0.0 ){

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
                                              _mouse_press_pos( other._mouse_press_pos ){

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
  _mouse_press_pos = other._mouse_press_pos;
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
  gluPerspective(45.0, ( double )( width ) / ( double )( height ), 0.01, 1000.0);
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
mouse_move( int x,
            int y,
            mouse_button_t mouseButton,
            int width,
            int height ){
  return;
} 

/**
 * mouse_press
 * callback when the mouse is pressed over the opengl display
 */
void
OpenGL_Camera::
mouse_press( int x,
              int y,
              mouse_button_t mouseButton,
              int width,
              int height ){
  _mouse_press_pos( 0 ) = x;
  _mouse_press_pos( 1 ) = y;
  return;
} 

/**
 * mouse_release
 * callback when a mouse button is released over the opengl display
 */
void
OpenGL_Camera::
mouse_release( int x,
                int y,
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
set_eye_position( Vector3f eyePosition ){
  _eye_position = eyePosition;
  return;
}

/**
 * set_target_position
 * sets the camera target position (what you are looking at)
 */
void
OpenGL_Camera::
set_target_position( Vector3f targetPosition ){
  _target_position = targetPosition;
  return;
}

/**
 * eye_position
 * returns the position of the camera eye
 */
Vector3f
OpenGL_Camera::
eye_position( void )const{
  return _eye_position;
} 

/**
 * target_position
 * returns the position of the camera target
 */
Vector3f
OpenGL_Camera::
target_position( void )const{
  return _target_position;
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
