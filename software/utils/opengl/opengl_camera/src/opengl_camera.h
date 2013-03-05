#ifndef OPENGL_OPENGL_CAMERA_H
#define OPENGL_OPENGL_CAMERA_H

#include <iostream>
#include <vector>
#include <GL/gl.h>

#include <kdl/frames.hpp>

namespace opengl {
  typedef enum {
    OPENGL_MOUSE_BUTTON_LEFT,
    OPENGL_MOUSE_BUTTON_MIDDLE,
    OPENGL_MOUSE_BUTTON_RIGHT,
    NUM_OPENGL_MOUSE_BUTTONS
  } mouse_button_t;

  class OpenGL_Camera {
  public:
    OpenGL_Camera();
    ~OpenGL_Camera();
    OpenGL_Camera( const OpenGL_Camera& other );
    OpenGL_Camera& operator=( const OpenGL_Camera& other );

    void apply_transform( void );
    void mouse_move( KDL::Vector2 mousePos, mouse_button_t mouseButton );  
    void mouse_press( KDL::Vector2 mousePos, mouse_button_t mouseButton );
    void mouse_release( KDL::Vector2 mousePos, mouse_button_t mouseButton );
  
    void set_viewport( unsigned int width, unsigned int height );
    void set_eye_position( KDL::Vector eyePosition );
    void set_target_position( KDL::Vector targetPosition );
    void set_field_of_view( double fieldOfView );

    KDL::Vector eye_position( void )const;
    KDL::Vector target_position( void )const;
    KDL::Vector click_position( KDL::Vector2 mousePos );
    void modelview_matrix( GLdouble modelviewMatrix[] );
    void projection_matrix( GLdouble projectionMatrix[] );
    void viewport( GLint viewport[] );
    KDL::Rotation eye_rotation( void )const;
    double field_of_view( void )const;
  
  protected:

    KDL::Vector _eye_position;
    KDL::Vector _target_position;
    double _field_of_view;

    KDL::Vector2 _mouse_press_pos;
    KDL::Vector _prev_eye_position;
    KDL::Vector _prev_target_position;
    KDL::Rotation _prev_eye_rotation;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Camera& other );
}

#endif /* OPENGL_OPENGL_CAMERA_H */

