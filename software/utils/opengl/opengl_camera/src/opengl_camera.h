#ifndef OPENGL_OPENGL_CAMERA_H
#define OPENGL_OPENGL_CAMERA_H

#include <iostream>
#include <vector>

#include <Eigen/Dense>

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

    void apply_transform( int width, int height );
    void mouse_move( int x, int y, mouse_button_t mouseButton, int width, int height );  
    void mouse_press( int x, int y, mouse_button_t mouseButton, int width, int height );
    void mouse_release( int x, int y, mouse_button_t mouseButton, int width, int height );

    void set_eye_position( Eigen::Vector3f eyePosition );
    void set_target_position( Eigen::Vector3f targetPosition );

    Eigen::Vector3f eye_position( void )const;
    Eigen::Vector3f target_position( void )const;

  protected:
    Eigen::Vector3f _eye_position;
    Eigen::Vector3f _target_position;

    Eigen::Vector2f _mouse_press_pos;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Camera& other );
}

#endif /* OPENGL_OPENGL_CAMERA_H */

