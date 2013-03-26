#ifndef OPENGL_OPENGL_OBJECT_COORDINATE_AXIS_H
#define OPENGL_OPENGL_OBJECT_COORDINATE_AXIS_H

#include <iostream>
#include <GL/glu.h>

#include "opengl/opengl_object.h"
#include "opengl/opengl_object_torus.h"

namespace opengl {
  class OpenGL_Object_Coordinate_Axis: public OpenGL_Object {
  public:
    OpenGL_Object_Coordinate_Axis( bool drawTranslationAxes = true, bool drawRotationAxes = true );
    ~OpenGL_Object_Coordinate_Axis();
    OpenGL_Object_Coordinate_Axis( const OpenGL_Object_Coordinate_Axis& other );
    OpenGL_Object_Coordinate_Axis& operator=( const OpenGL_Object_Coordinate_Axis& other );

    virtual void set_scale( double scale );
    virtual void draw( void );

  protected:
    bool _generate_dl( void );

    double              _scale;
    OpenGL_Object_Torus _opengl_object_torus;
    GLUquadric *        _quadric;
    GLuint              _dl;
    bool                _draw_translation_axes;
    bool                _draw_rotation_axes;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Coordinate_Axis& other );
}

#endif /* OPENGL_OPENGL_OBJECT_COORDINATE_AXIS_H */
