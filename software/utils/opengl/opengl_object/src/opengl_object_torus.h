#ifndef OPENGL_OPENGL_OBJECT_TORUS_H
#define OPENGL_OPENGL_OBJECT_TORUS_H

#include <iostream>
#include <GL/glu.h>

#include "opengl/opengl_object.h"

namespace opengl {
  class OpenGL_Object_Torus: public OpenGL_Object {
  public:
    OpenGL_Object_Torus( std::string id = "N/A", const KDL::Frame& transform = KDL::Frame::Identity(), const KDL::Frame& offset = KDL::Frame::Identity(), double majorRadius = 0.1, double minorRadius = 0.01 );
    ~OpenGL_Object_Torus();
    OpenGL_Object_Torus( const OpenGL_Object_Torus& other );
    OpenGL_Object_Torus& operator=( const OpenGL_Object_Torus& other );

    virtual void set_dimensions( double majorRadius, double minorRadius );
    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );

  protected:
    bool _generate_dl( void );

    double _major_radius;
    double _minor_radius;
    GLUquadric *    _quadric;
    GLuint _dl;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Torus& other );
}

#endif /* OPENGL_OPENGL_OBJECT_TORUS_H */
