#ifndef OPENGL_OPENGL_OBJECT_SPHERE_H
#define OPENGL_OPENGL_OBJECT_SPHERE_H

#include <iostream>
#include <GL/glu.h>

#include <Eigen/Dense>

#include "opengl/opengl_object.h"

namespace opengl {
  class OpenGL_Object_Sphere: public OpenGL_Object {
  public:
    OpenGL_Object_Sphere();
    ~OpenGL_Object_Sphere();
    OpenGL_Object_Sphere( const OpenGL_Object_Sphere& other );
    OpenGL_Object_Sphere& operator=( const OpenGL_Object_Sphere& other );
    
    void set( KDL::Frame transform, double dimensions );

    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );

    double dimensions( void )const;

  protected:
    bool _generate_dl( void );

    double _dimensions;
    GLUquadric * _quadric;
    GLuint _dl;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Sphere& other );
}

#endif /* OPENGL_OPENGL_OBJECT_SPHERE_H */
