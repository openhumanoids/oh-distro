#ifndef OPENGL_OPENGL_OBJECT_CYLINDER_H
#define OPENGL_OPENGL_OBJECT_CYLINDER_H

#include <iostream>
#include <GL/glu.h>

#include <Eigen/Dense>

#include "opengl/opengl_object.h"

namespace opengl {
  class OpenGL_Object_Cylinder: public OpenGL_Object {
  public:
    OpenGL_Object_Cylinder( std::string id = "N/A", Eigen::Vector2f dimensions = Eigen::Vector2f( 1.0, 1.0 ) );
    virtual ~OpenGL_Object_Cylinder();
    OpenGL_Object_Cylinder( const OpenGL_Object_Cylinder& other );
    OpenGL_Object_Cylinder& operator=( const OpenGL_Object_Cylinder& other );
    
    void set( KDL::Frame transform, Eigen::Vector2f dimensions );

    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );

    Eigen::Vector2f dimensions( void )const;

  protected:
    bool _generate_dl( void );

    Eigen::Vector2f _dimensions;
    GLUquadric * _quadric;
    GLuint _dl;
  
  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Cylinder& other );
}

#endif /* OPENGL_OPENGL_OBJECT_CYLINDER_H */
