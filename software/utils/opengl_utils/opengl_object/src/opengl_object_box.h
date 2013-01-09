#ifndef OPENGL_OPENGL_OBJECT_BOX_H
#define OPENGL_OPENGL_OBJECT_BOX_H

#include <iostream>

#include <Eigen/Dense>

#include "opengl_object/opengl_object.h"

namespace opengl {
  class OpenGL_Object_Box: public OpenGL_Object {
  public:
    OpenGL_Object_Box();
    ~OpenGL_Object_Box();
    OpenGL_Object_Box( const OpenGL_Object_Box& other );
    OpenGL_Object_Box& operator=( const OpenGL_Object_Box& other );

    void set( KDL::Frame transform, Eigen::Vector3f dimensions );

    virtual void draw( void );

    Eigen::Vector3f dimensions( void )const;

  protected:
    Eigen::Vector3f _dimensions;
    
  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Box& other );
}

#endif /* OPENGL_OPENGL_OBJECT_BOX_H */
