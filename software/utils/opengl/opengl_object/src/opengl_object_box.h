#ifndef OPENGL_OPENGL_OBJECT_BOX_H
#define OPENGL_OPENGL_OBJECT_BOX_H

#include <iostream>
#include <GL/gl.h>

#include <Eigen/Dense>

#include "opengl/opengl_object.h"

namespace opengl {
  class OpenGL_Object_Box: public OpenGL_Object {
  public:
    OpenGL_Object_Box( std::string id = "N/A", const KDL::Frame& transform = KDL::Frame::Identity(), const KDL::Frame& offset = KDL::Frame::Identity(), Eigen::Vector3f dimensions = Eigen::Vector3f( 1.0, 1.0, 1.0 ) );
    virtual ~OpenGL_Object_Box();
    OpenGL_Object_Box( const OpenGL_Object_Box& other );
    OpenGL_Object_Box& operator=( const OpenGL_Object_Box& other );
  
    void set( Eigen::Vector3f dimensions );
    void set( KDL::Frame transform, Eigen::Vector3f dimensions );
    virtual void set_color( Eigen::Vector3f color );
    virtual void set_color( Eigen::Vector4f color );
    virtual void set_transparency( double transparency );

    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );

    Eigen::Vector3f dimensions( void )const;

  protected:
    bool _generate_dl( void );
    void _draw_box( Eigen::Vector3f color );

    Eigen::Vector3f _dimensions;
    GLuint _dl;
    
  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Box& other );
}

#endif /* OPENGL_OPENGL_OBJECT_BOX_H */
