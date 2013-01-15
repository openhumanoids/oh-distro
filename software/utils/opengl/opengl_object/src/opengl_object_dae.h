#ifndef OPENGL_OPENGL_OBJECT_DAE_H
#define OPENGL_OPENGL_OBJECT_DAE_H

#include <iostream>
#include <map>
#include <vector>
#include <GL/gl.h>
#include <Eigen/Dense>


#include "opengl/opengl_object.h"

namespace opengl {
  class OpenGL_Object_DAE: public OpenGL_Object {
  public:
    OpenGL_Object_DAE();
    OpenGL_Object_DAE( std::string id, std::string filename );
    ~OpenGL_Object_DAE();
    OpenGL_Object_DAE( const OpenGL_Object_DAE& other );
    OpenGL_Object_DAE& operator=( const OpenGL_Object_DAE& other );

    void set( KDL::Frame transform );

    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );

    std::map< std::string, std::map< std::string, std::string > > geometry_data( void )const;

  protected:
    bool _generate_dl( void );
    void _load_opengl_object( std::string filename );

    std::map< std::string, std::vector< Eigen::Vector2f > > _v2_data;
    std::map< std::string, std::vector< Eigen::Vector3f > > _v3_data;
    std::map< std::string, std::vector< Eigen::Vector4f > > _v4_data;
    std::map< std::string, std::vector< unsigned int > > _index_data;
    std::map< std::string, std::map< std::string, std::string > > _geometry_data;
    GLuint _dl;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_DAE& other );
}

#endif /* OPENGL_OPENGL_OBJECT_DAE_H */
