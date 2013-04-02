#ifndef OPENGL_OPENGL_OBJECT_SPHERE_H
#define OPENGL_OPENGL_OBJECT_SPHERE_H

#include <iostream>
#include <GL/glu.h>

#include <Eigen/Dense>

#include "opengl/opengl_object.h"

namespace opengl {
  class OpenGL_Object_Sphere: public OpenGL_Object {
  public:
    OpenGL_Object_Sphere( std::string id = "N/A", const KDL::Frame& transform = KDL::Frame::Identity(), const KDL::Frame& offset = KDL::Frame::Identity(), double radius = 0.1 );
    virtual ~OpenGL_Object_Sphere();
    OpenGL_Object_Sphere( const OpenGL_Object_Sphere& other );
    OpenGL_Object_Sphere& operator=( const OpenGL_Object_Sphere& other );

    void set( double radius );    
    void set( KDL::Frame transform, double radius );
    virtual void set_color( Eigen::Vector3f color );
    virtual void set_color( Eigen::Vector4f color );

    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );
    static void draw( double radius, Eigen::Vector3f color );

    double radius( void )const;

  protected:
    bool _generate_dl( void );

    double _radius;
    GLUquadric * _quadric;
    GLuint _dl;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Sphere& other );
}

#endif /* OPENGL_OPENGL_OBJECT_SPHERE_H */
