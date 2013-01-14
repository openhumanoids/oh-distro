#ifndef OPENGL_OPENGL_OBJECT_H
#define OPENGL_OPENGL_OBJECT_H

#include <iostream>
#include <string>
#include <vector>
#include <kdl/tree.hpp>
#include <Eigen/Dense>

namespace opengl {
  class OpenGL_Object {
  public:
    OpenGL_Object( std::string id = "N/A" );
    ~OpenGL_Object();
    OpenGL_Object( const OpenGL_Object& other );
    OpenGL_Object& operator=( const OpenGL_Object& other );

    void apply_transform( void );
    virtual void draw( void );

    void set_id( std::string id );
    void set_visible( bool visible );
    void set_color( Eigen::Vector3f color );
    void set_transparency( double transparency );
    void set_transform( KDL::Frame transform );

    std::string id( void )const;
    bool visible( void )const;
    Eigen::Vector3f color( void )const;
    double transparency( void )const;
    KDL::Frame transform( void )const;

  protected:
    std::string _id;
    bool _visible;
    Eigen::Vector3f _color;
    double _transparency;
    KDL::Frame _transform;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object& other );
}

#endif /* OPENGL_OPENGL_OBJECT_H */
