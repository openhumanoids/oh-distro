#ifndef OPENGL_OPENGL_OBJECT_H
#define OPENGL_OPENGL_OBJECT_H

#include <iostream>
#include <string>
#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace opengl {
  class OpenGL_Object {
  public:
    OpenGL_Object(const std::string &id = "N/A",
		   bool isHighlighted = false, 
		   Eigen::Vector3f highlightColor = Eigen::Vector3f(0,0,0));
    virtual ~OpenGL_Object();
    OpenGL_Object( const OpenGL_Object& other );
    OpenGL_Object& operator=( const OpenGL_Object& other );

    void apply_transform( void );
    virtual void draw( void );
    virtual void draw( Eigen::Vector3f color );

    virtual void set_id( std::string id );
    virtual void set_visible( bool visible );
    virtual void set_color( Eigen::Vector3f color );
    virtual void setHighlighted(bool); 
    virtual void set_transparency( double transparency );
    virtual void set_transform( KDL::Frame transform );

    std::string id( void )const;
    bool visible( void )const;
    Eigen::Vector3f color( void )const;
    double transparency( void )const;
    KDL::Frame transform( void )const;

  protected:
    std::string _id;
    bool _visible;
    Eigen::Vector3f _color, _highlightColor;    
    double _transparency;
    KDL::Frame _transform;    
    bool _isHighlighted;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object& other );
}

#endif /* OPENGL_OPENGL_OBJECT_H */
