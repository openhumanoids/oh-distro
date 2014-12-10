#ifndef AUTHORING_OPENGL_OBJECT_AFFORDANCE_H
#define AUTHORING_OPENGL_OBJECT_AFFORDANCE_H

#include <iostream>

#include <affordance/AffordanceState.h>

#include "opengl/opengl_object.h"
#include "opengl/opengl_object_box.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"

namespace authoring {
  class OpenGL_Object_Affordance: public opengl::OpenGL_Object {
  public:
    OpenGL_Object_Affordance();
    ~OpenGL_Object_Affordance();
    OpenGL_Object_Affordance( const OpenGL_Object_Affordance& other );
    OpenGL_Object_Affordance& operator=( const OpenGL_Object_Affordance& other );
 
    virtual void set( affordance::AffordanceState& affordanceState );
    virtual void set_transparency( double transparency );

    virtual void draw( void );
  
    virtual void set_color( Eigen::Vector3f color );

  protected:
    opengl::OpenGL_Object_Box _opengl_object_box;
    opengl::OpenGL_Object_Cylinder _opengl_object_cylinder;
    opengl::OpenGL_Object_Sphere _opengl_object_sphere;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Affordance& other );
}

#endif /* AUTHORING_OPENGL_OBJECT_AFFORDANCE_H */
