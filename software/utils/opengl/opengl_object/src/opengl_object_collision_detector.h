#ifndef OPENGL_OPENGL_OBJECT_COLLISION_DETECTOR_H
#define OPENGL_OPENGL_OBJECT_COLLISION_DETECTOR_H

#include <iostream>

#include "collision/collision_detector.h"

#include "opengl/opengl_object.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"

namespace opengl {
  class OpenGL_Object_Collision_Detector: public OpenGL_Object {
  public:
    OpenGL_Object_Collision_Detector();
    ~OpenGL_Object_Collision_Detector();
    OpenGL_Object_Collision_Detector( const OpenGL_Object_Collision_Detector& other );
    OpenGL_Object_Collision_Detector& operator=( const OpenGL_Object_Collision_Detector& other );

    virtual void set( collision::Collision_Detector& collisionDetector );

    virtual void draw( void );

  protected:
    collision::Collision_Detector * _collision_detector;
    OpenGL_Object_Cylinder _opengl_object_cylinder;
    OpenGL_Object_Sphere _opengl_object_sphere;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Collision_Detector& other );
}

#endif /* OPENGL_OPENGL_OBJECT_COLLISION_DETECTOR_H */
