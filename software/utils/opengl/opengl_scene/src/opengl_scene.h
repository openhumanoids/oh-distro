#ifndef OPENGL_OPENGL_SCENE_H
#define OPENGL_OPENGL_SCENE_H

#include <iostream>
#include <vector>

#include <opengl/opengl_object.h>
#include <opengl/opengl_camera.h>

namespace opengl {
  class OpenGL_Scene {
  public:
    OpenGL_Scene(); 
    ~OpenGL_Scene();
    OpenGL_Scene( const OpenGL_Scene& other );
    OpenGL_Scene& operator=( const OpenGL_Scene& other );

    void draw( int width, int height );
    void add_object( OpenGL_Object& openglObject );
    void clear_objects( void );

    OpenGL_Camera& camera( void );
 
  protected:
    OpenGL_Camera _opengl_camera;
    std::vector< OpenGL_Object* > _opengl_objects;

  private:
    
  };
  std::ofstream& operator<<( std::ofstream& out, const OpenGL_Camera& other );
}

#endif /* OPENGL_OPENGL_SCENE_H */
