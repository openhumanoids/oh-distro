#include <GL/gl.h>

#include <opengl_scene/opengl_scene.h>

using namespace std;
using namespace opengl;

/**
 * OpenGL_Scene
 * class constructor
 */
OpenGL_Scene::
OpenGL_Scene() : _opengl_camera(),
                  _opengl_objects(){

}

/**
 * ~OpenGL_Scene
 * class destructor
 */
OpenGL_Scene::
~OpenGL_Scene(){

}

/**
 * OpenGL_Scene
 * copy constructor
 */
OpenGL_Scene::
OpenGL_Scene( const OpenGL_Scene& other ){

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Scene&
OpenGL_Scene::
operator=( const OpenGL_Scene& other ){
  return (*this);
} 

/**
 * draw
 * applies the camera transform and draws all of the OpenGL_Objects
 */
void
OpenGL_Scene::
draw( int width,
      int height ){
  _opengl_camera.apply_transform( width, height );
  for( unsigned int i = 0; i < _opengl_objects.size(); i++ ){
    if( _opengl_objects[ i ] != NULL ){
      _opengl_objects[ i ]->draw();
    }
  }
  return;
}

/**
 * add_object
 * adds an OpenGL_Object to the list of objects drawn by the draw function
 */
void
OpenGL_Scene::
add_object( OpenGL_Object& openglObject ){
  _opengl_objects.push_back( &openglObject );
  return;
}

/**
 * clear_objects
 * clears the list of objects drawn by the draw function 
 */
void
OpenGL_Scene::
clear_objects( void ){
  _opengl_objects.clear();
}

/**
 * camera
 * returns a reference to the OpenGL_Camera
 */
OpenGL_Camera& 
OpenGL_Scene::
camera( void ){
  return _opengl_camera;
}

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Scene& other ){
    return out;
  }
}
