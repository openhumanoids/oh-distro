#include "opengl_widget/opengl_widget.h"

using namespace std;
using namespace opengl;

/**
 * OpenGL_Widget
 * class constructor
 */
OpenGL_Widget::
OpenGL_Widget( QWidget * parent ) : QGLWidget( parent ),
                                    _opengl_scene() {
  setMinimumSize( 800, 600 );
  setMouseTracking( true );
}

/**
 * ~OpenGL_Widget 
 * class destructor
 */
OpenGL_Widget::
~OpenGL_Widget() {

}

/**
 * OpenGL_Widget
 * copy constructor
 */
OpenGL_Widget::
OpenGL_Widget( const OpenGL_Widget& other ) : QGLWidget(),
                                              _opengl_scene( other._opengl_scene ){

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Widget&
OpenGL_Widget::
operator=( const OpenGL_Widget& other ) {
  _opengl_scene = other._opengl_scene;
  return (*this);
}

/**
 * opengl_scene
 * returns a reference to the opengl_scene
 */
OpenGL_Scene&
OpenGL_Widget::
opengl_scene( void ){
  return _opengl_scene;
}

/**
 * opengl_scene
 * returns a const reference to the opengl_scene
 */
const OpenGL_Scene&
OpenGL_Widget::
opengl_scene( void )const{
  return _opengl_scene;
} 

/** 
 * initializeGL
 * sets up the OpenGL context
 */
void
OpenGL_Widget::
initializeGL(){
  glClearColor( 0.0, 0.0, 0.0, 1.0 );
  glEnable( GL_DEPTH_TEST );
  glEnable( GL_MULTISAMPLE );
  glEnable( GL_COLOR_MATERIAL );
  glEnable( GL_NORMALIZE );
  glEnable( GL_CULL_FACE );
  glEnable( GL_LINE_SMOOTH );
  glEnable( GL_BLEND );

  glShadeModel( GL_SMOOTH );
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable( GL_LIGHTING );
  glEnable( GL_LIGHT0 );
  GLfloat position[4] = { 0.0, 0.0, 10.0, 1.0 };
  glLightfv( GL_LIGHT0, GL_POSITION, position );
  return;
}

/**
 * resizeGL
 * adjusts the OpenGL context based on a resize of the widget
 */
void
OpenGL_Widget::
resizeGL( int width,
          int height ){
  int side = qMin( width, height );
  glViewport( 0, 0, width, height );
  return;
}

/**
 * paintGL
 * performs draw calls to OpenGL_Object classes in the OpenGL_Scene
 */
void
OpenGL_Widget::
paintGL(){
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  glLoadIdentity();
  _opengl_scene.draw( width(), height() );
  return;
}

/**
 * mouseMoveEvent 
 * callback when the mouse is moved
 */
void
OpenGL_Widget::
mouseMoveEvent( QMouseEvent * event ){
  update();
  return;
}

/**
 * mousePressEvent
 * callback when a mouse button is pressed
 */
void
OpenGL_Widget::
mousePressEvent( QMouseEvent * event ){
  update();
  return;
} 

/**
 * mouseReleaseEvent
 * callback when a mouse button is released
 */
void
OpenGL_Widget::
mouseReleaseEvent( QMouseEvent * event ){
  update();
  return;
}

/**
 * operator<<
 * ostream operator
 */
namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Widget& other ) {
    return out;
  }
}
