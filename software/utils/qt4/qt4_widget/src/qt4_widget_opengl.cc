#include <QtGui/QMouseEvent>
#include <kdl/tree.hpp>

#include "opengl/opengl_camera.h"
#include "qt4/qt4_widget_opengl.h"

using namespace std;
using namespace KDL;
using namespace opengl;
using namespace qt4;

/**
 * Qt4_Widget_OpenGL
 * class constructor
 */
Qt4_Widget_OpenGL::
Qt4_Widget_OpenGL( QWidget * parent ) : QGLWidget( parent ),
                                    _opengl_scene() {
  setMinimumSize( 800, 600 );
  setMouseTracking( true );
}

/**
 * ~Qt4_Widget_OpenGL 
 * class destructor
 */
Qt4_Widget_OpenGL::
~Qt4_Widget_OpenGL() {

}

/**
 * Qt4_Widget_OpenGL
 * copy constructor
 */
Qt4_Widget_OpenGL::
Qt4_Widget_OpenGL( const Qt4_Widget_OpenGL& other ) : QGLWidget(),
                                              _opengl_scene( other._opengl_scene ){

}

/**
 * operator=
 * assignment operator
 */
Qt4_Widget_OpenGL&
Qt4_Widget_OpenGL::
operator=( const Qt4_Widget_OpenGL& other ) {
  _opengl_scene = other._opengl_scene;
  return (*this);
}

/**
 * opengl_scene
 * returns a reference to the opengl_scene
 */
OpenGL_Scene&
Qt4_Widget_OpenGL::
opengl_scene( void ){
  return _opengl_scene;
}

/**
 * opengl_scene
 * returns a const reference to the opengl_scene
 */
const OpenGL_Scene&
Qt4_Widget_OpenGL::
opengl_scene( void )const{
  return _opengl_scene;
} 

/** 
 * initializeGL
 * sets up the OpenGL context
 */
void
Qt4_Widget_OpenGL::
initializeGL(){
  glClearColor( 0.0, 0.0, 0.0, 1.0 );
  glEnable( GL_DEPTH_TEST );
  glEnable( GL_MULTISAMPLE );
  glEnable( GL_COLOR_MATERIAL );
  glEnable( GL_NORMALIZE );
//  glEnable( GL_CULL_FACE );
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
Qt4_Widget_OpenGL::
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
Qt4_Widget_OpenGL::
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
Qt4_Widget_OpenGL::
mouseMoveEvent( QMouseEvent * event ){
  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT, width(), height() );
    break;
  case ( Qt::MidButton ):
    _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE, width(), height() );
    break;
  case ( Qt::RightButton ):
    _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT, width(), height() );
    break;
  default:
    break;
  }
  update();
  return;
}

/**
 * mousePressEvent
 * callback when a mouse button is pressed
 */
void
Qt4_Widget_OpenGL::
mousePressEvent( QMouseEvent * event ){
  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT, width(), height() );
    break;
  case ( Qt::MidButton ):
    _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE, width(), height() );
    break;
  case ( Qt::RightButton ):
    _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT, width(), height() );
    break;
  default:  
    break;
  }
  raycast( _opengl_scene.camera().eye_position(), _opengl_scene.camera().click_position( Vector2( event->x(), event->y() ), width(), height() ) );
  update();
  return;
} 

/**
 * mouseReleaseEvent
 * callback when a mouse button is released
 */
void
Qt4_Widget_OpenGL::
mouseReleaseEvent( QMouseEvent * event ){
  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT, width(), height() );
    break;
  case ( Qt::MidButton ):
    _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE, width(), height() );
    break;
  case ( Qt::RightButton ):
    _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT, width(), height() );
    break;
  default:
    break;
  }
  update();
  return;
}

/**
 * raycast
 * a function that is called on a click that has the eye and click (projected onto z=0) positions
 */
void
Qt4_Widget_OpenGL::
raycast( const Vector eyePosition,
          const Vector clickPosition ){
  cout << "calling raycast (eyePosition:" << eyePosition(0) << "," << eyePosition(1) << "," << eyePosition(2) << ",clickPosition:" << clickPosition(0) << "," << clickPosition(1) << "," << clickPosition(2) << ")" << endl;
  return;
}

/**
 * operator<<
 * ostream operator
 */
namespace qt4 {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_OpenGL& other ) {
    return out;
  }
}
