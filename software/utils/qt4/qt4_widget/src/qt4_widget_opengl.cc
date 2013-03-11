#include <QtGui/QMouseEvent>
#include <kdl/tree.hpp>

#include "opengl/opengl_camera.h"
#include "qt4/qt4_widget_opengl.h"

using namespace std;
using namespace KDL;
using namespace collision;
using namespace opengl;
using namespace interface;
using namespace qt4;

/**
 * Qt4_Widget_OpenGL
 * class constructor
 */
Qt4_Widget_OpenGL::
Qt4_Widget_OpenGL( QWidget * parent ) : QGLWidget( parent ),
                                        _update_timer( this ),
                                        _interface_handler(),
                                        _opengl_scene(),
                                        _opengl_object_collision_detector() {
  _opengl_object_collision_detector.set( _interface_handler.collision_detector() );
  opengl_scene().add_object( _opengl_object_collision_detector );
  setMinimumSize( 800, 600 );
  setMouseTracking( true );
  
  connect( &_update_timer, SIGNAL( timeout() ), this, SLOT( _handle_update_timer() ) );

  _update_timer.start( 100 );
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
                                                      _update_timer(),
                                                      _opengl_scene( other._opengl_scene ),
                                                      _interface_handler( other._interface_handler ){
  _update_timer.start( 100 );
}

/**
 * operator=
 * assignment operator
 */
Qt4_Widget_OpenGL&
Qt4_Widget_OpenGL::
operator=( const Qt4_Widget_OpenGL& other ) {
  _opengl_scene = other._opengl_scene;
  _interface_handler = other._interface_handler;
  return (*this);
}

void
Qt4_Widget_OpenGL::
add_collision_object( Collision_Object& collisionObject ){
  interface_handler().add_collision_object( collisionObject );
  return;
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

Interface_Handler&
Qt4_Widget_OpenGL::
interface_handler( void ){
  return _interface_handler;
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
  _opengl_scene.draw();
  return;
}

/**
 * mouseIdleEvent
 * callback when there is no mouse movement
 */
void
Qt4_Widget_OpenGL::
mouseIdleEvent( void ){
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );

  _interface_handler.set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler.set_modelview_matrix( modelview_matrix );
  _interface_handler.set_projection_matrix( projection_matrix );
  _interface_handler.set_viewport( viewport );

  _interface_handler.mouse_event_idle();
  
  if( _interface_handler.intersected_object() != NULL ){
    cout << "selected object: " << _interface_handler.intersected_object()->id() << endl;
  }

  return;
}

/**
 * mouseMoveEvent 
 * callback when the mouse is moved
 */
void
Qt4_Widget_OpenGL::
mouseMoveEvent( QMouseEvent * event ){
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );
  
  _interface_handler.set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler.set_modelview_matrix( modelview_matrix );
  _interface_handler.set_projection_matrix( projection_matrix );
  _interface_handler.set_viewport( viewport );

  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT );
    _interface_handler.mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_LEFT, event->x(), event->y() );
    break;
  case ( Qt::MidButton ):
    _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE );
    _interface_handler.mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_MIDDLE, event->x(), event->y() );
    break;
  case ( Qt::RightButton ):
    _opengl_scene.camera().mouse_move( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT );
    _interface_handler.mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_RIGHT, event->x(), event->y() );
    break;
  default:
    _interface_handler.mouse_event( MOUSE_EVENT_MOVE, MOUSE_BUTTON_NONE, event->x(), event->y() );
    break;
  }
  if( _interface_handler.intersected_object() != NULL ){
    cout << "selected object: " << _interface_handler.intersected_object()->id() << endl;
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
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );


  _interface_handler.set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler.set_modelview_matrix( modelview_matrix );
  _interface_handler.set_projection_matrix( projection_matrix );
  _interface_handler.set_viewport( viewport );

  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT );
    _interface_handler.mouse_event( MOUSE_EVENT_CLICK, MOUSE_BUTTON_LEFT, event->x(), event->y() );
    raycast( _opengl_scene.camera().eye_position(), _opengl_scene.camera().click_position( Vector2( event->x(), event->y() )  ) ); 
    break;
  case ( Qt::MidButton ):
    _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE );
    _interface_handler.mouse_event( MOUSE_EVENT_CLICK, MOUSE_BUTTON_MIDDLE, event->x(), event->y() );
    break;
  case ( Qt::RightButton ):
    _opengl_scene.camera().mouse_press( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT );
    _interface_handler.mouse_event( MOUSE_EVENT_CLICK, MOUSE_BUTTON_RIGHT, event->x(), event->y() );
    break;
  default:  
    break;
  }

  if( _interface_handler.intersected_object() != NULL ){
    cout << "selected object: " << _interface_handler.intersected_object()->id() << endl;
  }

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
  GLdouble modelview_matrix[ 16 ];
  GLdouble projection_matrix[ 16 ];
  GLint viewport[ 4 ];

  _opengl_scene.camera().modelview_matrix( modelview_matrix );
  _opengl_scene.camera().projection_matrix( projection_matrix );
  _opengl_scene.camera().viewport( viewport );


  _interface_handler.set_eye_position( _opengl_scene.camera().eye_position() );
  _interface_handler.set_modelview_matrix( modelview_matrix );
  _interface_handler.set_projection_matrix( projection_matrix );
  _interface_handler.set_viewport( viewport );

  switch( event->buttons() ){
  case ( Qt::LeftButton ):
    _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_LEFT );
    _interface_handler.mouse_event( MOUSE_EVENT_RELEASE, MOUSE_BUTTON_LEFT, event->x(), event->y() );
    break;
  case ( Qt::MidButton ):
    _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_MIDDLE );
    _interface_handler.mouse_event( MOUSE_EVENT_RELEASE, MOUSE_BUTTON_MIDDLE, event->x(), event->y() );
    break;
  case ( Qt::RightButton ):
    _opengl_scene.camera().mouse_release( Vector2( event->x(), event->y() ), OPENGL_MOUSE_BUTTON_RIGHT );
    _interface_handler.mouse_event( MOUSE_EVENT_RELEASE, MOUSE_BUTTON_RIGHT, event->x(), event->y() );
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
  return;
}

void
Qt4_Widget_OpenGL::
_handle_update_timer( void ){
  update();
  mouseIdleEvent();
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
