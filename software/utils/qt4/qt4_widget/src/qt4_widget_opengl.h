#ifndef QT4_WIDGET_OPENGL_H
#define QT4_WIDGET_OPENGL_H

#include <iostream>

#include <QtOpenGL/QGLWidget>
#include <QtCore/QTimer>

#include <kdl/frames.hpp>

#include <collision/collision_object.h>

#include <interface/interface_handler.h>

#include <opengl/opengl_scene.h>
#include <opengl/opengl_object_collision_detector.h>

namespace qt4 {
  class Qt4_Widget_OpenGL: public QGLWidget {
    Q_OBJECT
  public:
    Qt4_Widget_OpenGL( QWidget * parent = 0 );
    ~Qt4_Widget_OpenGL();
    Qt4_Widget_OpenGL( const Qt4_Widget_OpenGL& other );
    Qt4_Widget_OpenGL& operator=( const Qt4_Widget_OpenGL& other );

    void add_collision_object( collision::Collision_Object& collisionObject );

    opengl::OpenGL_Scene& opengl_scene( void );
    const opengl::OpenGL_Scene& opengl_scene( void )const;
    interface::Interface_Handler& interface_handler( void );

  protected slots:
    void _handle_update_timer( void );

  protected:
    virtual void initializeGL();
    virtual void resizeGL( int width, int height );
    virtual void paintGL();

    virtual void mouseIdleEvent( void );
    virtual void mouseMoveEvent( QMouseEvent * event );
    virtual void mousePressEvent( QMouseEvent * event );
    virtual void mouseReleaseEvent( QMouseEvent * event );

    virtual void raycast( const KDL::Vector eyePosition, const KDL::Vector clickPosition );

    QTimer                                    _update_timer;
    interface::Interface_Handler              _interface_handler;
    opengl::OpenGL_Scene                      _opengl_scene;
    opengl::OpenGL_Object_Collision_Detector  _opengl_object_collision_detector;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_OpenGL& other );
}

#endif /* QT4_WIDGET_OPENGL_H */
