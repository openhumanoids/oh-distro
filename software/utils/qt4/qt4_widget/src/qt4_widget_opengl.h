#ifndef QT4_WIDGET_OPENGL_H
#define QT4_WIDGET_OPENGL_H

#include <iostream>

#include <QtOpenGL/QGLWidget>
#include <kdl/frames.hpp>

#include <opengl/opengl_scene.h>

namespace qt4 {
  class Qt4_Widget_OpenGL: public QGLWidget {
    Q_OBJECT
  public:
    Qt4_Widget_OpenGL( QWidget * parent = 0 );
    ~Qt4_Widget_OpenGL();
    Qt4_Widget_OpenGL( const Qt4_Widget_OpenGL& other );
    Qt4_Widget_OpenGL& operator=( const Qt4_Widget_OpenGL& other );

    opengl::OpenGL_Scene& opengl_scene( void );
    const opengl::OpenGL_Scene& opengl_scene( void )const;

  protected:
    virtual void initializeGL();
    virtual void resizeGL( int width, int height );
    virtual void paintGL();

    virtual void mouseMoveEvent( QMouseEvent * event );
    virtual void mousePressEvent( QMouseEvent * event );
    virtual void mouseReleaseEvent( QMouseEvent * event );

    virtual void raycast( const KDL::Vector eyePosition, const KDL::Vector clickPosition );

    opengl::OpenGL_Scene _opengl_scene;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_OpenGL& other );
}

#endif /* QT4_WIDGET_OPENGL_H */
