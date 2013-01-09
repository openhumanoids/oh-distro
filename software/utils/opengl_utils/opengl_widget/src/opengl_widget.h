#ifndef OPENGL_OPENGL_WIDGET_H
#define OPENGL_OPENGL_WIDGET_H

#include <iostream>

#include <QtOpenGL/QGLWidget>

#include <opengl_scene/opengl_scene.h>

namespace opengl {
  class OpenGL_Widget: public QGLWidget {
    Q_OBJECT
  public:
    OpenGL_Widget( QWidget * parent = 0 );
    ~OpenGL_Widget();
    OpenGL_Widget( const OpenGL_Widget& other );
    OpenGL_Widget& operator=( const OpenGL_Widget& other );

    OpenGL_Scene& opengl_scene( void );
    const OpenGL_Scene& opengl_scene( void )const;

  protected:
    virtual void initializeGL();
    virtual void resizeGL( int width, int height );
    virtual void paintGL();

    virtual void mouseMoveEvent( QMouseEvent * event );
    virtual void mousePressEvent( QMouseEvent * event );
    virtual void mouseReleaseEvent( QMouseEvent * event );

    OpenGL_Scene _opengl_scene;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Widget& other );
}

#endif /* OPENGL_OPENGL_WIDGET_H */
