#ifndef QT4_QT4_WIDGET_GFE_OBJECT_H
#define QT4_QT4_WIDGET_GFE_OBJECT_H

#include <iostream>

#include <qt4/qt4_widget_opengl.h>
#include <state/state_gfe.h>
#include <opengl/opengl_object_gfe.h>

namespace qt4 {
  class Qt4_Widget_GFE_Object : public Qt4_Widget_OpenGL {
    Q_OBJECT
  public:
    Qt4_Widget_GFE_Object( QWidget * parent = 0 );
    ~Qt4_Widget_GFE_Object();
    Qt4_Widget_GFE_Object( const Qt4_Widget_GFE_Object& other );
    Qt4_Widget_GFE_Object& operator=( const Qt4_Widget_GFE_Object& other );

  public slots:
    void update_state( state::State_GFE& stateGFE );    

  protected:
    opengl::OpenGL_Object_GFE _opengl_object_gfe;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_GFE_Object& other );
}

#endif /* QT4_QT4_WIDGET_GFE_OBJECT_H */
