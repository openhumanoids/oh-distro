#ifndef AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H
#define AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H

#include <iostream>
#include <vector>

#include <qt4/qt4_widget_opengl.h>

#include <affordance/AffordanceState.h>
#include <opengl/opengl_object_gfe.h>
#include <state/state_gfe.h>

#include <authoring/opengl_object_affordance_collection.h>

namespace authoring {
  class Qt4_Widget_OpenGL_Authoring : public qt4::Qt4_Widget_OpenGL {
    Q_OBJECT
  public:
    Qt4_Widget_OpenGL_Authoring( QWidget * parent = 0 );
    ~Qt4_Widget_OpenGL_Authoring();
    Qt4_Widget_OpenGL_Authoring( const Qt4_Widget_OpenGL_Authoring& other );
    Qt4_Widget_OpenGL_Authoring& operator=( const Qt4_Widget_OpenGL_Authoring& other );

  public slots:
    void update_opengl_object_affordance_collection( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_affordance_collection_ghost( std::vector< affordance::AffordanceState >& affordanceCollection );
    void update_opengl_object_gfe( state::State_GFE& stateGFE );
    void update_opengl_object_gfe_ghost( state::State_GFE& stateGFE );

  protected:
    OpenGL_Object_Affordance_Collection _opengl_object_affordance_collection;
    OpenGL_Object_Affordance_Collection _opengl_object_affordance_collection_ghost;
    opengl::OpenGL_Object_GFE _opengl_object_gfe;
    opengl::OpenGL_Object_GFE _opengl_object_gfe_ghost;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_OpenGL_Authoring& other );
}

#endif /* AUTHORING_QT4_WIDGET_OPENGL_AUTHORING_H */
