#include "authoring/qt4_widget_opengl_authoring.h"

using namespace std;
using namespace qt4;
using namespace authoring;

Qt4_Widget_OpenGL_Authoring::
Qt4_Widget_OpenGL_Authoring( QWidget * parent ) : Qt4_Widget_OpenGL( parent ),
                                                  _opengl_object_affordance_collection(),
                                                  _opengl_object_affordance_collection_ghost() {
  _opengl_object_affordance_collection_ghost.set_transparency( 0.1 );
  opengl_scene().add_object( _opengl_object_affordance_collection );
  opengl_scene().add_object( _opengl_object_affordance_collection_ghost );
}

Qt4_Widget_OpenGL_Authoring::
~Qt4_Widget_OpenGL_Authoring() {

}

Qt4_Widget_OpenGL_Authoring::
Qt4_Widget_OpenGL_Authoring( const Qt4_Widget_OpenGL_Authoring& other ) {

}

Qt4_Widget_OpenGL_Authoring&
Qt4_Widget_OpenGL_Authoring::
operator=( const Qt4_Widget_OpenGL_Authoring& other ) {

  return (*this);
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_affordance_collection( vector< affordance::AffordanceState >& affordanceCollection ){
  _opengl_object_affordance_collection.set( affordanceCollection );
  update();
  return;
}

void 
Qt4_Widget_OpenGL_Authoring::
update_opengl_object_affordance_collection_ghost( vector< affordance::AffordanceState >& affordanceCollection ){
  _opengl_object_affordance_collection_ghost.set( affordanceCollection );
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_OpenGL_Authoring& other ) {
    return out;
  }

}
