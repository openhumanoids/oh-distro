#include "qt4/qt4_widget_gfe_object.h"

using namespace std;
using namespace state;
using namespace qt4;

Qt4_Widget_GFE_Object::
Qt4_Widget_GFE_Object( QWidget * parent ) : Qt4_Widget_OpenGL( parent ),
                                            _opengl_object_gfe() {
  State_GFE state_gfe;
  state_gfe.from_urdf();
  _opengl_object_gfe.set( state_gfe );
  opengl_scene().add_object( _opengl_object_gfe );
}

Qt4_Widget_GFE_Object::
~Qt4_Widget_GFE_Object() {

}

Qt4_Widget_GFE_Object::
Qt4_Widget_GFE_Object( const Qt4_Widget_GFE_Object& other ) {

}

Qt4_Widget_GFE_Object&
Qt4_Widget_GFE_Object::
operator=( const Qt4_Widget_GFE_Object& other ) {

  return (*this);
}

void
Qt4_Widget_GFE_Object::
update_state( State_GFE& stateGFE ){
  _opengl_object_gfe.set( stateGFE );
  update();
  return;
}

namespace qt4 {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_GFE_Object& other ) {
    return out;
  }

}
