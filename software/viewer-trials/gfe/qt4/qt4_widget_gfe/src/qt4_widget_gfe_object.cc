#include "qt4/qt4_widget_gfe_object.h"

using namespace std;
using namespace state;
using namespace collision;
using namespace qt4;

Qt4_Widget_GFE_Object::
Qt4_Widget_GFE_Object( QWidget * parent ) : Qt4_Widget_OpenGL( parent ),
                                            _opengl_object_gfe(),
                                            _collision_object_gfe() {
  opengl_scene().add_object( _opengl_object_gfe );
  for( unsigned int i = 0; i < _collision_object_gfe.collision_objects().size(); i++ ){
    add_collision_object( *_collision_object_gfe.collision_objects()[ i ] );
  }
}

Qt4_Widget_GFE_Object::
Qt4_Widget_GFE_Object( const string& xmlString,
                        QWidget * parent ) : Qt4_Widget_OpenGL( parent ),
                                              _opengl_object_gfe( xmlString ),
                                              _collision_object_gfe( "co-gfe", xmlString, COLLISION_OBJECT_GFE_COLLISION_OBJECT_VISUAL ) {
  opengl_scene().add_object( _opengl_object_gfe );
  for( unsigned int i = 0; i < _collision_object_gfe.collision_objects().size(); i++ ){
    add_collision_object( *_collision_object_gfe.collision_objects()[ i ] );
  }
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
  _collision_object_gfe.set( stateGFE );
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
