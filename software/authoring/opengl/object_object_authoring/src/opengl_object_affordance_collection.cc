#include "authoring/opengl_object_affordance_collection.h"

using namespace std;
using namespace Eigen;
using namespace affordance;
using namespace opengl;
using namespace authoring;

OpenGL_Object_Affordance_Collection::
OpenGL_Object_Affordance_Collection() : OpenGL_Object(),
                                        _opengl_object_affordance(),
                                        _affordance_collection(),
                                        _highlight_ids(){

}

OpenGL_Object_Affordance_Collection::
~OpenGL_Object_Affordance_Collection() {

}

OpenGL_Object_Affordance_Collection::
OpenGL_Object_Affordance_Collection( const OpenGL_Object_Affordance_Collection& other ) : OpenGL_Object( other ),
                                                                                          _opengl_object_affordance( other._opengl_object_affordance ),
                                                                                          _affordance_collection( other._affordance_collection ),
                                                                                          _highlight_ids( other._highlight_ids ){

}

OpenGL_Object_Affordance_Collection&
OpenGL_Object_Affordance_Collection::
operator=( const OpenGL_Object_Affordance_Collection& other ) {
  _opengl_object_affordance = other._opengl_object_affordance;
  _affordance_collection = other._affordance_collection;
  _highlight_ids = other._highlight_ids;
  return (*this);
}

void
OpenGL_Object_Affordance_Collection::
set( const vector< AffordanceState >& affordanceCollection ){
  _affordance_collection = affordanceCollection;
  return;
}

void
OpenGL_Object_Affordance_Collection::
draw( void ){
  if( visible() ){
    for( vector< AffordanceState >::iterator it = _affordance_collection.begin(); it != _affordance_collection.end(); ++it ){
      _opengl_object_affordance.set( *it );
      bool is_highlighted = false;
      for( unsigned int i = 0; i < _highlight_ids.size(); i++ ){
        if( _highlight_ids[ i ] == it->getName() ){
          is_highlighted = true;
        }
      }
      if( is_highlighted ){
        _opengl_object_affordance.set_color( Eigen::Vector3f( 1.0, 0.0, 0.0 ) );
      } else {
        _opengl_object_affordance.set_color( Eigen::Vector3f( 1.0, 1.0, 1.0 ) );
      }
      _opengl_object_affordance.draw();
    } 
  }
  return;
}

void
OpenGL_Object_Affordance_Collection::
set_color( Vector3f color ){
  _opengl_object_affordance.set_color( color );
  return;
}

void
OpenGL_Object_Affordance_Collection::
set_transparency( double transparency ){
  _opengl_object_affordance.set_transparency( transparency );
  return;
}

void
OpenGL_Object_Affordance_Collection::
set_highlight( const std::vector< std::string >& highlightIds ){
  _highlight_ids = highlightIds;
  return;
}

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Affordance_Collection& other ) {
    return out;
  }

}
