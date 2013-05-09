#include "authoring/opengl_object_affordance_collection.h"

using namespace std;
using namespace Eigen;
using namespace affordance;
using namespace opengl;
using namespace authoring;

OpenGL_Object_Affordance_Collection::
OpenGL_Object_Affordance_Collection() : OpenGL_Object(),
                                        _opengl_object_affordance(),
                                        _affordance_collection(){

}

OpenGL_Object_Affordance_Collection::
~OpenGL_Object_Affordance_Collection() {

}

OpenGL_Object_Affordance_Collection::
OpenGL_Object_Affordance_Collection( const OpenGL_Object_Affordance_Collection& other ) : OpenGL_Object( other ),
                                                                                          _opengl_object_affordance( other._opengl_object_affordance ),
                                                                                          _affordance_collection( other._affordance_collection ){

}

OpenGL_Object_Affordance_Collection&
OpenGL_Object_Affordance_Collection::
operator=( const OpenGL_Object_Affordance_Collection& other ) {
  _opengl_object_affordance = other._opengl_object_affordance;
  _affordance_collection = other._affordance_collection;
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

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Affordance_Collection& other ) {
    return out;
  }

}
