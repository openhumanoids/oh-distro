#include "authoring/opengl_object_affordance.h"

using namespace std;
using namespace Eigen;
using namespace affordance;
using namespace opengl;
using namespace authoring;

OpenGL_Object_Affordance::
OpenGL_Object_Affordance() : OpenGL_Object(),
                              _affordance_state(),
                              _opengl_object_box(),
                              _opengl_object_cylinder(),
                              _opengl_object_sphere(){

}

OpenGL_Object_Affordance::
~OpenGL_Object_Affordance() {

}

OpenGL_Object_Affordance::
OpenGL_Object_Affordance( const OpenGL_Object_Affordance& other ) : OpenGL_Object( other ),
                                                                    _affordance_state( other._affordance_state ){

}

OpenGL_Object_Affordance&
OpenGL_Object_Affordance::
operator=( const OpenGL_Object_Affordance& other ) {

  return (*this);
}

void
OpenGL_Object_Affordance::
set( AffordanceState& affordanceState ){
  _affordance_state = affordanceState;
  if( _affordance_state.getType() == AffordanceState::CYLINDER ){
    _opengl_object_cylinder.set( affordanceState.getFrame(),
                                  Vector2f( affordanceState._params[ AffordanceState::RADIUS_NAME ],
                                            affordanceState._params[ AffordanceState::LENGTH_NAME ] ) );
  } else if ( _affordance_state.getType() == AffordanceState::LEVER ){

  } else if ( _affordance_state.getType() == AffordanceState::SPHERE ){
    _opengl_object_sphere.set( affordanceState.getFrame(),
                                affordanceState._params[ AffordanceState::RADIUS_NAME ] );
  } else if ( _affordance_state.getType() == AffordanceState::BOX ){
    _opengl_object_box.set( affordanceState.getFrame(),
                            Vector3f( affordanceState._params[ AffordanceState::LENGTH_NAME ],
                                      affordanceState._params[ AffordanceState::WIDTH_NAME ],
                                      affordanceState._params[ AffordanceState::HEIGHT_NAME ] ) );
  }
  return;
}

void
OpenGL_Object_Affordance::
set_transparency( double transparency ){
  _opengl_object_box.set_transparency( transparency );
  _opengl_object_cylinder.set_transparency( transparency );
  _opengl_object_sphere.set_transparency( transparency );
  return;
}

void
OpenGL_Object_Affordance::
draw( void ){
  if( visible() ){
    if( _affordance_state.getType() == AffordanceState::CYLINDER ){
      _opengl_object_cylinder.draw();
    } else if ( _affordance_state.getType() == AffordanceState::LEVER ){
    } else if ( _affordance_state.getType() == AffordanceState::SPHERE ){
      _opengl_object_sphere.draw();
    } else if ( _affordance_state.getType() == AffordanceState::BOX ){
      _opengl_object_box.draw();
    }
  }
  return;
}

void
OpenGL_Object_Affordance::
set_color( Vector3f color ){
  _opengl_object_cylinder.set_color( color );
  _opengl_object_sphere.set_color( color );
  _opengl_object_box.set_color( color );
  return;
}

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Affordance& other ) {
    return out;
  }

}
