/**
 * @file    opengl_object_constraint_sequence.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to draw a constraint sequence
 */

#include "authoring/opengl_object_constraint_sequence.h"

using namespace std;
using namespace opengl;
using namespace authoring;

OpenGL_Object_Constraint_Sequence::
OpenGL_Object_Constraint_Sequence() : OpenGL_Object(),
                                      _constraint_sequence(),
                                      _opengl_object_constraint_task_space_region(),
                                      _highlight_ids() {

}

OpenGL_Object_Constraint_Sequence::
~OpenGL_Object_Constraint_Sequence() {

}

OpenGL_Object_Constraint_Sequence::
OpenGL_Object_Constraint_Sequence( const OpenGL_Object_Constraint_Sequence& other ) : OpenGL_Object( other ),
                                                                                      _constraint_sequence( other._constraint_sequence ),
                                                                                      _opengl_object_constraint_task_space_region( other._opengl_object_constraint_task_space_region ),
                                                                                      _highlight_ids( other._highlight_ids ) {

}

OpenGL_Object_Constraint_Sequence&
OpenGL_Object_Constraint_Sequence::
operator=( const OpenGL_Object_Constraint_Sequence& other ) {
  _constraint_sequence = other._constraint_sequence;
  _opengl_object_constraint_task_space_region = other._opengl_object_constraint_task_space_region;
  _highlight_ids = other._highlight_ids;
  return (*this);
}

void 
OpenGL_Object_Constraint_Sequence::
set( const Constraint_Sequence& constraintSequence ){
  _constraint_sequence = constraintSequence;
  return;
}

void
OpenGL_Object_Constraint_Sequence::
set_transparency( double transparency ){
  _transparency = transparency;
  _opengl_object_constraint_task_space_region.set_transparency( transparency );
}

void
OpenGL_Object_Constraint_Sequence::
set_highlight( const std::vector< std::string >& highlightIds ){
  _highlight_ids = highlightIds;
  return;
}

void 
OpenGL_Object_Constraint_Sequence::
draw( void ){
  if( visible() ){
    for( vector< Constraint_Task_Space_Region >::const_iterator it = _constraint_sequence.constraints().begin(); it != _constraint_sequence.constraints().end(); it++ ){
      _opengl_object_constraint_task_space_region.set( *it );
      bool is_highlighted = false;
      for( unsigned int i = 0; i < _highlight_ids.size(); i++ ){
        if( _highlight_ids[ i ] == it->id() ){
          is_highlighted = true;
        }
      }
      if( is_highlighted ){
        _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 0.0, 1.0, 1.0 ) );
      } else {
        _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 1.0, 1.0, 0.0 ) );
      }
      _opengl_object_constraint_task_space_region.draw();
    }
  }
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Constraint_Sequence& other ) {
    return out;
  }

}
