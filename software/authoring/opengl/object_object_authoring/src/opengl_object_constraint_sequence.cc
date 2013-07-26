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
using namespace affordance;
using namespace authoring;
using namespace collision;

OpenGL_Object_Constraint_Sequence::
OpenGL_Object_Constraint_Sequence() : OpenGL_Object(),
                                      _constraint_sequence(),
                                      _opengl_object_constraint_task_space_region(),
                                      _highlight_ids_strings() {
  _highlight_ids_strings.push_back(vector< std::string >()); //HIGHLIGHT_RED
  _highlight_ids_strings.push_back(vector< std::string >()); //HIGHLIGHT_GREEN
  _highlight_ids_strings.push_back(vector< std::string >()); //HIGHLIGHT_BLUE
  _highlight_ids_strings.push_back(vector< std::string >()); //HIGHLIGHT_PURPLE
}

OpenGL_Object_Constraint_Sequence::
~OpenGL_Object_Constraint_Sequence() {

}

OpenGL_Object_Constraint_Sequence::
OpenGL_Object_Constraint_Sequence( const OpenGL_Object_Constraint_Sequence& other ) : OpenGL_Object( other ),
                                                                                      _constraint_sequence( other._constraint_sequence ),
                                                                                      _opengl_object_constraint_task_space_region( other._opengl_object_constraint_task_space_region ),
                                                                                      _highlight_ids_strings( other._highlight_ids_strings ) {

}

OpenGL_Object_Constraint_Sequence&
OpenGL_Object_Constraint_Sequence::
operator=( const OpenGL_Object_Constraint_Sequence& other ) {
  _constraint_sequence = other._constraint_sequence;
  _opengl_object_constraint_task_space_region = other._opengl_object_constraint_task_space_region;
  _highlight_ids_strings = other._highlight_ids_strings;
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
set_affordance_collection( const vector< AffordanceState >& affordanceCollection ){
  _affordance_collection = affordanceCollection;
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
set_highlight( const std::vector< std::string >& highlightIds, highlight_class_t highlight_class ){
  if (highlight_class > HIGHLIGHT_PURPLE){
    printf("Unknown highlight class.\n");
  }
  _highlight_ids_strings[(int)highlight_class] = highlightIds;
  return;
}

void 
OpenGL_Object_Constraint_Sequence::
draw( void ){
  if( visible() ){
    for( vector< Constraint_Task_Space_Region >::const_iterator it = _constraint_sequence.constraints().begin(); it != _constraint_sequence.constraints().end(); it++ ){
      if( it->visible() ){
        bool found_one = false;
        for( unsigned int i = 0; i < _affordance_collection.size(); i++ ){
          if( _affordance_collection[ i ].getName() == it->child() ){
            _opengl_object_constraint_task_space_region.set_affordance( _affordance_collection[ i ] );
            found_one = true;
            break;
          }
        }
        _opengl_object_constraint_task_space_region.set( *it );
        int is_highlighted = HIGHLIGHT_NONE;
        for( unsigned int hgstr = 0; hgstr < _highlight_ids_strings.size(); hgstr++){
          for( unsigned int i = 0; i < _highlight_ids_strings[hgstr].size(); i++ ){
            if( _highlight_ids_strings[hgstr][ i ] == it->id() ){
              is_highlighted = hgstr;
            }
          }
        }
        if (!found_one){
          _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 1.0, 0.0, 0.0 ) );
        } else if( is_highlighted != -1 ){
          switch (is_highlighted){
            case HIGHLIGHT_RED:
              _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 1.0, 0.0, 0.0 ) );
              break;
            case HIGHLIGHT_BLUE:
              _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 0.0, 1.0, 1.0 ) );
              break;
            case HIGHLIGHT_GREEN:
              _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 0.0, 1.0, 0.0 ) );
              break;
            case HIGHLIGHT_PURPLE:
              _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 1.0, 0.0, 1.0 ) );
              break;
            default:
              printf("Somehow got invalid is_highlighted value.\n");
              break;
          }
        } else {
          _opengl_object_constraint_task_space_region.set_color( Eigen::Vector3f( 1.0, 1.0, 0.0 ) );
        }
        _opengl_object_constraint_task_space_region.draw();
      }
    }
  }
  return;
}

/* Add each constraint to specified collision detector */
void
OpenGL_Object_Constraint_Sequence::
add_to_collision( Collision_Detector& detector ){
  int iter = 0;
  char tmpstring[100];
  if( visible() ){
    for( vector< Constraint_Task_Space_Region >::const_iterator it = _constraint_sequence.constraints().begin(); it != _constraint_sequence.constraints().end(); it++ ){
      if( it->visible() ){
        for( unsigned int i = 0; i < _affordance_collection.size(); i++ ){
          if( _affordance_collection[ i ].getName() == it->child() ){
            _opengl_object_constraint_task_space_region.set_affordance( _affordance_collection[ i ] );
          }
        }
        _opengl_object_constraint_task_space_region.set( *it );
        sprintf(tmpstring, "C%d", iter);
        _opengl_object_constraint_task_space_region.add_to_collision(detector, tmpstring);
        //printf("Added %s to collision model...\n", tmpstring);
        iter++;
      }
    }
  }
  return;
}

/* Return first constraint with matching id, NULL if none */
Constraint_Task_Space_Region *
OpenGL_Object_Constraint_Sequence::
get_constraint_task_space_region( const std::string target_id ){
  for( vector< Constraint_Task_Space_Region >::iterator it = _constraint_sequence.constraints().begin(); it != _constraint_sequence.constraints().end(); it++ ){
    if( it->visible() ){
      if (it->id() == target_id)
        return &*it;
    }
  }
  return NULL;
}

/* Return the centroid of the box rendered  */
/*
Vector
OpenGL_Object_Constraint_Sequence::
get_constraint_task_space_region_centroid( const std::string target_id ){
  OpenGL
  Constraint_Task_Space_Region rightone;
  for( vector< OpenGL_ObjectConstraint_Task_Space_Region >::iterator it = _opengl_object_constraint_task_space_region.constraints().begin(); it != _constraint_sequence.constraints().end(); it++ ){
    if( it->visible() ){
      if (it->id() == target_id)
        // found it. Now find its child...
        for( unsigned int i = 0; i < _affordance_collection.size(); i++ ){
          if( _affordance_collection[ i ].getName() == it->child() ){
            // found it.

          }
        }
        break;
    }
  }
  /* and its child 
  return NULL;
}
*/

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Constraint_Sequence& other ) {
    return out;
  }

}
