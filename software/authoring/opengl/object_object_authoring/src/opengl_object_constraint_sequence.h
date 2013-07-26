/**
 * @file    opengl_object_constraint_sequence.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to draw a constraint sequence
 */

#ifndef AUTHORING_OPENGL_OBJECT_CONSTRAINT_SEQUENCE_H
#define AUTHORING_OPENGL_OBJECT_CONSTRAINT_SEQUENCE_H

#include <iostream>

#include "opengl/opengl_object.h"

#include <authoring/constraint_sequence.h>
#include <authoring/opengl_object_constraint_task_space_region.h>

#include <collision/collision_detector.h>

namespace authoring {

  // a highlight of a higher index shows up over one of a lower index
  enum highlight_class_t{
    HIGHLIGHT_NONE = -1,
    HIGHLIGHT_RED = 0,
    HIGHLIGHT_GREEN = 1,
    HIGHLIGHT_BLUE = 2,
    HIGHLIGHT_PURPLE = 3
  };

  class OpenGL_Object_Constraint_Sequence: public opengl::OpenGL_Object {
  public:
    OpenGL_Object_Constraint_Sequence();
    ~OpenGL_Object_Constraint_Sequence();
    OpenGL_Object_Constraint_Sequence( const OpenGL_Object_Constraint_Sequence& other );
    OpenGL_Object_Constraint_Sequence& operator=( const OpenGL_Object_Constraint_Sequence& other );

    virtual void set( const Constraint_Sequence& constraintSequence );
    virtual void set_affordance_collection( const std::vector< affordance::AffordanceState >& affordanceCollection );
    virtual void set_transparency( double transparency );
    virtual void set_highlight( const std::vector< std::string >& highlightIds, highlight_class_t highlight_class = HIGHLIGHT_BLUE );
    virtual void draw( void );
    virtual void add_to_collision( collision::Collision_Detector& detector );
    virtual Constraint_Task_Space_Region * get_constraint_task_space_region( const std::string target_id );
  protected:
    Constraint_Sequence _constraint_sequence;
    std::vector< affordance::AffordanceState > _affordance_collection;
    OpenGL_Object_Constraint_Task_Space_Region _opengl_object_constraint_task_space_region;
    std::vector< std::vector< std::string > > _highlight_ids_strings;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Constraint_Sequence& other );
}

#endif /* AUTHORING_OPENGL_OBJECT_CONSTRAINT_SEQUENCE_H */
