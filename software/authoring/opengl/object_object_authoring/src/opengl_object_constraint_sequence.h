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

namespace authoring {
  class OpenGL_Object_Constraint_Sequence: public opengl::OpenGL_Object {
  public:
    OpenGL_Object_Constraint_Sequence();
    ~OpenGL_Object_Constraint_Sequence();
    OpenGL_Object_Constraint_Sequence( const OpenGL_Object_Constraint_Sequence& other );
    OpenGL_Object_Constraint_Sequence& operator=( const OpenGL_Object_Constraint_Sequence& other );

    virtual void set( const Constraint_Sequence& constraintSequence );
    virtual void set_affordance_collection( const std::vector< affordance::AffordanceState >& affordanceCollection );
    virtual void set_transparency( double transparency );
    virtual void set_highlight( const std::vector< std::string >& highlightIds );
    virtual void draw( void );

  protected:
    Constraint_Sequence _constraint_sequence;
    std::vector< affordance::AffordanceState > _affordance_collection;
    OpenGL_Object_Constraint_Task_Space_Region _opengl_object_constraint_task_space_region;
    std::vector< std::string > _highlight_ids;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Constraint_Sequence& other );
}

#endif /* AUTHORING_OPENGL_OBJECT_CONSTRAINT_SEQUENCE_H */
