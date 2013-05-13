/**
 * @file    opengl_object_trajectory_gfe.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to visualize a GFE trajectory
 */

#ifndef OPENGL_OPENGL_OBJECT_TRAJECTORY_GFE_H
#define OPENGL_OPENGL_OBJECT_TRAJECTORY_GFE_H

#include <iostream>

#include <state/state_gfe.h>

#include "opengl/opengl_object.h"
#include "opengl/opengl_object_gfe.h"

namespace opengl {
  typedef enum {
    OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_LINK_PATHS,
    OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_WRIST_PATHS,
    NUM_OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_MODES
  } opengl_object_trajectory_gfe_render_mode_t;

  class OpenGL_Object_Trajectory_GFE: public OpenGL_Object {
  public:
    OpenGL_Object_Trajectory_GFE( opengl_object_trajectory_gfe_render_mode_t renderMode = OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_LINK_PATHS );
    OpenGL_Object_Trajectory_GFE( std::string urdfFilename, opengl_object_trajectory_gfe_render_mode_t renderMode = OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_LINK_PATHS );
    ~OpenGL_Object_Trajectory_GFE();
    OpenGL_Object_Trajectory_GFE( const OpenGL_Object_Trajectory_GFE& other );
    OpenGL_Object_Trajectory_GFE& operator=( const OpenGL_Object_Trajectory_GFE& other );

    void set( const std::vector< state::State_GFE >& trajectory );
    void set_current_index( unsigned int currentIndex );

    virtual void draw( void );

  protected:
    opengl_object_trajectory_gfe_render_mode_t _render_mode;
    std::vector< state::State_GFE > _trajectory;
    OpenGL_Object_GFE _opengl_object_gfe;
    OpenGL_Object_GFE _opengl_object_gfe_ghost;
    unsigned int _current_index;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Trajectory_GFE& other );
}

#endif /* OPENGL_OPENGL_OBJECT_TRAJECTORY_GFE_H */
