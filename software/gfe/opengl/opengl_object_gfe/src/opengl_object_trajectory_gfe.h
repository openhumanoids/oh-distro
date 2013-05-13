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
  class OpenGL_Object_Trajectory_GFE: public OpenGL_Object {
  public:
    OpenGL_Object_Trajectory_GFE( void );
    OpenGL_Object_Trajectory_GFE( std::string urdfFilename );
    ~OpenGL_Object_Trajectory_GFE();
    OpenGL_Object_Trajectory_GFE( const OpenGL_Object_Trajectory_GFE& other );
    OpenGL_Object_Trajectory_GFE& operator=( const OpenGL_Object_Trajectory_GFE& other );

    void set( const std::vector< state::State_GFE >& trajectory );
    inline void set_current_index( unsigned int currentIndex ){ _current_index = currentIndex; return; };
    inline void set_visible_current_index( bool visibleCurrentIndex ){ _visible_current_index = visibleCurrentIndex; return; };      
    inline void set_visible_trajectory( bool visibleTrajectory ){ _visible_trajectory = visibleTrajectory; return; };      
    inline void set_visible_trajectory_wrist( bool visibleTrajectoryWrist ){ _visible_trajectory_wrist = visibleTrajectoryWrist; return; };      

    virtual void draw( void );

  protected:
    std::vector< state::State_GFE > _trajectory;
    OpenGL_Object_GFE _opengl_object_gfe;
    OpenGL_Object_GFE _opengl_object_gfe_ghost;
    unsigned int _current_index;
    bool _visible_current_index;
    bool _visible_trajectory;
    bool _visible_trajectory_wrist;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Trajectory_GFE& other );
}

#endif /* OPENGL_OPENGL_OBJECT_TRAJECTORY_GFE_H */
