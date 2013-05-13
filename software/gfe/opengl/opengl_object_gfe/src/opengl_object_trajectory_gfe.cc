/**
 * @file    opengl_object_trajectory_gfe.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to visualize a GFE trajectory
 */

#include <GL/gl.h>

#include "opengl/opengl_object_trajectory_gfe.h"

using namespace std;
using namespace KDL;
using namespace kinematics;
using namespace state;
using namespace opengl;

OpenGL_Object_Trajectory_GFE::
OpenGL_Object_Trajectory_GFE( void ) : OpenGL_Object(),
                                  _trajectory(),
                                  _opengl_object_gfe(),
                                  _opengl_object_gfe_ghost(),
                                  _current_index( 0 ),
                                  _visible_current_index( true ),
                                  _visible_trajectory( true ),
                                  _visible_trajectory_wrist( false ){
  _opengl_object_gfe_ghost.set_transparency( 0.25 );
}

OpenGL_Object_Trajectory_GFE::
OpenGL_Object_Trajectory_GFE( std::string urdfFilename ) : OpenGL_Object(),
                                                                                        _trajectory(),
                                                                                        _opengl_object_gfe( urdfFilename ),
                                                                                        _current_index( 0 ),
                                                                                        _visible_current_index( true ),
                                                                                        _visible_trajectory( true ),
                                                                                        _visible_trajectory_wrist( false ){

}

OpenGL_Object_Trajectory_GFE::
~OpenGL_Object_Trajectory_GFE() {

}

OpenGL_Object_Trajectory_GFE::
OpenGL_Object_Trajectory_GFE( const OpenGL_Object_Trajectory_GFE& other ) {

}

OpenGL_Object_Trajectory_GFE&
OpenGL_Object_Trajectory_GFE::
operator=( const OpenGL_Object_Trajectory_GFE& other ) {
  
  return (*this);
}

void
OpenGL_Object_Trajectory_GFE::
set( const vector< State_GFE >& trajectory ){
  _trajectory = trajectory;
  return;
}

void
OpenGL_Object_Trajectory_GFE::
draw( void ){
  if( visible() ){
    Kinematics_Model_GFE& kinematics_model = _opengl_object_gfe.kinematics_model();
    if( _visible_current_index ){
      if( _current_index < _trajectory.size() ){
        _opengl_object_gfe.set( _trajectory[ _current_index ] );
        _opengl_object_gfe.draw();
      }
    }
    if( _visible_trajectory ){
      for( unsigned int i = 0; i < _trajectory.size(); i++ ){
        _opengl_object_gfe_ghost.set( _trajectory[ i ] );
        _opengl_object_gfe_ghost.draw();
      }
    }
    if( _visible_trajectory_wrist ){
      glDisable( GL_LIGHTING );
      glLineWidth( 5.0 );
      glColor4f( 0.0, 1.0, 0.0, 1.0 );
      glBegin( GL_LINE_STRIP );
      for( unsigned int i = 0; i < _trajectory.size(); i++ ){
        kinematics_model.set( _trajectory[ i ] );
        Frame frame = kinematics_model.link( "l_hand" );
        glVertex3f( frame.p( 0 ), frame.p( 1 ), frame.p( 2 ) ); 
      }
      glEnd();
      glBegin( GL_LINE_STRIP );
      for( unsigned int i = 0; i < _trajectory.size(); i++ ){
        kinematics_model.set( _trajectory[ i ] );
        Frame frame = kinematics_model.link( "r_hand" );
        glVertex3f( frame.p( 0 ), frame.p( 1 ), frame.p( 2 ) );
      } 
      glEnd(); 
      glBegin( GL_LINE_STRIP );
      for( unsigned int i = 0; i < _trajectory.size(); i++ ){
        kinematics_model.set( _trajectory[ i ] );
        Frame frame = kinematics_model.link( "l_foot" );
        glVertex3f( frame.p( 0 ), frame.p( 1 ), frame.p( 2 ) );
      }
      glEnd();
      glBegin( GL_LINE_STRIP );
      for( unsigned int i = 0; i < _trajectory.size(); i++ ){
        kinematics_model.set( _trajectory[ i ] );
        Frame frame = kinematics_model.link( "r_foot" );
        glVertex3f( frame.p( 0 ), frame.p( 1 ), frame.p( 2 ) );
      }
      glEnd();
      glLineWidth( 1.0 );
      glEnable( GL_LIGHTING );
    }
  }
  return;
}

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Trajectory_GFE& other ) {
    return out;
  }

}
