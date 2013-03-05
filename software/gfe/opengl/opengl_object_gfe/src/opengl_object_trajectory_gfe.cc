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
using namespace tmh_types;
using namespace KDL;
using namespace kinematics;
using namespace state;
using namespace opengl;

OpenGL_Object_Trajectory_GFE::
OpenGL_Object_Trajectory_GFE() : OpenGL_Object(),
                                  _render_mode( OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_WRIST_PATHS ),
                                  _trajectory(),
                                  _opengl_object_gfe(){
  _opengl_object_gfe.set_transparency( 0.25 );
}

OpenGL_Object_Trajectory_GFE::
OpenGL_Object_Trajectory_GFE( std::string urdfFilename ) : OpenGL_Object(),
                                                            _trajectory(),
                                                            _opengl_object_gfe( urdfFilename ){

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
set( const Trajectory< State_GFE >& trajectory ){
  _trajectory = trajectory;
  return;
}

void
OpenGL_Object_Trajectory_GFE::
draw( void ){
  if( visible() ){
    Kinematics_Model_GFE& kinematics_model = _opengl_object_gfe.kinematics_model();
    if( _render_mode == OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_LINK_PATHS ){
      for( unsigned int i = 0; i < _trajectory.size(); i++ ){
        _opengl_object_gfe.set( _trajectory[ i ] );
        _opengl_object_gfe.draw();
      }
    } else if ( _render_mode == OPENGL_OBJECT_TRAJECTORY_GFE_RENDER_WRIST_PATHS ){ 
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
      if( _trajectory.size() > 0 ){
        _opengl_object_gfe.set( _trajectory[ _trajectory.size() - 1 ] );
        _opengl_object_gfe.draw();
      }
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
