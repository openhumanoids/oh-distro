/**
 * @file    opengl_object_constraint_task_space_region.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to draw a constraint task space region
 */

#include "authoring/opengl_object_constraint_task_space_region.h"

using namespace std;
using namespace KDL;
using namespace opengl;
using namespace authoring;

OpenGL_Object_Constraint_Task_Space_Region::
OpenGL_Object_Constraint_Task_Space_Region() : OpenGL_Object(),
                                                _opengl_object_box() {

}

OpenGL_Object_Constraint_Task_Space_Region::
~OpenGL_Object_Constraint_Task_Space_Region() {

}

OpenGL_Object_Constraint_Task_Space_Region::
OpenGL_Object_Constraint_Task_Space_Region( const OpenGL_Object_Constraint_Task_Space_Region& other ) : OpenGL_Object( other ),
                                                                                                        _opengl_object_box( other._opengl_object_box ){

}

OpenGL_Object_Constraint_Task_Space_Region&
OpenGL_Object_Constraint_Task_Space_Region::
operator=( const OpenGL_Object_Constraint_Task_Space_Region& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _offset = other._offset;
  _opengl_object_box = other._opengl_object_box;
  return (*this);
}

void 
OpenGL_Object_Constraint_Task_Space_Region::
set( const Constraint_Task_Space_Region& constraint ){
  _opengl_object_box.set_visible( constraint.active() && constraint.visible() );

  double xmin = constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].first ? constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second : -1000.0;
  double xmax = constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].first ? constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second : 1000.0;
  double ymin = constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].first ? constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second : -1000.0;
  double ymax = constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].first ? constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second : 1000.0;
  double zmin = constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].first ? constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second : -1000.0;
  double zmax = constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].first ? constraint.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second : 1000.0;

  _opengl_object_box.set( constraint.offset() * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), 
                                                        Vector( 0.5 * ( xmax + xmin ), 0.5 * ( ymax + ymin ), 0.5 * ( zmax + zmin ) ) ),
                          Eigen::Vector3f( fabs( xmax - xmin ) + 0.001, fabs( ymax - ymin ) + 0.001, fabs( zmax - zmin ) + 0.001 ) ); 
  return;
} 

void 
OpenGL_Object_Constraint_Task_Space_Region::
set_transparency( double transparency ){
  _opengl_object_box.set_transparency( transparency );
  return;
} 

void 
OpenGL_Object_Constraint_Task_Space_Region::
set_color( Eigen::Vector3f color ){
  _opengl_object_box.set_color( color );
  return;
}
    
void 
OpenGL_Object_Constraint_Task_Space_Region::
draw( void ){
  if( visible() ){
    _opengl_object_box.draw();
  }
  return;
}


namespace authoring {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Constraint_Task_Space_Region& other ) {
    return out;
  }

}
