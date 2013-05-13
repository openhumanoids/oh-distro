#include "opengl/opengl_object_coordinate_axis.h"

using namespace std;
using namespace Eigen;
using namespace opengl;

OpenGL_Object_Coordinate_Axis::
OpenGL_Object_Coordinate_Axis( bool drawTranslationAxes,
                                bool drawRotationAxes ) : OpenGL_Object(),
                                                          _scale( 0.1 ),
                                                          _opengl_object_torus(),
                                                          _quadric( NULL ),
                                                          _dl( 0 ),
                                                          _draw_translation_axes( drawTranslationAxes ),
                                                          _draw_rotation_axes( drawRotationAxes ){

}

OpenGL_Object_Coordinate_Axis::
~OpenGL_Object_Coordinate_Axis() {

}

OpenGL_Object_Coordinate_Axis::
OpenGL_Object_Coordinate_Axis( const OpenGL_Object_Coordinate_Axis& other ) : OpenGL_Object( other ),
                                                                              _scale( other._scale ),
                                                                              _quadric( NULL ),
                                                                              _dl( 0 ),
                                                                              _draw_translation_axes( other._draw_translation_axes ),
                                                                              _draw_rotation_axes( other._draw_rotation_axes ){
  
}

OpenGL_Object_Coordinate_Axis&
OpenGL_Object_Coordinate_Axis::
operator=( const OpenGL_Object_Coordinate_Axis& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _offset = other._offset;
  _scale = other._scale;
  _quadric = NULL;
  _dl = 0;
  _draw_translation_axes = other._draw_translation_axes;
  _draw_rotation_axes = other._draw_rotation_axes;
  return (*this);
}

void
OpenGL_Object_Coordinate_Axis::
set_scale( double scale ){
  _scale = scale;
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 1 );
    _dl = 0;
  }
  return;
}

void
OpenGL_Object_Coordinate_Axis::
draw( void ){
  if( visible() ){
    if( glIsList( _dl ) == GL_TRUE ){
      glPushMatrix();
      apply_transform();
      glCallList( _dl );
      glPopMatrix();
    } else if( _generate_dl() ) {
      draw();
    }
  }
  return;
}

bool
OpenGL_Object_Coordinate_Axis::
_generate_dl( void ){
  if( glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 1 );
    _dl = 0;
  }
  if( _quadric == NULL ){
    _quadric = gluNewQuadric();
  }
  _dl = glGenLists( 1 );
  glNewList( _dl, GL_COMPILE );
  // draw the center
  glPushMatrix();
  glColor4f( 1.0, 1.0, 1.0, 1.0 );
  gluSphere( _quadric, _scale/20.0, 8, 8 );
  glPopMatrix();
  if( _draw_translation_axes ){
    // draw the red x-axis
    glPushMatrix();
    glColor4f( 1.0, 0.0, 0.0, 1.0 );
    glRotatef( 90.0, 0.0, 1.0, 0.0 );
    gluCylinder( _quadric, _scale/20.0, _scale/20.0, _scale, 8, 8 );
    glTranslatef( 0.0, 0.0, _scale );
    gluCylinder( _quadric, _scale/10.0, 0.0, _scale/5.0, 8, 8 );
    glPopMatrix();
    // draw the green y-axis
    glPushMatrix();
    glColor4f( 0.0, 1.0, 0.0, 1.0 );
    glRotatef( -90.0, 1.0, 0.0, 0.0 );
    gluCylinder( _quadric, _scale/20.0, _scale/20.0, _scale, 8, 8 );
    glTranslatef( 0.0, 0.0, _scale );
    gluCylinder( _quadric, _scale/10.0, 0.0, _scale/5.0, 8, 8 );
    glPopMatrix();
    // draw the blue z-axis
    glPushMatrix();
    glColor4f( 0.0, 0.0, 1.0, 1.0 );
    gluCylinder( _quadric, _scale/20.0, _scale/20.0, _scale, 8, 8 );
    glTranslatef( 0.0, 0.0, _scale );
    gluCylinder( _quadric, _scale/10.0, 0.0, _scale/5.0, 8, 8 );
    glPopMatrix();
    _opengl_object_torus.set_dimensions( _scale/1.5, _scale/40.0 );
  }
  if( _draw_rotation_axes ){
    // draw the roll ring
    glPushMatrix();
    glRotatef( 90.0, 0.0, 1.0, 0.0 );
    _opengl_object_torus.draw( Vector3f( 1.0, 0.0, 0.0 ) );
    glPopMatrix();
    // draw the pitch ring
    glPushMatrix();
    glRotatef( 90.0, 1.0, 0.0, 0.0 );
    _opengl_object_torus.draw( Vector3f( 0.0, 1.0, 0.0 ) );
    glPopMatrix();
    // draw the yaw ring
    glPushMatrix();
    _opengl_object_torus.draw( Vector3f( 0.0, 0.0, 1.0 ) );
    glPopMatrix();
  }
  glEndList();
  return true;
}

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Coordinate_Axis& other ) {
    return out;
  }

}
