#include "opengl/opengl_object_torus.h"

using namespace std;
using namespace Eigen;
using namespace opengl;

OpenGL_Object_Torus::
OpenGL_Object_Torus( double majorRadius,
                      double minorRadius ) : OpenGL_Object(),
                                              _major_radius( majorRadius ),
                                              _minor_radius( minorRadius ),
                                              _quadric( NULL ),
                                              _dl( 0 ){

}

OpenGL_Object_Torus::
~OpenGL_Object_Torus() {

}

OpenGL_Object_Torus::
OpenGL_Object_Torus( const OpenGL_Object_Torus& other ) : OpenGL_Object( other ),
                                                          _major_radius( other._major_radius ),
                                                          _minor_radius( other._minor_radius ),
                                                          _quadric( NULL ),
                                                          _dl( 0 ){

}

OpenGL_Object_Torus&
OpenGL_Object_Torus::
operator=( const OpenGL_Object_Torus& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _major_radius = other._major_radius;
  _minor_radius = other._minor_radius;
  _quadric = NULL;
  _dl = 0;
  return (*this);
}

void
OpenGL_Object_Torus::
set_dimensions( double majorRadius,
                double minorRadius ){
  _major_radius = majorRadius;
  _minor_radius = minorRadius;
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  return;
}

void
OpenGL_Object_Torus::
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

void
OpenGL_Object_Torus::
draw( Vector3f color ){
  if( visible() ){
    glPushMatrix();
    apply_transform();
    glColor4f( color( 0 ), color( 1 ), color( 2 ), transparency() );
    glBegin( GL_QUADS );
    for( unsigned int i = 0; i < 32; i++ ){
      double t1 = 2.0 * M_PI * double( i ) / 32.0;
      double t2 = 2.0 * M_PI * double( i + 1 ) / 32.0;
      for( unsigned int j = 0; j < 32; j++ ){
        double p1 = 2.0 * M_PI * double( j ) / 32.0;
        double p2 = 2.0 * M_PI * double( j + 1 ) / 32.0;

        Vector3f v1( ( _major_radius + _minor_radius * cos( p1 ) ) * cos( t1 ),
                    ( _major_radius + _minor_radius * cos( p1 ) ) * sin( t1 ),
                    _minor_radius * sin( p1 ) );
        Vector3f v2( ( _major_radius + _minor_radius * cos( p2 ) ) * cos( t1 ),
                    ( _major_radius + _minor_radius * cos( p2 ) ) * sin( t1 ),
                    _minor_radius * sin( p2 ) );
        Vector3f v3( ( _major_radius + _minor_radius * cos( p2 ) ) * cos( t2 ),
                    ( _major_radius + _minor_radius * cos( p2 ) ) * sin( t2 ),
                      _minor_radius * sin( p2 ) );
        Vector3f v4( ( _major_radius + _minor_radius * cos( p1 ) ) * cos( t2 ),
                      ( _major_radius + _minor_radius * cos( p1 ) ) * sin( t2 ),
                      _minor_radius * sin( p1 ) );
        Vector3f v21 = v2 - v1;
        Vector3f v31 = v3 - v1;
        Vector3f n = v31.cross( v21 );
        glNormal3f( n( 0 ), n( 1 ), n( 2 ) );
        glVertex3f( v1( 0 ), v1( 1 ), v1( 2 ) );
        glVertex3f( v2( 0 ), v2( 1 ), v2( 2 ) );
        glVertex3f( v3( 0 ), v3( 1 ), v3( 2 ) );
        glVertex3f( v4( 0 ), v4( 1 ), v4( 2 ) );
      }
    }
    glEnd();
    glPopMatrix();
  }
  return;
}

bool
OpenGL_Object_Torus::
_generate_dl( void ){
  if( glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  if( _quadric == NULL ){
    _quadric = gluNewQuadric();
  }
  _dl = glGenLists( 1 );
  glNewList( _dl, GL_COMPILE );
  glColor4f( color()( 0 ), color()( 1 ), color()( 2 ), transparency() );
  glBegin( GL_QUADS );
  for( unsigned int i = 0; i < 32; i++ ){
    double t1 = 2.0 * M_PI * double( i ) / 32.0;
    double t2 = 2.0 * M_PI * double( i + 1 ) / 32.0;
    for( unsigned int j = 0; j < 32; j++ ){
      double p1 = 2.0 * M_PI * double( j ) / 32.0;
      double p2 = 2.0 * M_PI * double( j + 1 ) / 32.0;

      Vector3f v1( ( _major_radius + _minor_radius * cos( p1 ) ) * cos( t1 ),
                    ( _major_radius + _minor_radius * cos( p1 ) ) * sin( t1 ),
                    _minor_radius * sin( p1 ) );
      Vector3f v2( ( _major_radius + _minor_radius * cos( p2 ) ) * cos( t1 ),
                    ( _major_radius + _minor_radius * cos( p2 ) ) * sin( t1 ),
                    _minor_radius * sin( p2 ) );
      Vector3f v3( ( _major_radius + _minor_radius * cos( p2 ) ) * cos( t2 ),
                    ( _major_radius + _minor_radius * cos( p2 ) ) * sin( t2 ),
                      _minor_radius * sin( p2 ) );
      Vector3f v4( ( _major_radius + _minor_radius * cos( p1 ) ) * cos( t2 ),
                      ( _major_radius + _minor_radius * cos( p1 ) ) * sin( t2 ),
                      _minor_radius * sin( p1 ) );
      Vector3f v21 = v2 - v1;
      Vector3f v31 = v3 - v1;
      Vector3f n = v31.cross( v21 );
      glNormal3f( n( 0 ), n( 1 ), n( 2 ) );
      glVertex3f( v1( 0 ), v1( 1 ), v1( 2 ) );
      glVertex3f( v2( 0 ), v2( 1 ), v2( 2 ) );
      glVertex3f( v3( 0 ), v3( 1 ), v3( 2 ) );
      glVertex3f( v4( 0 ), v4( 1 ), v4( 2 ) );
    }
  }
  glEnd();
  glEndList(); 
  return true;
}

namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Torus& other ) {
    return out;
  }

}
